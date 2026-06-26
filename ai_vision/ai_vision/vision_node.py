import sys

VENV_SITE_PACKAGES = "/home/student/depthai_env/lib/python3.12/site-packages"

if VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

import depthai as dai
import numpy as np
import time
import cv2
from interfaces.srv import CoordRef
from ultralytics import YOLO

# ROS2: hiermee kunnen we een afbeelding als topic publiceren.
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

model_path = "/home/student/ROS2_airobotcel/ai_vision/resources/models/best.pt"

CLASS_NAMES = ["Batterij", "Borstel", "Bout", "Plug"]
CONF_THRES = 0.65
ARUCO_SIZE_MM = 100.0
ARUCO_ID = 0

pipeline = dai.Pipeline()

# -------------------------------------------------
# Camera input
# -------------------------------------------------
# RGB-camera op CAM_A.
cam = pipeline.create(dai.node.Camera).build(
    dai.CameraBoardSocket.CAM_A,
    sensorFps=10
)

rgb = cam.requestOutput(
    size=(640, 640),
    type=dai.ImgFrame.Type.RGB888p,
    fps=10
)

# -------------------------------------------------
# Host queues
# -------------------------------------------------
rgb_q = rgb.createOutputQueue(maxSize=1, blocking=False)

pipeline.start()

# -------------------------------------------------
# YOLO OBB model op host/VM
# -------------------------------------------------
model = YOLO(model_path)

# -------------------------------------------------
# ArUco detector
# -------------------------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

if hasattr(cv2.aruco, "ArucoDetector"):
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
else:
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_detector = None

print("YOLO OBB + ArUco gestart ✔", flush=True)

# -------------------------------------------------
# ROS2 publisher voor de HMI
# -------------------------------------------------
rclpy.init()

ros_node = rclpy.create_node("ai_vision_node")

# CvBridge zet een OpenCV/Numpy-afbeelding om naar sensor_msgs/Image.
bridge = CvBridge()

# De HMI gaat later op dit topic subscriben.
debug_image_pub = ros_node.create_publisher(
    Image,
    "cam",
    1
)


# =================================================
# --- YAW (GEAANGEPAST VOOR STABIELE GRIJPBEWEGINGEN) ---
# Rond een gemeten hoek af naar stappen van 15 graden.
# Normaliseert stabiel naar [-90, 90] graden voor de transformatie-node.
# =================================================
def quantize_yaw_15deg(yaw_deg):
    if yaw_deg is None:
        return None

    # Breng de hoek naar een positief bereik [0, 180)
    yaw_deg = yaw_deg % 180.0
    step = 15.0

    # Rond af op de dichtstbijzijnde stap van 15 graden
    quantized = float((int((yaw_deg + step / 2) // step) * step) % 180.0)

    # Breng weer terug naar het stabiele gripper-bereik [-90, 90] voor de transformatie-node
    if quantized > 90.0:
        quantized -= 180.0
        
    return quantized


# -------------------------------------------------
# Laatste beschikbare data
# -------------------------------------------------
latest_rgb = None       # --- YAW / DEBUG --- RGB-frame bewaren
latest_aruco = None
latest_best_detection = None   # --- SERVICE --- laatste beste detectie bewaren
last_print = time.time()


# -------------------------------------------------
# Service voor hoofdcontroller
# -------------------------------------------------
def handle_coord_ref(request, response):
    if latest_best_detection is None:
        response.success = False
        response.message = "Geen detectie beschikbaar"
        return response

    d = latest_best_detection

    if d["x_work_mm"] is None or d["y_work_mm"] is None:
        response.success = False
        response.message = "AruCo niet gevonden, geen werkveldcoordinaten beschikbaar"
        return response

    if d["yaw_deg"] is None:
        response.success = False
        response.message = "Yaw niet beschikbaar"
        return response

    response.success = True
    response.class_name = d["class_name"]
    response.class_id = int(d["class_id"])
    response.confidence = float(d["confidence"])
    response.x_mm = float(d["x_work_mm"])
    response.y_mm = float(d["y_work_mm"])
    response.yaw_deg = float(d["yaw_deg"])
    response.width_px = float(d["w_px"])
    response.height_px = float(d["h_px"])
    response.message = "Objectpositie beschikbaar"

    return response


coord_ref_service = ros_node.create_service(
    CoordRef,
    "/ai_vision/coord_ref",
    handle_coord_ref
)

DETECT_EVERY_N_FRAMES = 5
frame_counter = 0
last_detections = []

while True:
    # -------------------------------------------------
    # RGB ophalen + bewaren voor YOLO OBB + ArUco
    # -------------------------------------------------
    rgb_msg = rgb_q.tryGet()

    if rgb_msg is None:
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        time.sleep(0.01)
        continue

    frame_rgb = rgb_msg.getCvFrame()
    latest_rgb = frame_rgb
    frame_counter += 1
    run_detection = (frame_counter % DETECT_EVERY_N_FRAMES == 0)

    # ArUco en Ultralytics werken prettiger met BGR.
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    # -------------------------------------------------
    # ArUco detecteren
    # -------------------------------------------------
    if aruco_detector is not None:
        corners, ids, _ = aruco_detector.detectMarkers(frame_bgr)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame_bgr,
            aruco_dict,
            parameters=aruco_params
        )

    latest_aruco = None

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if ARUCO_ID is not None and marker_id != ARUCO_ID:
                continue

            pts = marker_corners[0].astype(np.float32)

            # ArUco corners: top-left, top-right,
            # bottom-right, bottom-left.
            p0, p1, p2, p3 = pts
            center = np.mean(pts, axis=0)

            x_axis = p1 - p0
            y_axis = p3 - p0

            aruco_width_px = np.linalg.norm(x_axis)
            aruco_height_px = np.linalg.norm(y_axis)
            aruco_size_px = (aruco_width_px + aruco_height_px) / 2.0

            if aruco_size_px <= 0:
                continue

            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)

            mm_per_px = ARUCO_SIZE_MM / aruco_size_px

            latest_aruco = {
                "id": int(marker_id),
                "center": center,
                "x_axis": x_axis,
                "y_axis": y_axis,
                "mm_per_px": mm_per_px,
            }

            break

    # -------------------------------------------------
    # YOLO OBB detectie op host/VM
    # -------------------------------------------------
    if run_detection:

        results = model.predict(
            source=frame_rgb,
            imgsz=320,
            conf=CONF_THRES,
            verbose=False
        )

        result = results[0]
        detections = []

        if result.obb is not None:
            # result.obb.xywhr:
            # x_center, y_center, width, height, rotation_radians
            xywhr_list = result.obb.xywhr.cpu().numpy()
            cls_list = result.obb.cls.cpu().numpy()
            conf_list = result.obb.conf.cpu().numpy()

            # De vier hoekpunten van elke gedraaide bounding box.
            obb_points_list = result.obb.xyxyxyxy.cpu().numpy()

            for xywhr, cls_id, conf, obb_points in zip(
                xywhr_list,
                cls_list,
                conf_list,
                obb_points_list
            ):
                x, y, w, h, angle_rad = xywhr

                cls_id = int(cls_id)
                confidence = float(conf)

                # OBB-hoek in graden.
                yaw_raw_deg = float(np.degrees(angle_rad))

                # Altijd de lange zijde als richting gebruiken (voorkomt flip).
                if w < h:
                    yaw_raw_deg += 90.0

                # GECORRIGEERD: Forceer de ruwe hoek direct stabiel tussen -90 en 90 graden
                # Dit voorkomt dat hoeken rond de 95 graden fout overspringen in de transformatie-node.
                yaw_raw_deg = (yaw_raw_deg + 90.0) % 180.0 - 90.0

                # Afronden naar 0, 15, 30, ..., of negatieve stappen.
                yaw_deg = quantize_yaw_15deg(yaw_raw_deg)

                # Pixelpositie → positie t.o.v. ArUco.
                x_work_mm = None
                y_work_mm = None

                if latest_aruco is not None:
                    object_px = np.array([x, y], dtype=np.float32)
                    rel_px = object_px - latest_aruco["center"]

                    aruco_x_mm = float(
                        np.dot(rel_px, latest_aruco["x_axis"])
                        * latest_aruco["mm_per_px"]
                    )

                    aruco_y_mm = float(
                        np.dot(rel_px, latest_aruco["y_axis"])
                        * latest_aruco["mm_per_px"]
                    )

                    x_work_mm = aruco_x_mm
                    y_work_mm = aruco_y_mm

                class_name = (
                    CLASS_NAMES[cls_id]
                    if cls_id < len(CLASS_NAMES)
                    else str(cls_id)
                )

                detections.append({
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "x_px": float(x),
                    "y_px": float(y),
                    "x_work_mm": x_work_mm,
                    "y_work_mm": y_work_mm,
                    "yaw_deg": yaw_deg,
                    "w_px": float(w),
                    "h_px": float(h),
                    "obb_points": obb_points.astype(np.int32),
                })

        # Deze lijst wordt alleen vernieuwd wanneer er echt inference is gedaan.
        last_detections = detections

    else:
        # Tussen twee inference-runs: behoud de vorige bounding box
        # zodat het debugbeeld wel live blijft verversen.
        detections = last_detections

    # -------------------------------------------------
    # Alleen detectie met hoogste confidence behouden
    # -------------------------------------------------
    if len(detections) > 0:
        best_detection = max(
            detections,
            key=lambda d: d["confidence"]
        )

        detections = [best_detection]

    # -------------------------------------------------
    # --- SERVICE ---
    # Pas NÁ de detectie kiezen we één beste detectie.
    # -------------------------------------------------
    valid_for_service = [
        d for d in detections
        if d["x_work_mm"] is not None
        and d["y_work_mm"] is not None
        and d["yaw_deg"] is not None
    ]

    if len(valid_for_service) > 0:
        latest_best_detection = max(
            valid_for_service,
            key=lambda d: d["confidence"]
        )

    # -------------------------------------------------
    # Debug image voor HMI
    # -------------------------------------------------
    debug_frame = latest_rgb.copy()

    for d in detections:
        # Gedraaide OBB-box tekenen.
        cv2.polylines(
            debug_frame,
            [d["obb_points"]],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2
        )

        x_text = int(d["x_px"])
        y_text = int(d["y_px"])

        # Yaw kan None zijn als er iets faalt.
        if d["yaw_deg"] is None:
            yaw_text = "yaw=?"
        else:
            yaw_text = f"yaw={d['yaw_deg']:.1f} deg"

        # Als ArUco niet zichtbaar is, bestaan werkveld-X/Y niet.
        if d["x_work_mm"] is None:
            position_text = yaw_text
        else:
            position_text = (
                f"x={d['x_work_mm']:.0f} "
                f"y={d['y_work_mm']:.0f} "
                f"| {yaw_text}"
            )

        label_text = (
            f"{d['class_name']} "
            f"{d['confidence']:.2f}"
        )

        # Eerste regel bij het object.
        cv2.putText(
            debug_frame,
            label_text,
            (x_text, max(20, y_text - 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        # Tweede regel bij het object.
        cv2.putText(
            debug_frame,
            position_text,
            (x_text, max(40, y_text - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 0),
            1
        )

    # Numpy/OpenCV RGB-image -> ROS2 sensor_msgs/Image.
    image_msg = bridge.cv2_to_imgmsg(
        debug_frame,
        encoding="bgr8"
    )

    image_msg.header.stamp = ros_node.get_clock().now().to_msg()
    image_msg.header.frame_id = "oak_rgb_camera"

    debug_image_pub.publish(image_msg)

    # ROS2 callbacks verwerken zonder je vision-loop te blokkeren.
    rclpy.spin_once(ros_node, timeout_sec=0.0)

    # -------------------------------------------------
    # Console-output
    # -------------------------------------------------
    if time.time() - last_print > 0.5:
        print("----", flush=True)

        if latest_aruco is None:
            print("AruCo: niet gevonden", flush=True)
        else:
            print(
                f"AruCo id={latest_aruco['id']} "
                f"mm_per_px={latest_aruco['mm_per_px']:.3f}",
                flush=True
            )

        for d in detections[:5]:
            yaw_text = (
                "None"
                if d["yaw_deg"] is None
                else f"{d['yaw_deg']:.1f} deg"
            )

            if d["x_work_mm"] is None:
                print(
                    f"{d['class_name']} "
                    f"conf={d['confidence']:.2f} "
                    f"x_px={d['x_px']:.1f} "
                    f"y_px={d['y_px']:.1f} "
                    f"yaw={yaw_text}",
                    flush=True
                )
            else:
                print(
                    f"{d['class_name']} "
                    f"conf={d['confidence']:.2f} "
                    f"x={d['x_work_mm']:.1f} mm "
                    f"y={d['y_work_mm']:.1f} mm "
                    f"yaw={yaw_text}",
                    flush=True
                )

        last_print = time.time()

    time.sleep(0.01)