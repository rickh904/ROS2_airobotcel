import depthai as dai
import numpy as np
import time
import cv2

# ROS2: hiermee kunnen we een afbeelding als topic publiceren.
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

blob_path = "/home/student/.cache/blobconverter/best_openvino_2022.1_6shave.blob"

CLASS_NAMES = ["Batterij", "Borstel", "Bout", "Plug"]
CONF_THRES = 0.30
ARUCO_SIZE_MM = 100.0
ARUCO_ID = 0

# --- TOEGEVOEGD: NMS ---
NMS_THRES = 0.45

pipeline = dai.Pipeline()

# -------------------------------------------------
# Camera + YOLO input
# -------------------------------------------------
# RGB-camera op CAM_A.
cam = pipeline.create(dai.node.Camera).build(
    dai.CameraBoardSocket.CAM_A,
    sensorFps=5
)

rgb = cam.requestOutput(
    size=(640, 640),
    type=dai.ImgFrame.Type.RGB888p,
    fps=5
)

# -------------------------------------------------
# YOLO blob
# -------------------------------------------------
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath(blob_path)
rgb.link(nn.input)

# -------------------------------------------------
# Host queues
# -------------------------------------------------
nn_q = nn.out.createOutputQueue(maxSize=1, blocking=False)
rgb_q = rgb.createOutputQueue(maxSize=1, blocking=False)

pipeline.start()

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

print("YOLO + ArUco gestart ✔", flush=True)

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
    "/ai_vision/debug_image",
    10
)

# =================================================
# --- YAW ---
# Bepaalt de hoek van het product in de RGB-camera.
#
# YOLO geeft een axis-aligned bbox.
# Daarom: crop → threshold → grootste contour →
# minAreaRect → hoek van de lange zijde.
# =================================================
def estimate_yaw_from_bbox(frame_rgb, x, y, w, h):
    # --- TOEGEVOEGD ---
    # Maak de crop kleiner dan de YOLO-box. Daardoor pakt
    # minAreaRect minder snel de bbox-rand of achtergrond.
    shrink = 0.60
    w = w * shrink
    h = h * shrink

    # YOLO bbox is center-x, center-y, width, height.
    x1 = int(max(0, x - w / 2))
    y1 = int(max(0, y - h / 2))
    x2 = int(min(frame_rgb.shape[1], x + w / 2))
    y2 = int(min(frame_rgb.shape[0], y + h / 2))

    crop = frame_rgb[y1:y2, x1:x2]

    if crop.size == 0:
        return None

    # RGB → grijs, zodat we object/background kunnen scheiden.
    gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Otsu kiest automatisch een threshold.
    _, thresh = cv2.threshold(
        blur,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Test zowel wit object op donkere achtergrond
    # als donker object op lichte achtergrond.
    contours_normal, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    contours_inverted, _ = cv2.findContours(
        255 - thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    contours = contours_normal + contours_inverted

    if not contours:
        return None

    # Neem de grootste contour: vermoedelijk het product.
    # Kies een contour die niet de hele crop/bounding-box is.
    crop_area = crop.shape[0] * crop.shape[1]

    valid_contours = []

    for c in contours:
        area = cv2.contourArea(c)

        if area < 50:
            continue

    # Als contour bijna de hele crop vult, is het waarschijnlijk achtergrond/crop-rand.
        if area > 0.80 * crop_area:
            continue

        valid_contours.append(c)

    if not valid_contours:
        return None

    contour = max(valid_contours, key=cv2.contourArea)

    # Kleine rommel/ruis negeren.
    if cv2.contourArea(contour) < 50:
        return None

    # Geeft een gedraaide rechthoek om de contour.
    rect = cv2.minAreaRect(contour)
    (_, _), (rect_w, rect_h), angle = rect

    # OpenCV's angle hangt af van welke zijde als "breedte"
    # wordt gezien. We willen de richting van de lange zijde.
    if rect_w < rect_h:
        yaw_deg = angle + 90.0
    else:
        yaw_deg = angle

    # Normaliseer naar [-90, 90).
    # Voor symmetrische producten is 180° verschil hetzelfde.
    while yaw_deg >= 90:
        yaw_deg -= 180

    while yaw_deg < -90:
        yaw_deg += 180

    return float(yaw_deg)


# -------------------------------------------------
# Laatste beschikbare data
# -------------------------------------------------
latest_rgb = None       # --- YAW --- RGB-frame bewaren
latest_aruco = None
last_good_yaw = None
last_print = time.time()

while True:
    # -------------------------------------------------
    # RGB ophalen + bewaren voor yaw + ArUco detecteren
    # -------------------------------------------------
    rgb_msg = rgb_q.tryGet()
    if rgb_msg is not None:
        frame_rgb = rgb_msg.getCvFrame()

        # --- YAW ---
        # Dit frame gebruiken we straks om binnen de YOLO bbox
        # de contour en dus de yaw te bepalen.
        latest_rgb = frame_rgb

        # ArUco gebruikt BGR/grayscale; RGB werkt vaak ook,
        # maar BGR is hier duidelijker.
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

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
    # YOLO output ophalen
    # -------------------------------------------------
    nn_msg = nn_q.tryGet()

    if nn_msg is None:
        time.sleep(0.01)
        continue

    output = nn_msg.getFirstTensor()      # (1, 8, 8400)
    output = np.squeeze(output).T         # (8400, 8)

    boxes = output[:, :4]                 # x, y, w, h
    scores = output[:, 4:]                # class scores

    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)

    mask = confidences > CONF_THRES

    # =================================================
    # --- TOEGEVOEGD: NMS ---
    # YOLO geeft vaak meerdere overlappende boxes voor
    # hetzelfde product. NMS houdt alleen de beste over.
    # =================================================
    filtered_boxes = boxes[mask]
    filtered_class_ids = class_ids[mask]
    filtered_confidences = confidences[mask]

    nms_boxes = []

    for box in filtered_boxes:
        x, y, w, h = box

        # cv2.dnn.NMSBoxes verwacht linksboven-x,
        # linksboven-y, breedte, hoogte.
        nms_boxes.append([
            int(x - w / 2),
            int(y - h / 2),
            int(w),
            int(h)
        ])

    if len(nms_boxes) > 0:
        indices = cv2.dnn.NMSBoxes(
            nms_boxes,
            filtered_confidences.astype(float).tolist(),
            CONF_THRES,
            NMS_THRES
        )
        indices = np.array(indices).flatten() if len(indices) > 0 else []
    else:
        indices = []

    detections = []

    # -------------------------------------------------
    # Per YOLO detectie: werkveld-X/Y en yaw
    # -------------------------------------------------
    # --- AANGEPAST: loop alleen over NMS-overblijvers ---
    for i in indices:
        box = filtered_boxes[i]
        cls_id = filtered_class_ids[i]
        conf = filtered_confidences[i]

        x, y, w, h = box

        # =================================================
        # --- YAW ---
        # Gebruik het laatste RGB-frame en alleen de crop
        # binnen deze YOLO bbox om de producthoek te vinden.
        # =================================================
        yaw_deg = None

        if latest_rgb is not None:
            yaw_candidate = estimate_yaw_from_bbox(latest_rgb, x, y, w, h)

            # Negeer verdachte yaw-sprongen naar exact 0/-90/90
            # als we eerder al een plausibele hoek hadden.
            if yaw_candidate is not None:
                if abs(yaw_candidate) < 2.0 or abs(abs(yaw_candidate) - 90.0) < 2.0:
                    yaw_deg = last_good_yaw
                else:
                    yaw_deg = yaw_candidate
                    last_good_yaw = yaw_candidate
            else:
                yaw_deg = last_good_yaw

        # -------------------------------------------------
        # Pixelpositie → positie t.o.v. ArUco nulpunt
        # -------------------------------------------------
        x_work_mm = None
        y_work_mm = None

        if latest_aruco is not None:
            object_px = np.array([x, y], dtype=np.float32)
            rel_px = object_px - latest_aruco["center"]

            x_work_mm = float(
                np.dot(rel_px, latest_aruco["x_axis"])
                * latest_aruco["mm_per_px"]
            )

            y_work_mm = float(
                np.dot(rel_px, latest_aruco["y_axis"])
                * latest_aruco["mm_per_px"]
            )

        detections.append({
            "class_id": int(cls_id),
            "class_name": CLASS_NAMES[int(cls_id)],
            "confidence": float(conf),
            "x_px": float(x),
            "y_px": float(y),
            "x_work_mm": x_work_mm,
            "y_work_mm": y_work_mm,

            # --- YAW ---
            "yaw_deg": yaw_deg,

            # --- bbox-grootte
            "w_px": float(w),
            "h_px": float(h),
        })

    # -------------------------------------------------
    # Debug image voor HMI
    #
    # Dit maakt een kopie van het RGB-beeld, tekent daarop
    # bounding boxes + detectiedata, en publiceert die als
    # ROS2 Image-topic naar /ai_vision/debug_image.
    # -------------------------------------------------
    if latest_rgb is not None:
        debug_frame = latest_rgb.copy()

        for d in detections:
            # YOLO geeft center-x, center-y, width, height.
            x1 = int(d["x_px"] - d["w_px"] / 2)
            y1 = int(d["y_px"] - d["h_px"] / 2)
            x2 = int(d["x_px"] + d["w_px"] / 2)
            y2 = int(d["y_px"] + d["h_px"] / 2)

            # Zorg dat de box binnen het beeld blijft.
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(debug_frame.shape[1] - 1, x2)
            y2 = min(debug_frame.shape[0] - 1, y2)

            # Bounding box tekenen.
            cv2.rectangle(
                debug_frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            # Yaw kan None zijn als contouranalyse faalt.
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

            # Eerste regel boven de bbox.
            cv2.putText(
                debug_frame,
                label_text,
                (x1, max(20, y1 - 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

            # Tweede regel boven de bbox.
            cv2.putText(
                debug_frame,
                position_text,
                (x1, max(40, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (0, 255, 0),
                1
            )

        # Numpy/OpenCV RGB-image -> ROS2 sensor_msgs/Image.
        image_msg = bridge.cv2_to_imgmsg(
            debug_frame,
            encoding="rgb8"
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
            # --- YAW ---
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