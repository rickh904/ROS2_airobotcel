# Gripper

Wanneer de robot is verbonden met ROS2, kunnen we ook de gripper van de robot aansturen. De gripper van de uFactory Lite robot is een parallelle gripper of vacuum gripper die wordt aangedreven door een pompje.

De gripper kan worden bestuurd door middel van een ROS2-service.

Je kunt services bekijken door het volgende commando uit te voeren in een terminal:
```bash
ros2 service list
```

Je zult een service zien met de naam `/xarm/set_vacuum_gripper`. Deze service kan worden gebruikt om de gripper te openen of te sluiten.

Met deze kun je zowel een parallelle gripper als een vacuüm gripper aansturen. 

## Commandline
```bash
ros2 service call /xarm/set_vacuum_gripper xarm_msgs/srv/VacuumGripperCtrl "{'on': false}"
```

```bash
ros2 service call /xarm/set_vacuum_gripper xarm_msgs/srv/VacuumGripperCtrl "{'on': true}"
```