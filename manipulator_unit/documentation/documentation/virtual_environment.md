# Virtuele omgeving

De virtuele omgeving welke zichtbaar is in RVIZ of Gazebo is gemodelleerd in een URDF-bestand.

Het URDF-bestand kun je hier vinden:

```bash
~/<workspace>/src/my_ufactory_ROS2/my_uf_description/urdf/uf_robot.urdf.xacro
```

In dit bestand kun je virtuele elementen toevoegen aan je virtuele omgeving die overeenkomen met je fysieke robot-cel.

Je kunt de omgeving valideren met:
```bash
ros2 launch my_uf_description view_environment.launch.py
```

>Nadat je de virtuele omgeving hebt gewijzigd dien je de `Self-Collisions` in de MoveIt-configuratie met de `MoveIt Setup assistant` opnieuw te genereren.