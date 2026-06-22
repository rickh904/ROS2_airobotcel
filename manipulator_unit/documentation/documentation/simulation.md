# Simulatie uFactory Lite robot

De `Gazebo` simulatie kan als volgt worden opgestart:

```
ros2 launch my_uf_bringup simulation.launch.py
```

Tevens zal RVIZ worden opgestart en een virtuele weergave van de (simulatie)robot-opstelling wordt nu zichtbaar. 
De stand van de robot in de virtuele rviz-wereld moet overeen komen met de stand van de uFactory Lite robot in Gazebo.

Je kunt de robot nu laten bewegen door het selecteren van een pose met de knop `Goal State` een positie kiezen en de weg naar de positie volgen met de `Plan` knop. Vervolgens kun je `Plan & Execute` of `Execute` bedienen waarna de robot zal bewegen naar de gekozen pose.