# MoveIt Configuratie

In de template is een MoveIt Configuratie opgenomen die het mogelijk maakt de robot(virtueel of in het echt) te besturen met de `movegroup`-node.

De movegroup kan worden opgestart met het volgende commando:

```bash
ros2 launch my_uf_bringup movegroup.launch.py
```
>Uiteraard moet eerst de simualtie of robot gestart zijn.


## MoveIt Setup Assistant
Je kunt de MoveIt configuratie wijzigen met de `MoveIt Setup assistant`:

```
ros2 launch my_uf_moveit_config setup_assistant.launch.py 
```

In de `MoveIt Setup assistant` zijn alleen de tab-bladen `Self-Collisions` en `Robot Poses` van belang. Wijzig van andere tab-bladen de inhoud `MoveIt Setup assistant`niet, dit kan er voor zorgen dat je MoveIt configuratie niet meer werkt. Zorg er tevens voor dat bij `Configuration Files` alleen het bestand met `.srdf` geselecteerd is.

## Herstel van MoveIt Configuratie
Mocht om enige reden je MoveIt configuratie beschadigd zijn geraakt dan kun je dit herstellen door de inhoud uit de backup folder terug te zetten.
```bash
cp -v ~/my_uf_ws/src/my_ufactory_ROS2/my_uf_moveit_config/config/backup/*.* ~/my_uf_ws/src/my_ufactory_ROS2/my_uf_moveit_config/config
```



