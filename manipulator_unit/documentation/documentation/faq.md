# Veel gestelde vragen

## Wanneer moet ik `colcon build` gebruiken?
Het `colcon build --symlink-install` wordt alleen gebruikt voor de volgende situaties:
* Er zijn bestanden aan een ROS2 package toegevoegd
* Er is een nieuwe ROS2 package gemaakt
* De inhoud van een `C` of `C++` bestand is gewijzigd
* De inhoud van de setup.py van een Python package is gewijzigd
* De inhoud van `package.xml` of `CMakeLists.txt` is gewijzigd

Na de build dien je altijd het volgende commando in `alle openstaande` terminals uit te voeren:
```
source ~/my_uf_ws/install/setup.bash
```