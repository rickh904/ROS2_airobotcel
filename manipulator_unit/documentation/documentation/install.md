# Installatie van de uFactory Lite6-template

Hier wordt beschreven hoe je de template kunt verkrijgen, kunt bouwen en tenslotte kunt testen.

## Development computer
Als in dit document gesproken wordt over een development-computer dan wordt hiermee bedoeld de laptop/computer waarop je de software in ROS2 ontwikkelt.

## Cloning de ROS2 uFactory Lite6 template
Voor het maken van de ROS2 uFactory Lite6 template maak je gebruik van een Github repository. Je kunt er voor kiezen om deze clone onder een eigen account van Github te plaatsen (1e keuze hieronder). Je kunt daarna eenvoudig backup's van je werk maken naar je eigen Github account.

> we maken gebruik van een prefix `my_ur` in de packages van de repository om onderscheid te maken met de standaard uFactory Lite6 packages.

:::::{card} 

::::{tab-set}

:::{tab-item} Met GIT-repository support

* Maak een account aan bij [Github](https://github.com/) en login op dit account

* Open de [my_ufactory_ROS2](https://github.com/AvansMechatronica/my_ufactory_ROS2) repository

* Maak een Fork van de repository naar je eigen Github account door op het **Fork icoon**  te klikken:

![image](../images/fork.jpg)

* Volg de instructies, maar wijzig de naam van de nieuwe repository niet. Bevestig met **Create Fork**  

* Nu kun je de workspace als volgt creëren

```bash
mkdir -p ~/my_uf_ws/src
cd ~/my_uf_ws/src
git clone https://github.com/<jouw_account_naam>/my_ufactory_ROS2.git
```

*ps. Het gebruik van github (zoals add, commit & push commando's) valt  buiten de scope van deze documentatie*

:::

:::{tab-item} Zonder GIT-repository support

* Je kunt de workspace als volgt creëren
```bash
mkdir -p ~/my_uf_ws/src
cd ~/my_uf_ws/src
git clone https://github.com/AvansMechatronica/my_ufactory_ROS2.git
```

:::

::::

:::::



## Installatie van uFactory Lite6 Robot support packages
Met onderstaand commando worden alle benodigde software voor de template geinstalleerd en de workspace gebouwd met colcon.

```bash
cd ~/my_uf_ws/src/my_ufactory_ROS2/install
./install.bash
```

## Bouwen van de workspace
> Dit is al gebeurd in de installatie. Wijzig je iets in de workspace dan kun je als volgt bouwen.
```bash
# Build the workspace
cd ~/my_uf_ws
colcon build --symlink-install
source install/setup.bash
```

Heb je slechts 1 package gewijzigd dan kun je onderstaand commando gebruiken om betreffende package te bouwen.

```bash
# Build one of the packages in the workspace
cd ~/my_uf_ws
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

## Toevoegen van de `install/setup.bash` aan `.bashrc`
Om te voorkomen dat je ieder keer de workspace `my_uf_ws` moet sourcen wordt de `install/setup.bash` toegevoegd aan `.bashrc` (dit (verborgen)bestand bevindt zich in de `$HOME` directory)
Je kunt het volgende script uitvoeren in een command-console:

```bash
# Add source command to .bashrc if it doesn't already exist
if ! grep -Fxq "source ~/my_uf_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/my_uf_ws/install/setup.bash" >> ~/.bashrc
    echo "Added source command to .bashrc"
else
    echo "Source command already exists in .bashrc"
fi  
```

## Testen van de installatie
Je kunt de installatie testen door onderstaand commando. Je hebt hiervoor geen fysieke robot of simulatie omgeving nodig.

```bash
ros2 launch my_uf_moveit_config demo.launch.py
```
Je kunt nu in RVIZ het model van de robot-applicatie zien en met de knop `Goal State` een positie kiezen en de weg naar de positie volgen met de `Plan` knop. Stel eventueel eerst een `Start State` in.

>Omdat er geen fysieke robot of Gazebo simulatie is kun je geen `Execute` functies uitvoeren.
