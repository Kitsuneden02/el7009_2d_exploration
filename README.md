# Proyecto N°3: Exploración basada en fronteras de ambientes 2D

Este repositorio contiene la implementación del Proyecto N°3 del curso **EL7009 – Robótica Móvil**. El objetivo es integrar un sistema de exploración autónoma basado en detección de fronteras, utilizando **slam_toolbox** y **Nav2**, sobre ROS 2 y simulación en Gazebo.

El robot navega de forma autónoma hacia las fronteras del mapa, generando metas basadas en zonas de transición entre espacio conocido y desconocido, mapeando el entorno 2D sin intervención humana.

---

### Instalación
OBS: Requiere ROS2 Jazzy instalado previamente.

```
# 1) Clonar repositorio
git clone https://github.com/Kitsuneden02/el7009_2d_exploration
cd el7009_2d_exploration

# 1.1) Ejecutar instalador si lo desea
./install.sh

# 2) Instalar dependencias necesarias
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup

# (Opcional) Instalar dependencias Python
pip install -r requirements.txt

# 3) Compilar workspace
colcon build --symlink-install

# 4) Sourcear workspace
source install/setup.bash
```

---

### Uso

#### 1) Lanzar robot y simulación en Gazebo
```
ros2 launch el7009_diff_drive_robot robot.launch.py
```
#### 2) Lanzar SLAM y Nav2
```
ros2 launch frontier_exploration slam_n_nav.launch.py
```
IMPORTANTE: Se debe esperar a que termine de cargar antes de seguir con el siguiente launch. Cuando esté completamente cargado se debería apreciar lo siguiente en Rviz:
<img width="591" height="375" alt="Screenshot from 2025-07-29 22-04-12" src="https://github.com/user-attachments/assets/7703a3e9-8030-42fc-88cd-67fac839c5ae" />
#### 3) Lanzar explorador de fronteras
```
ros2 launch frontier_exploration exploration.launch.py
```

#### Extra) Sourcear modelos
Si se desea usar hospital, se deben hacer las respectivas modificaciones en `robot.launch.py`. Además también deberá exportar el path a los modelos antes de ejecutar todo el sistema:
```
export GZ_SIM_RESOURCE_PATH=/path/to/el7009_2d_exploration/src/el7009_diff_drive_robot/models/hospital/model
```

---

### Créditos y referencias

* [slam\_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [Nav2 – ROS 2 Navigation Stack](https://github.com/ros-planning/navigation2)
* [TurtleBot3 Simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
* [Dataset of Gazebo Worlds and Maps – Michael Herd](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)
* Yamauchi, B. (1997). *A Frontier-Based Approach for Autonomous Exploration*. CIRA'97.

