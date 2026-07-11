# Proyecto de Vehículos Autónomos: Controlador Reactivo Follow the Gap (FTG)

Este repositorio contiene el desarrollo del proyecto práctico para el curso de Vehículos Autónomos. Consiste en la implementación de un controlador reactivo basado en el algoritmo **Follow the Gap** utilizando el simulador oficial de **F1TENTH** sobre ROS 2.

## 📘 Descripción General
El objetivo principal es diseñar un algoritmo capaz de procesar lecturas de un sensor LiDAR de un vehículo a escala, identificar la brecha libre (gap) más profunda y segura dentro del entorno, y calcular en tiempo real los comandos de dirección (`steering_angle`) y velocidad (`speed`) para navegar una pista de carreras de forma autónoma sin colisiones.

---

## 🛠️ Estructura del Repositorio

El espacio de trabajo está organizado con los siguientes paquetes en la carpeta `src/`:
* **`f1tenth_gym_ros`**: Entorno oficial del simulador basado en las indicaciones de [widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository).
* **`controllers`**: Paquete de ROS 2 que contiene el nodo con la lógica e implementación de nuestro algoritmo Follow the Gap (FTG).
* **`path_planning`** y **`waypoints`**: Módulos complementarios para la planeación de trayectorias.

---

## 🏎️ Guía de Ejecución

Para iniciar el entorno gráfico en RViz junto con las pistas de competencia y activar el nodo del vehículo con el algoritmo Follow the Gap, ejecuta en terminales separadas:

**Terminal 1: Lanzar el simulador de F1TENTH**
```bash
source ~/F1Tenth-Repository/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
