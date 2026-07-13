# F1TENTH Multi-Agent Racing & Follow the Gap Simulator

Este repositorio contiene el entorno de simulación y los controladores autónomos desarrollados para carreras **Multi-Agente (2 vehículos simultáneos)** en el circuito modificado de **Oschersleben**, utilizando ROS 2 (Humble) y el simulador oficial de F1TENTH.

El proyecto implementa y optimiza el algoritmo de navegación reactiva **Follow the Gap**, permitiendo que un vehículo principal compita de forma segura y realice adelantamientos frente a un oponente dinámico.

---

## 📂 Estructura del Repositorio

El proyecto se organiza dentro del espacio de trabajo de ROS 2 de la siguiente manera:

```text
F1Tenth-Repository/
├── src/
│   ├── f1tenth_gym_ros/          # Simulador oficial de F1TENTH
│   │   ├── config/
│   │   │   └── sim.yaml          # Configuración del entorno y posiciones de salida
│   │   └── maps/
│   │       └── Oschersleben_map_obs/  # Archivos del mapa modificado (.yaml, .png)
│   └── controllers/              # Paquete principal de control autónomo
│       ├── package.xml
│       ├── setup.py              # Registro de puntos de entrada de ROS 2
│       ├── controllers/
│       │   ├── __init__.py
│       │   ├── reactive_gap_follow.py  # Controlador del Carro Principal (Rápido)
│       │   └── gap_node_opp.py         # Controlador del Oponente (Lento)
│       └── launch/
│           └── race_launch.py    # Lanzamiento automatizado del entorno completo
└── README.md
```
## 🛠️ Aspectos Clave de la Configuración.

1. Entorno Multi-Agente (sim.yaml)El simulador está configurado para levantar de manera nativa dos chasis independientes bajo los namespaces /ego_racecar y /opp_racecar. Las poses iniciales en la grilla de salida están dispuestas en fila para evitar colisiones críticas en el instante de arranque:Carro Principal (Ego): Ubicado en el origen de la pista ($X=0.0$, $Y=0.0$, $\theta=0.0$).Carro Oponente (Opp): Posicionado de forma adelantada ($X=9.5$, $Y=8.5$, $\theta=3.14$).

2. Optimizaciones del Controlador Principal (reactive_gap_follow.py)
Para garantizar un desempeño competitivo en Oschersleben, el algoritmo base fue mejorado mediante:

    Filtro Anti-Oscilación: Implementación de un suavizado exponencial por media móvil ($\alpha = 0.50$) en el ángulo de dirección para mitigar movimientos violentos del servo en rectas.

    Ajuste del Campo Visual: Recorte del rango máximo del LiDAR a 6.8 metros para forzar al vehículo a anticipar los vértices de las curvas cerradas en lugar de seguir falsas salidas.

    Ensanchamiento de Seguridad: Incremento del radio de la burbuja a 52 índices alrededor del obstáculo más cercano, eliminando por completo los roces físicos (wall scraping) contra los muros externos.

3. Oponente Dinámico Móvil (gap_node_opp.py)
El segundo vehículo actúa como un obstáculo móvil predecible. Utiliza una configuración de seguridad conservadora y corre a una velocidad regulada de 4.2 m/s, sirviendo como entorno de pruebas en tiempo real para las maniobras de adelantamiento del carro principal.

---

## 🚀 Flujo de Ejecución y Despliegue

Para poner en marcha la simulación multi-agente, se necesitan únicamente **dos terminales independientes**. El primer bloque compila el entorno y levanta la interfaz gráfica, mientras que el segundo ejecuta ambos vehículos en paralelo usando un hilo de fondo.

### Terminal 1: Compilación y Lanzamiento del Simulador (RViz)
Este bloque realiza una limpieza completa de los residuos de compilaciones previas, regenera los enlaces simbólicos y arranca el entorno gráfico forzando la compatibilidad de OpenGL.

```bash
cd ~/F1Tenth-Repository
rm -rf build/ install/
colcon build --symlink-install
source install/setup.bash
MESA_GL_VERSION_OVERRIDE=3.3 ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
### Terminal 2: Activación en Paralelo de Ambos Vehículos
Una vez que RViz esté completamente abierto y muestre los dos chasis estáticos en la grilla de salida, ejecuta este bloque en una segunda pestaña. El operador & se encarga de enviar al carro principal al fondo para que ambos corran simultáneamente sin bloquear la terminal.
```bash
cd ~/F1Tenth-Repository
source install/setup.bash

# Lanzar el carro principal (Rápido) en segundo plano
ros2 run controllers reactive_gap_follow &

# Lanzar el carro oponente (Lento) en primer plano
ros2 run controllers gap_node_opp
```

---

## 📺 Demostración en Video

A continuación se presenta un video con la ejecución en tiempo real del algoritmo **Follow the Gap** en el entorno multi-agente, donde se evidencia el comportamiento del vehículo principal realizando adelantamientos limpios y evitando colisiones:

[▶️ Ver Video de la Ejecución en Simulación](https://www.youtube.com/watch?v=kiFlOH-mIZ0)

---

## 👤 Autor y Contacto

Este proyecto fue desarrollado como parte de las actividades prácticas en el ámbito de la robótica autónoma y sistemas de control interactivos.

* **Nombre:** Marcos Emmanuel Balón García
* **Rol:** Estudiante de Ingeniería en Mecatrónica (6to Semestre)
* **Clase:** Vehìculos no tripulados
* **Universidad:** Escuela Superior Politécnica del Litoral
* **Contacto:** [GitHub Profile](https://github.com/Marcos-6783) | mebalon@espol.edu.ec / marcosemmanuelbalon@gmail.com

---

