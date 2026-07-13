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

## 🚀 Flujo de Ejecución Rápido
Sigue estos pasos en tu terminal para limpiar, compilar y lanzar la simulación completa con un solo comando:

Navegar y realizar una compilación limpia del paquete:
```bash
cd ~/F1Tenth-Repository
rm -rf build/controllers/ install/controllers/
colcon build --packages-select controllers
source install/setup.bash
```

