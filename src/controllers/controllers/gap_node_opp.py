#!/usr/bin/env python3

# ============================================================
# gap_node_opp.py — Follow the Gap (Versión Oponente - Más Lento)
#
# Navegación reactiva: en cada scan del LIDAR el robot busca
# el espacio libre más grande frente a él y se dirige a su centro.
# Pasos: preprocesar → punto más cercano → burbuja → gap más
# grande → punto objetivo → publicar dirección y velocidad.
# ============================================================

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGapOpp(Node): # Nombre de clase único
    def __init__(self):
        super().__init__('reactive_follow_gap_opp_node') # Nombre de nodo único para ROS 2
        self.get_logger().info("¡Nodo Oponente (Lento) iniciado correctamente!")

        # --- SUSCRIPCIÓN Y PUBLICADOR PARA EL OPONENTE ---
        self.create_subscription(LaserScan, '/opp_scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

        # Odometría del oponente para monitoreo de vueltas
        self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)

        # --- PARÁMETROS AJUSTABLES OPTIMIZADOS ---
        self.rango_max = 11.0         # m — Recorte dinámico para anticipar curvas a velocidad
        self.radio_burbuja = 75       # nº de rayos a poner en cero alrededor del obstáculo cercano
        self.ventana_suavizado = 3    # nº de rayos para la media móvil preliminar
        self.umbral_gap = 1.2          # m — un rayo cuenta como "libre" si supera este valor

        # Recorte del campo de visión (FOV frontal optimizado a ~85°)
        self.fov_recorte = math.radians(85)   

        # --- CONFIGURACIÓN DE VELOCIDADES REDUCIDAS ---
        self.vel_recta = 4.5          # m/s en tramos completamente despejados (Antes 6.8)
        self.vel_curva = 1.50         # m/s velocidad mínima en giros cerrados (Antes 1.95)

        # Anti-oscilación (Filtro Exponencial)
        self.zona_muerta = math.radians(1.5)  # Umbral mínimo real para evitar zigzagueo fino
        self.alpha_suavizado = 0.35           # Peso del ángulo nuevo
        self.steering_previo = 0.0           # Último ángulo publicado

        # Índices del sector frontal — se calculan en el primer scan
        self.idx_inicio = None
        self.idx_fin = None

        # --- CONTADOR Y CRONÓMETRO DE VUELTAS (Oponente) ---
        self.pos_salida = None        
        self.num_vuelta = 0           
        self.tiempo_vuelta_ini = None 
        self.salio_de_salida = False  
        self.umbral_lejos = 5.0       
        self.umbral_cerca = 2.0       

    # ----------------------------------------------------------
    def preprocess_lidar(self, ranges):
        """ Limpia y acota el horizonte del scan crudo """
        proc = np.array(ranges, dtype=np.float64)

        # inf y NaN → 0 (los tratamos como obstáculo inmediato)
        proc[np.isinf(proc)] = 0.0
        proc[np.isnan(proc)] = 0.0

        # Acotar la distancia máxima para reaccionar a tiempo a la velocidad de carrera
        proc[proc > self.rango_max] = self.rango_max

        # Suavizado preliminar por media móvil
        if self.ventana_suavizado > 1:
            kernel = np.ones(self.ventana_suavizado) / self.ventana_suavizado
            proc = np.convolve(proc, kernel, mode='same')

        return proc

    # ----------------------------------------------------------
    def find_max_gap(self, free_space_ranges):
        """ Encuentra la secuencia continua más larga de valores libres """
        libre = free_space_ranges > self.umbral_gap

        mejor_inicio, mejor_fin = 0, 0   
        mejor_largo = 0
        inicio_actual = None             

        for i, es_libre in enumerate(libre):
            if es_libre:
                if inicio_actual is None:
                    inicio_actual = i
            else:
                if inicio_actual is not None:
                    largo = i - inicio_actual
                    if largo > mejor_largo:
                        mejor_largo = largo
                        mejor_inicio, mejor_fin = inicio_actual, i - 1
                    inicio_actual = None

        if inicio_actual is not None:
            largo = len(libre) - inicio_actual
            if largo > mejor_largo:
                mejor_inicio, mejor_fin = inicio_actual, len(libre) - 1

        return mejor_inicio, mejor_fin

    # ----------------------------------------------------------
    def find_best_point(self, start_i, end_i):
        """ Retorna el centro geométrico del gap seleccionado """
        return (start_i + end_i) // 2

    # ----------------------------------------------------------
    def odom_callback(self, msg):
        """ Monitoreo y telemetría de tiempos de vuelta del Oponente """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.pos_salida is None:
            self.pos_salida = (x, y)
            self.tiempo_vuelta_ini = time.time()
            self.get_logger().info("[OPONENTE] Cronómetro iniciado — Esperando primera vuelta...")
            return

        dx = x - self.pos_salida[0]
        dy = y - self.pos_salida[1]
        dist_salida = math.sqrt(dx * dx + dy * dy)

        if not self.salio_de_salida:
            if dist_salida > self.umbral_lejos:
                self.salio_de_salida = True
        else:
            if dist_salida < self.umbral_cerca:
                ahora = time.time()
                tiempo_vuelta = ahora - self.tiempo_vuelta_ini
                self.num_vuelta += 1
                self.get_logger().info(
                    f"[OPONENTE] VUELTA {self.num_vuelta} completada — tiempo: {tiempo_vuelta:.2f} s"
                )
                self.tiempo_vuelta_ini = ahora
                self.salio_de_salida = False

    # ----------------------------------------------------------
    def lidar_callback(self, data):
        """ Ejecuta la canalización clásica de Follow the Gap """
        angle_min = data.angle_min
        angle_increment = data.angle_increment

        # Configuración adaptativa de índices frontales en el primer callback
        if self.idx_inicio is None:
            centro = len(data.ranges) // 2
            n_rayos = int(self.fov_recorte / angle_increment)
            self.idx_inicio = max(0, centro - n_rayos)
            self.idx_fin = min(len(data.ranges) - 1, centro + n_rayos)

        # 1) Preprocesamiento
        proc = self.preprocess_lidar(data.ranges)
        frente = proc[self.idx_inicio:self.idx_fin + 1]

        # 2) Encontrar obstáculo crítico (Excluyendo falsos ceros)
        valid_indices = np.where(frente > 0.1)[0]
        if len(valid_indices) > 0:
            idx_cercano = valid_indices[np.argmin(frente[valid_indices])]
            
            # 3) Aplicar burbuja de seguridad sobre el array frontal
            ini_burbuja = max(0, idx_cercano - self.radio_burbuja)
            fin_burbuja = min(len(frente), idx_cercano + self.radio_burbuja)
            frente[ini_burbuja:fin_burbuja] = 0.0

        # 4) Buscar el hueco más profundo disponible
        gap_inicio, gap_fin = self.find_max_gap(frente)

        # 5) Seleccionar punto objetivo
        idx_objetivo = self.find_best_point(gap_inicio, gap_fin)

        # Mapeo de índices de regreso al marco global del LIDAR
        idx_global = idx_objetivo + self.idx_inicio
        steering_angle = angle_min + idx_global * angle_increment

        # --- FILTROS DE ANTI-OSCILACIÓN ---
        if abs(steering_angle) < self.zona_muerta:
            steering_angle = 0.0

        # Filtro de paso bajo exponencial
        steering_angle = (self.alpha_suavizado * steering_angle +
                          (1.0 - self.alpha_suavizado) * self.steering_previo)
        self.steering_previo = steering_angle

        # Límite físico de dirección del coche F1TENTH (±0.41 Rad5)
        steering_angle = max(-0.41, min(0.41, steering_angle))

        # 6) Gestión de Velocidad Dinámica Continua
        factor_giro = abs(steering_angle) / 0.41
        speed = self.vel_recta - (self.vel_recta - self.vel_curva) * factor_giro

        # Publicar los comandos Ackermann finales para el oponente
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'opp_racecar' # Frame correspondiente al oponente
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    reactive_follow_gap_node = ReactiveFollowGapOpp()
    try:
        rclpy.spin(reactive_follow_gap_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_follow_gap_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()