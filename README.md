# EKF_Localization

Este proyecto implementa y simula un sistema de localización 3D para un dron usando un Filtro de Kalman Extendido (EKF) en ROS2. El sistema fusiona datos de GPS, odometría, balizas y altímetro, siendo robusto ante la pérdida temporal de cualquier sensor. Incluye simulación de sensores, visualización en RViz y herramientas para analizar los resultados de los experimentos.

## Instalación y uso rápido

1. **Crea un workspace ROS2 y clona el repositorio:**
   ```bash
   mkdir -p ar_ws/src
   cd ar_ws/src
   git clone https://github.com/lolahzc/EKF_Localization.git
   cd ..
   ```
2. **Configura RViz:**
   Edita la ruta `rviz_config_path` en `kalman_filters/launch/ekf_launch.py` para que apunte a tu archivo `.rviz`.

3. **Compila:**
   ```bash
   colcon build
   source install/setup.bash
   ```
4. **Lanza la simulación y el EKF:**
   ```bash
   ros2 launch kalman_filters ekf_launch.py
   ```

## Estructura
- `kalman_filters/`: Código del EKF y launch files.
- `moving_point/`: Simulación de sensores y entorno.
- `graficas/`: Figuras de los resultados de los experimentos realizados.

## Experimentos
1. **Caso ideal:** Balizas con 6 m de rango, 90% acierto de envío de mensajes en entorno radioeléctrico.
2. **Entorno radioeléctrico medio:** 60% acierto.
3. **Balizas con poco rango:** 4 m.
4. **Túnel:** GPS desactivado.
5. **Más ruido:** Q=0.8, Rgps=0.3.
