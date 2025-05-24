# EKF_Localization

## Instrucciones

Dejo los pasos a seguir por si no os va algo u os hacéis la picha un lío.

1. Os hacéis un workspace nuevo donde queráis de vuestro ordenador.
    - mkdir ar_ws
    - cd ar_ws
    - mkdir src
    - cd src

2. Os clonáis este repositorio en el que solo están los paquetes. 
    - Yo utilizo ssh pero vosotros creo que estáis acostumbrados a hacer https
        - git clone git@github.com:lolahzc/EKF_Localization.git
        - git clone https://github.com/lolahzc/EKF_Localization.git

3. Antes de seguir os tenéis que meter en ~/kalman_filters/launch/ekf_launch.py. Aquí cambiáis la ruta que sale en **rviz_config_path** por vuestra ruta.

4. Ahora ya podéis ir a la carpeta del workspace y hacer **colcon build**

5. Para lanzar el código:
    - En la carpeta de vuestro repositorio hacéis:
        - source install/setup.bash
        - ros2 launch kalman_filters ekf_launch.py   

El resto ya es código que podéis bichear. En el paquete de kalman_filters como podréis intuir está todo lo que es puro código de filtro y todo lo que viene siendo la simulación de la bolita y todo dato de sensor está en moving_point.

* PD: Puede que se me olvide algo o haya escrito mal algún comando.



## Experimentos

1. El bueno: balizas con 6 metros, con ruido 10 cm, con 90%
2. Entorno radioelectrico medio: 60%
3. Entorno radioelectrico muy malo: 40% (este no se pone)
4. Balizas con poco rango: 4 metros
5. Tunel: GPS no manda
6. Mas ruido: Q=0.8, Rgps=0.3
