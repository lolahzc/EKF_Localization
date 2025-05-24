import pandas as pd
import matplotlib.pyplot as plt

# Cargar el archivo CSV
df = pd.read_csv("exp_52.csv")

# Tiempo en segundos desde el inicio (relativo)
df['__time'] = df['__time'] - df['__time'].iloc[0]

# Columnas para cada eje
columns_x = {
    'EKF X': '/ekf/pos_estimated/pose/position/x',
    'Ground Truth X': '/ground_truth/odom/pose/pose/position/x',
}
columns_y = {
    'EKF Y': '/ekf/pos_estimated/pose/position/y',
    'Ground Truth Y': '/ground_truth/odom/pose/pose/position/y',
}
columns_z = {
    'EKF Z': '/ekf/pos_estimated/pose/position/z',
    'Ground Truth Z': '/ground_truth/odom/pose/pose/position/z',
}

# Función para graficar cada eje
for columns, eje in zip([columns_x, columns_y, columns_z], ['X', 'Y', 'Z']):
    plt.figure(figsize=(12, 6))
    for label, col in columns.items():
        if col in df.columns:
            series = df[['__time', col]].dropna()
            tiempos = series['__time'].to_numpy()
            valores = series[col].to_numpy()
            plt.plot(tiempos, valores, label=label)
    plt.xlabel("Tiempo (segundos desde inicio)")
    plt.ylabel(f"Valor {eje}")
    plt.title(f"Comparación de datos: EKF vs Ground Truth ({eje})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
plt.show()

# --- Gráfica de balizas (ruido vs real) ---
beacon_columns = {
    'Beacon 0 (ruido)': '/beacon_distances/data[0]',
    'Beacon 0 (real)': '/beacon_distances_real/data[0]',
    'Beacon 1 (ruido)': '/beacon_distances/data[1]',
    'Beacon 1 (real)': '/beacon_distances_real/data[1]',
    'Beacon 2 (ruido)': '/beacon_distances/data[2]',
    'Beacon 2 (real)': '/beacon_distances_real/data[2]',
    'Beacon 3 (ruido)': '/beacon_distances/data[3]',
    'Beacon 3 (real)': '/beacon_distances_real/data[3]',
    'Beacon 4 (ruido)': '/beacon_distances/data[4]',
    'Beacon 4 (real)': '/beacon_distances_real/data[4]',
    'Beacon 5 (ruido)': '/beacon_distances/data[5]',
    'Beacon 5 (real)': '/beacon_distances_real/data[5]',
}
plt.figure(figsize=(12, 6))
for label, col in beacon_columns.items():
    if col in df.columns:
        series = df[['__time', col]].dropna()
        tiempos = series['__time'].to_numpy()
        valores = series[col].to_numpy()
        plt.plot(tiempos, valores, label=label)
plt.xlabel("Tiempo (segundos desde inicio)")
plt.ylabel("Distancia (m)")
plt.title("Comparación de datos: Balizas (ruido vs real)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
