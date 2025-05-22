import pandas as pd
import matplotlib.pyplot as plt

# Cargar el archivo CSV
df = pd.read_csv("hola.csv")

# Tiempo en segundos desde el inicio (relativo)
df['__time'] = df['__time'] - df['__time'].iloc[0]

# Columnas que se desean graficar
columns = {
    'EKF X': '/ekf/pos_estimated/pose/position/x',
    'Ground Truth X': '/ground_truth/odom/pose/pose/position/x',
    'GPS Latitude': '/sensors/gps/latitude'
}

# Crear figura
plt.figure(figsize=(12, 6))

# Recorrer y graficar cada una
for label, col in columns.items():
    if col in df.columns:
        series = df[['__time', col]].dropna()
        tiempos = series['__time'].to_numpy()
        valores = series[col].to_numpy()
        plt.plot(tiempos, valores, label=label)

plt.xlabel("Tiempo (segundos desde inicio)")
plt.ylabel("Valor")
plt.title("Comparación de datos: EKF, Ground Truth y GPS")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
