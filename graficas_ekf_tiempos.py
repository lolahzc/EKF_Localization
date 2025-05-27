import pandas as pd
import matplotlib.pyplot as plt
import os

# Ruta del archivo CSV de tiempos
csv_path = "ekf_tiempos.csv"

# Leer el archivo CSV
df = pd.read_csv(csv_path, header=None, names=["Tipo", "Timestamp", "Tiempo"])

# Convertir timestamp a relativo si quieres (opcional)
df["Timestamp"] = df["Timestamp"] - df["Timestamp"].iloc[0]

# Graficar cada tipo de callback
tipos = df["Tipo"].unique()
plt.figure(figsize=(12, 6))
for tipo in tipos:
    datos = df[df["Tipo"] == tipo]
    # Convertir a numpy array para evitar el error de pandas
    x = datos["Timestamp"].to_numpy()
    y = datos["Tiempo"].to_numpy()
    plt.plot(x, y, label=tipo, marker='.', linestyle='-', alpha=0.7)

plt.xlabel("Timestamp relativo (s)")
plt.ylabel("Tiempo de ejecución (s)")
plt.title("Coste computacional de callbacks EKF")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Guardar la figura
output_dir = "graficas"
os.makedirs(output_dir, exist_ok=True)
plt.savefig(os.path.join(output_dir, "ekf_tiempos.png"))
plt.show()
