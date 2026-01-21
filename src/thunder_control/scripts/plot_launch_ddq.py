import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

# --- CONFIGURAZIONE ---
# Percorso del file CSV
csv_path = '/home/thunder_dev/thunder_dynamics/src/thunder_control/build/simulation_data.csv'

# Se vuoi testarlo nella cartella corrente, scommenta la riga sotto:
# csv_path = 'simulation_data.csv'

# Passo di campionamento (Time Step)
DT = 0.001 

# --- CONTROLLI PRELIMINARI ---
if not os.path.exists(csv_path):
    print(f"Errore: Non trovo il file in: {csv_path}")
    print("Controlla il percorso o esegui la simulazione.")
    exit()

try:
    df = pd.read_csv(csv_path)
    print(f"CSV caricato con successo. Righe: {len(df)}")
    print("Colonne:", df.columns.tolist())
except Exception as e:
    print(f"Errore durante la lettura del CSV: {e}")
    exit()

# --- CREAZIONE ASSE TEMPORALE ---
if 't' in df.columns:
    t = df['t']
else:
    # Genera il tempo basandosi sull'indice e il DT
    t = np.arange(len(df)) * DT

# --- DEFINIZIONE GRUPPI DI DATI ---
# Definisco i nomi delle colonne per le 3 categorie
joints = range(1, 8) # Giunti da 1 a 7

cols_q   = [f'q{i}' for i in joints]     # ['q1', 'q2', ..., 'q7']
cols_dq  = [f'dq{i}' for i in joints]    # ['dq1', 'dq2', ..., 'dq7']
cols_ddq = [f'ddq{i}' for i in joints]   # ['ddq1', 'ddq2', ..., 'ddq7']

# --- FUNZIONE DI PLOT ---
output_folder = "/home/thunder_dev/thunder_dynamics/src/thunder_control/plots"

# Crea la cartella se non esiste (evita errori FileNotFoundError)
os.makedirs(output_folder, exist_ok=True) 

# --- FUNZIONE DI PLOT ---
def plot_category(time_axis, data_frame, columns, title, ylabel, filename, folder_path):
    """
    Funzione helper per generare e salvare i plot in modo uniforme
    """
    plt.figure(figsize=(10, 6))
    
    for col in columns:
        if col in data_frame.columns:
            plt.plot(time_axis, data_frame[col], linewidth=1.5, label=col)
        else:
            print(f"Attenzione: Colonna {col} non trovata nel CSV")

    plt.title(title, fontsize=14)
    plt.xlabel("Tempo [s]", fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0)
    plt.tight_layout()

    # --- MODIFICA QUI: Creazione del percorso completo ---
    full_path = os.path.join(folder_path, filename)
    
    plt.savefig(full_path, dpi=150)
    plt.close()
    print(f"Salvato: {full_path}")

# --- GENERAZIONE DEI 3 PLOT ---

# 1. Posizioni (q)
plot_category(t, df, cols_q, 
              title="Posizioni Giunti (q1 - q7)", 
              ylabel="Posizione [rad]", 
              filename="franka_1_positions.png",
              folder_path=output_folder) # <--- Passiamo la cartella

# 2. Velocità (dq)
plot_category(t, df, cols_dq, 
              title="Velocità Giunti (dq1 - dq7)", 
              ylabel="Velocità [rad/s]", 
              filename="franka_2_velocities.png",
              folder_path=output_folder) # <--- Passiamo la cartella

# 3. Accelerazioni (ddq)
plot_category(t, df, cols_ddq, 
              title="Accelerazioni Giunti (ddq1 - ddq7)", 
              ylabel="Accelerazione [rad/s^2]", 
              filename="franka_3_accelerations.png",
              folder_path=output_folder) # <--- Passiamo la cartella

print(f"Tutti i grafici sono stati salvati in: {output_folder}")