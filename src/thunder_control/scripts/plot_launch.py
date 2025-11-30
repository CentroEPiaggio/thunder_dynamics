import matplotlib.pyplot as plt
import csv
import os

# Cerca il file csv
csv_path = '/home/thunder_dev/thunder_dynamics/src/thunder_control/build/simulation_data.csv'

if not os.path.exists(csv_path):
    print(f"Errore: Non trovo {csv_path}. Esegui prima ./franka_mpc_run")
    exit()

# Leggi il file manualmente
t = []
ref_q_elbow = []
ref_v_elbow = []
q_elbow = []
dq_elbow = []
tau_elbow = []

with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row['t']))
        ref_q_elbow.append(float(row['ref_q_elbow']))
        ref_v_elbow.append(float(row['ref_v_elbow']))
        q_elbow.append(float(row['q_elbow']))
        dq_elbow.append(float(row['dq_elbow']))
        tau_elbow.append(float(row['tau_elbow']))

# Crea i plot
plt.figure(figsize=(12, 8))

# 1. Posizione Gomito
plt.subplot(3, 1, 1)
plt.plot(t, ref_q_elbow, 'k--', linewidth=2, label='Reference (Planner)')
plt.plot(t, q_elbow, 'b', linewidth=2, label='Real (MPC)')
plt.title("Lancio Franka - Giunto Gomito")
plt.ylabel("Posizione [rad]")
plt.legend()
plt.grid(True)

# 2. Velocità Gomito
plt.subplot(3, 1, 2)
plt.plot(t, ref_v_elbow, 'k--', linewidth=2, label='Ref Vel')
plt.plot(t, dq_elbow, 'r', linewidth=2, label='Real Vel')
plt.axhline(y=2.0, color='g', linestyle=':', label='Target Lancio (2.0 rad/s)')
plt.ylabel("Velocità [rad/s]")
plt.legend()
plt.grid(True)

# 3. Coppia Calcolata
plt.subplot(3, 1, 3)
plt.plot(t, tau_elbow, 'g', linewidth=1.5)
plt.ylabel("Coppia [Nm]")
plt.xlabel("Tempo [s]")
plt.grid(True)

plt.tight_layout()

# SALVA invece di mostrare
plt.savefig('franka_launch_analysis.png', dpi=150, bbox_inches='tight')
print("Plot salvato come 'franka_launch_analysis.png'")

# Opzionale: salva anche in PDF
plt.savefig('franka_launch_analysis.pdf', bbox_inches='tight')
print("Plot salvato anche come 'franka_launch_analysis.pdf'")