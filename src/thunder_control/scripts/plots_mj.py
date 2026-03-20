import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import os
import matplotlib

matplotlib.use("TkAgg")

# --- CONFIGURAZIONE PERCORSI ---
BASE_PATH = "/home/thunder_dev/thunder_dynamics/src/thunder_control/build/"
FILE_TRAJS = os.path.join(BASE_PATH, "trajectories_history.csv")
FILE_DATA = os.path.join(BASE_PATH, "simulation_data.csv")

# --- PARAMETRI DI VELOCIZZAZIONE ---
SPEED_UP_FACTOR = 5
ANIMATION_INTERVAL = 1


def run_plot():
    if not os.path.exists(FILE_TRAJS) or not os.path.exists(FILE_DATA):
        print(f"Errore: File non trovati in {BASE_PATH}")
        return

    N_HORIZON = 20
    t_hor_lim = 0.2

    # 1. Caricamento Dati
    df_ref = pd.read_csv(FILE_TRAJS, header=None)
    df_real = pd.read_csv(FILE_DATA)

    # 2. OTTIMIZZAZIONE: Conversione in NumPy Arrays
    data_ref_np = df_ref.values
    q_real_full = df_real.iloc[:, 0].values
    
    # --- CALCOLO DINAMICO DEL TEMPO FINALE ---
    t_end_sim = data_ref_np[-1, 0] 
    x_limit = t_end_sim * 1.1

    # Pre-calcolo asse temporale reale
    dt_sim = 0.001
    t_real_full = np.arange(len(q_real_full)) * dt_sim

    # Setup Plot
    fig, ax = plt.subplots(figsize=(24, 14))

    # --- ELEMENTI DINAMICI ---
    (line_real,) = ax.plot([], [], "b-", linewidth=2, label="Traiettoria Reale (Sim)")
    (line_mj,) = ax.plot([], [], "r--", linewidth=1.5, label="Riferimento MinJerk")
    (line_mpc,) = ax.plot([], [], "y-o", linewidth=1.5, markersize=4, alpha=0.7, label="Predizione MPC (Corrente)")
    dot_current = ax.plot([], [], "go", markersize=8, zorder=5)[0]

    # --- ELEMENTO STATICO ---
    TARGET_IDX = 1

    if len(data_ref_np) > TARGET_IDX:
        # Estrazione dati riga specifica
        row_fixed = data_ref_np[TARGET_IDX]
        t_sim_fixed = row_fixed[0]
        q_mpc_fixed = row_fixed[N_HORIZON + 2 :]

        # Calcolo asse temporale per questa specifica predizione
        time_to_go_fixed = max(t_end_sim - t_sim_fixed, t_hor_lim)
        dt_node_fixed = time_to_go_fixed / N_HORIZON
        t_horizon_fixed = t_sim_fixed + np.arange(N_HORIZON + 1) * dt_node_fixed

        # Plot statico (Arancione)
        ax.plot(
            t_horizon_fixed,
            q_mpc_fixed,
            color="orange",
            linewidth=2.5,
            linestyle="-",
            label="MPC (Iterazione fissa)",
        )

    ax.set_title("Analisi MPC: Reale vs Riferimento vs Predizione", fontsize=14)
    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel("q1 [rad]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)

    # Limiti assi
    ax.set_xlim(0, x_limit)
    y_min = min(q_real_full.min(), data_ref_np[:, 1:].min())
    y_max = max(q_real_full.max(), data_ref_np[:, 1:].max())
    ax.set_ylim(y_min - 0.1, y_max + 0.1)

    frames_indices = range(0, len(data_ref_np), SPEED_UP_FACTOR)
    last_t_sim = [0.0]  
    def update(frame_idx):
        row = data_ref_np[frame_idx]
        t_sim = row[0]
        last_t_sim[0] = t_sim

        q_mj = row[1 : N_HORIZON + 2]
        q_mpc = row[N_HORIZON + 2 :]

        time_to_go = max(t_end_sim - t_sim, t_hor_lim)
        dt_node = time_to_go / N_HORIZON
        t_horizon = t_sim + np.arange(N_HORIZON + 1) * dt_node

        # Update linee dinamiche
        line_mj.set_data(t_horizon, q_mj)
        line_mpc.set_data(t_horizon, q_mpc)

        idx_real = int(t_sim / dt_sim)
        idx_real = min(idx_real, len(t_real_full))

        if idx_real > 0:
            line_real.set_data(t_real_full[:idx_real], q_real_full[:idx_real])
            dot_current.set_data([t_sim], [q_real_full[idx_real - 1]])
        else:
            dot_current.set_data([t_sim], [row[1]])

        return line_mj, line_real, line_mpc, dot_current

    ani = FuncAnimation(
        fig,
        update,
        frames=frames_indices,
        blit=False,
        interval=ANIMATION_INTERVAL,
        repeat=False,
    )

    # --- LOGICA PAUSA / PLAY ---
    paused = False

    def toggle_pause(event):
        nonlocal paused

        # Gestione uscita con ESC
        if event.name == "key_press_event" and event.key == "escape":
            plt.close(fig)
            return

        # 2. 'r' per Restart
        if event.name == "key_press_event" and event.key == "r":
            # Ferma l'animazione corrente
            ani.event_source.stop()

            # Resetta l'iteratore dei frame all'inizio
            ani.frame_seq = ani.new_frame_seq()

            # Pulisce le linee dal grafico (opzionale ma pulito)
            line_real.set_data([], [])
            line_mj.set_data([], [])
            line_mpc.set_data([], [])
            dot_current.set_data([], [])

            # Resetta stato pausa e titolo
            paused = False
            ax.set_title("Analisi MPC (Restarted)")

            # Riavvia
            ani.event_source.start()
            plt.draw()
            return

        # Controllo se l'evento è SPAZIO o CLICK mouse
        if (event.name == "key_press_event" and event.key == " ") or (
            event.name == "button_press_event"
        ):

            if paused:
                ani.event_source.start()
                ax.set_title("Analisi MPC (Running...)")
            else:
                ani.event_source.stop()
                ax.set_title("Analisi MPC (PAUSED at t={:.2f}s)".format(last_t_sim[0]))

            paused = not paused
            plt.draw()  

    
    fig.canvas.mpl_connect("key_press_event", toggle_pause)
    # fig.canvas.mpl_connect("button_press_event", toggle_pause)

    plt.show()


if __name__ == "__main__":
    run_plot()
