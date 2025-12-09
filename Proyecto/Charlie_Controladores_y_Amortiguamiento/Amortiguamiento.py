# amortiguamiento_desde_serial.py — Firma: Nadia
# Lee desde el puerto serie los bloques COAST/BRAKE del sketch AS5600-only,
# guarda CSV y estima b y Kt^2/R por ajuste exponencial de |omega|.
#
# Uso (Windows):
#   python amortiguamiento_desde_serial.py --port COM3 --baud 115200 --J 0.00108 --plot
#
# Uso (Linux):
#   python amortiguamiento_desde_serial.py --port /dev/ttyACM0 --baud 115200 --J 0.00108 --plot
#
# Requisitos: pyserial, numpy, matplotlib (opcional si desea graficar)

import argparse, sys, time, csv, math, datetime
from typing import List, Tuple, Optional

try:
    import serial  # pyserial
except ImportError:
    print("Instala pyserial: pip install pyserial")
    sys.exit(1)

try:
    import numpy as np
except Exception:
    np = None

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


def timestamp() -> str:
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_line_to_row(s: str) -> Optional[List[float]]:
    # Convierte 't,theta_deg,omega_deg_s' a [t, theta_deg, omega_deg_s]
    try:
        t, th, w = s.strip().split(",")
        return [float(t), float(th), float(w)]
    except Exception:
        return None


def listen_and_log(ser: serial.Serial, timeout_block: float = 45.0) -> Tuple[list, list]:
    """Escucha bloques COAST y BRAKE. Devuelve (coast_rows, brake_rows)."""
    coast, brake = [], []
    block = None
    got_header = False
    t_block = time.time()
    while True:
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            if block and (time.time() - t_block) > timeout_block:
                block, got_header = None, False
            continue

        if s == "DONE":
            break
        if s in ("COAST", "BRAKE"):
            block = s
            got_header = False
            t_block = time.time()
            continue
        if s.lower() == "t,theta_deg,omega_deg_s":
            got_header = True
            continue

        if got_header and block:
            row = parse_line_to_row(s)
            if row:
                (coast if block == "COAST" else brake).append(row)

    return coast, brake


def fit_tau_from_abs_omega(t: np.ndarray, w_abs: np.ndarray, omega_min: float):
    """Ajusta ln(|w|)=m t + b para puntos |w|>omega_min. Devuelve (tau, A)."""
    mask = np.isfinite(t) & np.isfinite(w_abs) & (w_abs > omega_min)
    t2 = t[mask]
    y2 = w_abs[mask]
    if t2.size < 10:
        raise RuntimeError("Pocos puntos válidos para el ajuste exponencial.")
    p = np.polyfit(t2, np.log(y2), 1)  # ln y = m t + b
    tau = -1.0 / p[0]
    A = float(np.exp(p[1]))
    return float(tau), A


def main():
    ap = argparse.ArgumentParser(description="Amortiguamiento desde SERIAL (AS5600-only).")
    ap.add_argument("--port", required=True, help="COMx o /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--J", type=float, required=True, help="Inercia [kg·m^2]")
    ap.add_argument("--omega_min_deg_s", type=float, default=5.0, help="Umbral para ajuste (deg/s)")
    ap.add_argument("--timeout", type=float, default=45.0, help="Timeout por bloque [s]")
    ap.add_argument("--no_brake", action="store_true", help="Ignorar bloque BRAKE si no se usa")
    ap.add_argument("--plot", action="store_true", help="Graficar |omega| y ajuste")
    ap.add_argument("--save_prefix", default=None, help="Prefijo para archivos (por defecto timestamp).")
    args = ap.parse_args()

    if np is None:
        print("Este script requiere numpy para el ajuste. Instala con: pip install numpy")
        sys.exit(1)

    ts = args.save_prefix or timestamp()
    coast_csv = f"coast_encoder_{ts}.csv"
    brake_csv = f"brake_encoder_{ts}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    try:
        ser.write(b"RUN\n")
        time.sleep(0.05)
        coast_rows, brake_rows = listen_and_log(ser, timeout_block=args.timeout)
    finally:
        ser.close()

    # Guardar CSV
    if coast_rows:
        with open(coast_csv, "w", newline="") as f:
            csv.writer(f).writerows(coast_rows)
        print(f"CSV: {coast_csv} ({len(coast_rows)} filas)")
    else:
        print("Advertencia: no se capturó COAST.")

    if brake_rows and not args.no_brake:
        with open(brake_csv, "w", newline="") as f:
            csv.writer(f).writerows(brake_rows)
        print(f"CSV: {brake_csv} ({len(brake_rows)} filas)")
    elif not args.no_brake:
        print("Advertencia: no se capturó BRAKE.")

    # Procesamiento y ajuste
    results = {}

    def process_block(rows, label):
        t = np.array([r[0] for r in rows], dtype=float)
        w_abs = np.abs(np.deg2rad([r[2] for r in rows]))
        tau, A = fit_tau_from_abs_omega(t, w_abs, np.deg2rad(args.omega_min_deg_s))
        if args.plot and plt is not None:
            fig = plt.figure()
            plt.plot(t, w_abs)
            mask = w_abs > np.deg2rad(args.omega_min_deg_s)
            t2 = t[mask]
            plt.plot(t2, A * np.exp(-t2 / tau), linestyle="--")
            plt.grid(True)
            plt.xlabel("t (s)")
            plt.ylabel("|omega| (rad/s)")
            plt.title(f"{label}: tau={tau:.3f}s")
            fig.savefig(f"{label.lower()}_{ts}.png", dpi=150, bbox_inches="tight")
        return tau

    tau_open = None
    if coast_rows:
        tau_open = process_block(coast_rows, "COAST")
        b = args.J / tau_open
        results["tau_open_s"] = tau_open
        results["b_Nm_s_per_rad"] = b
        print(f"COAST: tau_open={tau_open:.4f} s  =>  b={b:.3e} N·m·s/rad")
    else:
        print("No se puede estimar b sin COAST.")

    if brake_rows and (tau_open is not None) and not args.no_brake:
        tau_short = process_block(brake_rows, "BRAKE")
        Kt2_over_R = args.J / tau_short - args.J / tau_open
        results["tau_short_s"] = tau_short
        results["Kt2_over_R"] = Kt2_over_R
        print(f"BRAKE: tau_short={tau_short:.4f} s  =>  Kt^2/R={Kt2_over_R:.3e}")
    elif not args.no_brake and tau_open is not None:
        print("No se puede estimar Kt^2/R sin BRAKE.")

    # Guardar resumen
    import json
    with open(f"resumen_{ts}.json", "w", encoding="utf-8") as jf:
        json.dump(results, jf, indent=2)
    print(f"Resumen: resumen_{ts}.json")

    if args.plot and plt is not None:
        try:
            plt.show()
        except Exception:
            pass


if __name__ == "__main__":
    main()
