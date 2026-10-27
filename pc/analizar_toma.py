"""
analizar_toma.py -- corre el pipeline sparse-6 completo sobre una toma.

Salidas por toma:
  * sesgo de giroscopio por sensor (de las ventanas de reposo)
  * eje de flexion funcional por sensor + varianza explicada
  * eventos de marcha bilaterales, parametros espacio-temporales, simetria
  * largo de zancada y clearance por ZUPT
  * inclinacion de tronco y pelvis, braceo
  * angulo de la cadena sagital pelvis->pie por ciclo de marcha
  * <toma>_orientations.sto para OpenSense IMU IK

Uso:  python analizar_toma.py <csv> [<csv> ...] [--fig CARPETA]
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sparse6 as s6
from mapa_sensores import mapa_de, FRAME_OPENSENSE, identificar


def analizar(csv, carpeta_fig=None, verbose=True):
    R, t = s6.cargar_toma(csv)
    if len(R) < 2:
        print("  %s: sin datos suficientes" % os.path.basename(csv))
        return None
    nombre = os.path.basename(csv).split("_log_")[0]
    mapa = mapa_de(csv)
    seg = {m: mapa.get(m, "?" + m[:5]) for m in R}
    auto, diag = identificar(R)

    print("=" * 92)
    print("%s   %d sensores   %.1f s @ 100 Hz" % (nombre, len(R), t[-1]))
    print("  identificacion automatica vs mapa del operador:")
    for m in sorted(R):
        print("    %s  operador=%-9s  auto=%-9s  frac_ZUPT=%.3f  mov_parado=%6.1f d/s"
              % (m, seg[m], auto.get(m, "-"), diag["frac_zupt"][m],
                 diag["mov_con_pies_plantados"].get(m, np.nan)))

    # ---- por sensor: reposo, sesgo, marco funcional, orientacion
    D = {}
    print("  sensor      |sesgo| ZUPT  |sesgo| VQF   dif    AP/ML  %reposo  signo")
    for m in sorted(R, key=lambda k: seg[k]):
        _, acc, gyro = R[m]
        rep = s6.detectar_reposo(acc, gyro)
        sesgo = s6.sesgo_gyro(gyro, rep)
        Rseg, rel = s6.marco_segmento(acc, rep, ~rep)
        q, sesgo_vqf, rep_vqf = s6.orientacion(acc, gyro, sesgo=sesgo)
        # el sentido del eje AP del pie se resuelve con su propio avance, y eso
        # necesita la orientacion: por eso va DESPUES de `orientacion`.
        signo = 1.0
        if seg[m].startswith("pie"):
            signo = s6.signo_pie_por_avance(acc, q, rep, Rseg)
            if signo < 0:
                Rseg, rel = s6.marco_segmento(acc, rep, ~rep, signo=signo)
        # el sesgo residual que VQF encuentra DESPUES de restar el de ZUPT
        # mide cuanto se equivoco la estimacion por ZUPT
        res = np.linalg.norm(sesgo_vqf[len(sesgo_vqf) // 2:].mean(axis=0))
        D[seg[m]] = dict(mac=m, acc=acc, gyro=gyro, rep=rep, sesgo=sesgo,
                         Rseg=Rseg, rel_ap=rel, q=q, signo=signo)
        print("  %-11s %8.3f     %8.3f    %6.3f   %5.2f   %5.1f   %+.0f"
              % (seg[m], np.linalg.norm(sesgo),
                 np.linalg.norm(sesgo) + res, res, rel,
                 100 * rep.mean(), signo))

    # ---- eventos de marcha bilaterales
    res = dict(nombre=nombre, t=t, D=D)
    for lado, nm in (("der", "pie_der"), ("izq", "pie_izq")):
        if nm not in D:
            continue
        d = D[nm]
        ic, fo, rep = s6.eventos_marcha(t, d["acc"], d["gyro"])
        p = s6.trayectoria_pie(d["acc"], d["q"], rep)
        pm = s6.parametros_marcha(t, ic, fo)
        largo, clear = s6.parametros_espaciales(p, ic, fo)
        d.update(ic=ic, fo=fo, p=p, pm=pm, largo=largo, clear=clear)
        if pm:
            ok = np.isfinite(largo) & (largo < 2.5) & (largo > 0.2)
            print("  pie %s: %d apoyos, %d ciclos validos | zancada %.3f+-%.3f s"
                  " | apoyo %.1f+-%.1f %% | cadencia %.1f pasos/min"
                  % (lado, len(ic), pm["n_ciclos"],
                     np.median(pm["t_zancada"]), pm["t_zancada"].std(),
                     np.median(pm["pct_apoyo"]), pm["pct_apoyo"].std(),
                     pm["cadencia_pasos_min"]))
            print("          largo de zancada %.3f+-%.3f m (n=%d) | clearance %.3f m"
                  % (np.median(largo[ok]) if ok.any() else np.nan,
                     largo[ok].std() if ok.any() else np.nan, ok.sum(),
                     np.nanmedian(clear)))

    # ---- doble apoyo / simetria
    if "pie_der" in D and "pie_izq" in D and len(D["pie_der"]["ic"]) > 2:
        rd, ri = D["pie_der"]["rep"], D["pie_izq"]["rep"]
        doble = (rd & ri).mean() / max((rd | ri).mean(), 1e-9)
        print("  doble apoyo: %.1f %% del tiempo con al menos un pie en el suelo"
              % (100 * doble))
        td = D["pie_der"]["pm"].get("t_zancada")
        ti = D["pie_izq"]["pm"].get("t_zancada")
        if td is not None and ti is not None and len(td) and len(ti):
            a, b = np.median(td), np.median(ti)
            print("  asimetria de zancada D/I: %.1f %%" % (100 * abs(a - b) / ((a + b) / 2)))

    # ---- tronco, pelvis, braceo
    for nm, etiqueta in (("espalda", "pelvis"), ("cabeza", "tronco/cabeza")):
        if nm not in D:
            continue
        d = D[nm]
        sag, fro = s6.angulos_mundo(d["q"], d["Rseg"])
        d.update(sag=sag, fro=fro)
        print("  %-13s inclinacion AP %6.1f+-%4.1f deg | ML %6.1f+-%4.1f deg | ROM AP %.1f deg"
              % (etiqueta, sag.mean(), sag.std(), fro.mean(), fro.std(),
                 np.percentile(sag, 95) - np.percentile(sag, 5)))

    for nm, etiqueta in (("mano_der", "braceo der"), ("mano_izq", "braceo izq")):
        if nm not in D or "espalda" not in D:
            continue
        sag, fro, tra = s6.angulos_relativos(D["espalda"]["q"], D[nm]["q"],
                                             D["espalda"]["Rseg"], D[nm]["Rseg"])
        D[nm].update(sag=sag, fro=fro)
        print("  %-13s ROM sagital respecto de la pelvis: %.1f deg"
              % (etiqueta, np.percentile(sag, 95) - np.percentile(sag, 5)))

    # ---- cadena sagital pelvis -> pie  (la ecuacion clave del objetivo 1)
    for lado, nm in (("der", "pie_der"), ("izq", "pie_izq")):
        if nm not in D or "espalda" not in D:
            continue
        sag, fro, tra = s6.angulos_relativos(D["espalda"]["q"], D[nm]["q"],
                                             D["espalda"]["Rseg"], D[nm]["Rseg"])
        ic = D[nm].get("ic", np.array([], int))
        ciclos = s6.por_ciclo(sag, t, ic)
        D[nm].update(cadena=sag, cadena_ciclos=ciclos)
        if len(ciclos) >= 3:
            mu, sd = ciclos.mean(0), ciclos.std(0)
            print("  cadena sagital pelvis->pie %s: %d ciclos | ROM %.1f deg |"
                  " dispersion entre ciclos %.1f deg (media de sd)"
                  % (lado, len(ciclos), mu.max() - mu.min(), sd.mean()))

    # ---- export OpenSense
    quats, nombres = {}, []
    for nm, d in D.items():
        fr = FRAME_OPENSENSE.get(nm)
        if fr:
            quats[fr] = d["q"]
            nombres.append(fr)
    if nombres:
        destino = os.path.join(os.path.dirname(csv) if carpeta_fig is None
                               else carpeta_fig, nombre + "_orientations.sto")
        s6.exportar_sto(destino, t, quats, sorted(nombres))
        print("  -> %s  (%d frames, %d muestras)"
              % (os.path.basename(destino), len(nombres), len(t)))
    res["D"] = D
    return res


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    fig = None
    if "--fig" in sys.argv:
        fig = sys.argv[sys.argv.index("--fig") + 1]
        os.makedirs(fig, exist_ok=True)
    for csv in args:
        if os.path.getsize(csv) < 100000 or "run_log" in csv:
            continue
        analizar(csv, carpeta_fig=fig)
