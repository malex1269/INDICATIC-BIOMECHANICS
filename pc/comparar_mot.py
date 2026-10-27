"""
comparar_mot.py -- OpenCap vs OpenCap+IMU vs OpenCap+IMU+pelvis, por ciclo.

    python3 comparar_mot.py testnorma2l9112026
    python3 comparar_mot.py --todas
    python3 comparar_mot.py test1normal9112026 --mot ../resultados/sesion_2026-09-11/2_para_opensim --salida ../resultados/sesion_2026-09-11/3_analisis
    python3 comparar_mot.py --todas --salida CARPETA      (default ../resultados)

Toma los tres `.mot` del mismo trial que hay en la carpeta `--mot` (default `../resultados/`)
(el de OpenCap puro, el fusionado y el fusionado con pelvis), los corta en
ciclos de marcha y saca lo que hace falta para compararlos:

  <toma>_comparacion_mot.png          curvas 0-100 % del ciclo, media +- DE,
                                      una fila por coordenada, der / izq
  <toma>_comparacion_mot_tabla.csv    por coordenada, lado y version: n ciclos,
                                      ROM, maximo y minimo por ciclo (media y
                                      DE), y contra OpenCap: r y RMSE
  <toma>_comparacion_mot_curvas.csv   las curvas medias y su DE, formato
                                      largo, para graficar aparte (Excel, etc)

LOS CICLOS SALEN DEL IMU, no del .mot. El contacto inicial lo detecta
`parametros_marcha.contacto_inicial` en el pie (el mismo que usa la app), y
se lleva al tiempo de OpenCap con el arranque que da `sincronizar`. Asi las
tres versiones se cortan en los mismos instantes y las curvas son
comparables punto a punto. Solo se usan las zancadas tipicas (ver
`parametros_marcha.ciclos_del_lado`); los pasitos de las vueltas quedan
fuera.

QUE ES CADA VERSION
  OpenCap       el .mot de OpenCap tal cual: la referencia.
  +IMU          el mismo, con la cadena de la pierna (pie - sacro) y la del
                brazo (mano) incrustadas. `fusionar_mot.py` default.
  +IMU+pelvis   ademas la pelvis del sacro, compensando lumbar y caderas.

Ojo con la lectura: las versiones "+IMU" PARTEN de OpenCap, asi que en todo
lo que el IMU no toca (tobillo, subtalar...) la diferencia es cero por
construccion, y en lo que si toca la diferencia ES el aporte del IMU, no un
error de nadie. La pelvis de OpenCap no es referencia (ver
docs/3_analisis_y_validacion.md, seccion 3.4).
"""
import argparse
import csv as _csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import opencap_io as oc
import sincronizar as S
import reconstruir_toma as R
import validar_reconstruccion as VR
import fusionar_mot as F
import parametros_marcha as PM

DIR_MOT = R.DIR_SALIDAS
DIR_SALIDA = R.DIR_SALIDAS

VERSIONES = [("OpenCap", "{trial}.mot", "k"),
             ("+IMU", "{tag}_opencap_mas_imu.mot", "tab:blue"),
             ("+IMU+pelvis", "{tag}_opencap_mas_imu_con_pelvis.mot", "tab:orange")]

# (etiqueta, como se calcula por lado: lista de (coordenada, signo) con {s}
#  = r/l). Las cadenas son las cantidades que el IMU mide de verdad; las
#  coordenadas sueltas son como se reparten.
COORDS = [
    ("pelvis_tilt", [("pelvis_tilt", 1.0)]),
    ("hip_flexion", [("hip_flexion_{s}", 1.0)]),
    ("knee_angle", [("knee_angle_{s}", 1.0)]),
    ("ankle_angle", [("ankle_angle_{s}", 1.0)]),
    ("cadena pierna (hip - knee + ankle)", [("hip_flexion_{s}", 1.0),
                                            ("knee_angle_{s}", -1.0),
                                            ("ankle_angle_{s}", 1.0)]),
    ("arm_flex", [("arm_flex_{s}", 1.0)]),
    ("elbow_flex", [("elbow_flex_{s}", 1.0)]),
    ("cadena brazo (arm_flex + elbow)", [("arm_flex_{s}", 1.0),
                                         ("elbow_flex_{s}", 1.0)]),
    ("lumbar_extension", [("lumbar_extension", 1.0)]),
]
SIN_LADO = {"pelvis_tilt", "lumbar_extension"}   # se cortan con el pie derecho


def _serie(df, partes, lado):
    s = "r" if lado == "derecha" else "l"
    x = np.zeros(len(df))
    for c, sg in partes:
        x = x + sg * df[c.format(s=s)].to_numpy(float)
    return x


def ciclos_en_opencap(csv, trc, t_oc):
    """Ciclos tipicos del IMU llevados a indices de la grilla de OpenCap.

    Devuelve (dict lado -> lista de (i0, i1), dict lado -> despegue_pct,
    arranque_s). El instante t de OpenCap es (t + arranque) del IMU.
    """
    sync = S.sincronizar(csv, trc, verbose=False)
    off = sync["arranque_s"]
    res = PM.analizar(csv, verbose=False)
    out, desp = {}, {}
    for lado, cc in (res.ciclos or {}).items():
        if not cc:
            continue
        idx = []
        for i0, i1 in cc["cic"]:
            a = np.searchsorted(t_oc, res.t[i0] - off)
            b = np.searchsorted(t_oc, res.t[i1] - off)
            if 0 <= a < b <= len(t_oc) and b - a >= 10:
                idx.append((int(a), int(b)))
        out[lado] = idx
        desp[lado] = cc.get("despegue_pct")
    return out, desp, off, sync


def ciclos_plausibles_en_opencap(df, cic, lado, r_min=0.5, rom_max=2.0):
    """De los ciclos `cic` (indices en la grilla de OpenCap), cuales son
    tipicos TAMBIEN en OpenCap.

    Los ciclos salen del IMU, que no sabe cuando OpenCap devuelve cualquier
    cosa: en TEST2 (2026-09-11) el sujeto entra en camara caminando durante
    los primeros 4 s y OpenCap le pone el tobillo pegado a +-50 grados, la
    pelvis 40 grados de lado y 300 grados de rango en la cadena, con la
    confianza de los keypoints 2D alta (no es que no lo vea: es que la
    solucion es mala). Ese tramo es marcha perfectamente normal para el IMU,
    asi que entraba a la comparacion y arruinaba la media. El criterio es el
    mismo que se le aplica al pie del IMU en `ciclos_del_lado`, sobre las
    curvas de OpenCap: la cadena de la pierna tiene que parecerse a la
    mediana de los ciclos (r >= `r_min`) y el ROM del tobillo no puede pasar
    de `rom_max` veces la mediana. Devuelve (ciclos que quedan, n descartados).
    """
    if len(cic) < 3:
        return cic, 0
    cadena = _serie(df, [("hip_flexion_{s}", 1.0), ("knee_angle_{s}", -1.0),
                         ("ankle_angle_{s}", 1.0)], lado)
    tob = _serie(df, [("ankle_angle_{s}", 1.0)], lado)
    _g, C, _m, _sd = PM.ciclo_promedio(cadena, cic)
    _g, A, _m, _sd = PM.ciclo_promedio(tob, cic)
    medn = np.median(C, axis=0)
    rom = np.ptp(A, axis=1)
    ok = np.array([np.corrcoef(c, medn)[0, 1] >= r_min for c in C])         & (rom <= rom_max * np.median(rom))
    if ok.sum() < 2:
        return cic, 0
    return [c for c, k in zip(cic, ok) if k], int((~ok).sum())


def comparar(tag, verbose=True, dir_mot=DIR_MOT):
    trial = R.PARES.get(tag)
    if not trial:
        raise SystemExit("%s no tiene trial de OpenCap pareado" % tag)
    csv = F.csv_crudo(tag)
    trc, _ = VR.rutas_oc(tag, trial)

    mots = {}
    for nombre, patron, _c in VERSIONES:
        ruta = os.path.join(dir_mot, patron.format(tag=tag, trial=trial))
        if not os.path.isfile(ruta):
            print("  falta %s (corre fusionar_mot.py); se omite" % os.path.basename(ruta))
            continue
        mots[nombre] = oc.leer_mot(ruta)
    t_oc = mots["OpenCap"][0]
    for nombre, (t, _d) in mots.items():
        if len(t) != len(t_oc):
            raise SystemExit("%s no esta en la grilla de OpenCap" % nombre)

    cic, desp, off, sync = ciclos_en_opencap(csv, trc, t_oc)
    r_sync = sync["r"]
    fuera = {}
    for lado in list(cic):
        cic[lado], fuera[lado] = ciclos_plausibles_en_opencap(
            mots["OpenCap"][1], cic[lado], lado)
    if verbose:
        if sync["metodo"] == "golpes":
            print("  sincronizacion por %d golpes de talon (precision %.0f ms): "
                  "OpenCap arranca %+.3f s despues"
                  % (sync["n_golpes"], sync["precision_ms"], off))
        else:
            print("  sincronizacion por correlacion r=%.2f, OpenCap arranca "
                  "%+.2f s despues" % (r_sync, off))
        for lado, c in cic.items():
            print("  %-9s %d zancadas tipicas · despegue %s %%%s"
                  % (lado, len(c), "%.0f" % desp[lado] if desp.get(lado) else "?",
                     " (%d descartadas: OpenCap atipico en ese ciclo)" % fuera[lado]
                     if fuera[lado] else ""))

    curvas, filas = {}, []           # curvas[(coord, lado, version)] = (m, sd, C)
    for etiq, partes in COORDS:
        lados = ["derecha"] if etiq in SIN_LADO else ["derecha", "izquierda"]
        for lado in lados:
            if lado not in cic or not cic[lado]:
                continue
            ref = None
            for nombre, _p, _c in VERSIONES:
                if nombre not in mots:
                    continue
                x = _serie(mots[nombre][1], partes, lado)
                _g, C, m, sd = PM.ciclo_promedio(x, cic[lado])
                curvas[(etiq, lado, nombre)] = (m, sd, C)
                rom = np.ptp(C, axis=1)
                fila = {"toma": tag, "coordenada": etiq,
                        "lado": "-" if etiq in SIN_LADO else lado,
                        "version": nombre, "n_ciclos": len(C),
                        "ROM_media": rom.mean(), "ROM_DE": rom.std(),
                        "max_media": C.max(axis=1).mean(), "max_DE": C.max(axis=1).std(),
                        "min_media": C.min(axis=1).mean(), "min_DE": C.min(axis=1).std()}
                if nombre == "OpenCap":
                    ref = C
                    fila["r_vs_OpenCap"], fila["RMSE_vs_OpenCap"] = 1.0, 0.0
                else:
                    d = C - ref
                    a, b = C.ravel(), ref.ravel()
                    fila["r_vs_OpenCap"] = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1]) \
                        if a.std() > 1e-9 and b.std() > 1e-9 else float("nan")
                    fila["RMSE_vs_OpenCap"] = float(np.sqrt(np.mean(d ** 2)))
                filas.append(fila)
    return dict(tag=tag, trial=trial, cic=cic, desp=desp, curvas=curvas,
                filas=filas, versiones=[v for v in VERSIONES if v[0] in mots],
                fuera=fuera, sync=dict(arranque_s=off, r=r_sync))


def imprimir(res):
    print("\n  %-34s %-9s %-12s %3s %9s %9s %9s %7s %7s"
          % ("coordenada", "lado", "version", "n", "ROM", "max", "min",
             "r", "RMSE"))
    ultima = None
    for f in res["filas"]:
        if (f["coordenada"], f["lado"]) != ultima:
            ultima = (f["coordenada"], f["lado"])
            print()
        print("  %-34s %-9s %-12s %3d %5.1f±%-3.0f %5.1f±%-3.0f %5.1f±%-3.0f %7.2f %7.1f"
              % (f["coordenada"], f["lado"], f["version"], f["n_ciclos"],
                 f["ROM_media"], f["ROM_DE"], f["max_media"], f["max_DE"],
                 f["min_media"], f["min_DE"], f["r_vs_OpenCap"], f["RMSE_vs_OpenCap"]))


def guardar(res, carpeta):
    tag = res["tag"]
    if not os.path.isdir(carpeta):
        os.makedirs(carpeta)
    cols = ["toma", "coordenada", "lado", "version", "n_ciclos", "ROM_media",
            "ROM_DE", "max_media", "max_DE", "min_media", "min_DE",
            "r_vs_OpenCap", "RMSE_vs_OpenCap"]
    p1 = os.path.join(carpeta, tag + "_comparacion_mot_tabla.csv")
    with open(p1, "w", newline="", encoding="utf-8") as fh:
        w = _csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for f in res["filas"]:
            w.writerow({k: ("%.2f" % f[k] if isinstance(f[k], float) else f[k])
                        for k in cols})
    p2 = os.path.join(carpeta, tag + "_comparacion_mot_curvas.csv")
    g = np.linspace(0, 100, PM.N_CICLO)
    with open(p2, "w", newline="", encoding="utf-8") as fh:
        w = _csv.writer(fh)
        w.writerow(["coordenada", "lado", "version", "pct_ciclo", "media_deg", "DE_deg"])
        for (etiq, lado, nombre), (m, sd, _C) in res["curvas"].items():
            for k in range(len(g)):
                w.writerow([etiq, "-" if etiq in SIN_LADO else lado, nombre,
                            "%.0f" % g[k], "%.2f" % m[k], "%.2f" % sd[k]])
    return p1, p2


def figura(res, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    g = np.linspace(0, 100, PM.N_CICLO)
    fig, ax = plt.subplots(len(COORDS), 2, figsize=(13, 2.3 * len(COORDS)),
                           sharex=True)
    fig.suptitle("%s  ↔  %s — ciclo promedio ± 1 DE, mismos ciclos del IMU en las "
                 "tres versiones" % (res["tag"], res["trial"]), fontsize=12)
    for i, (etiq, _p) in enumerate(COORDS):
        for j, lado in enumerate(("derecha", "izquierda")):
            a = ax[i, j]
            if etiq in SIN_LADO and j == 1:
                a.axis("off")
                continue
            # OpenCap se dibuja ultimo para que quede encima: en lo que el IMU
            # no toca las curvas coinciden y si no, la referencia se tapa.
            for nombre, _pat, col in sorted(res["versiones"],
                                            key=lambda v: v[0] == "OpenCap"):
                k = (etiq, "derecha" if etiq in SIN_LADO else lado, nombre)
                if k not in res["curvas"]:
                    continue
                m, sd, C = res["curvas"][k]
                a.plot(g, m, color=col, lw=1.8 if nombre == "OpenCap" else 1.4,
                       label="%s (n=%d)" % (nombre, len(C)))
                if nombre != "+IMU":
                    a.fill_between(g, m - sd, m + sd, color=col, alpha=.12)
            d = res["desp"].get("derecha" if etiq in SIN_LADO else lado)
            if d:
                a.axvline(d, color="gray", ls="--", lw=.9)
            a.set_title("%s · %s" % (etiq, "pelvis" if etiq in SIN_LADO else lado),
                        fontsize=9.5)
            a.grid(alpha=.3); a.tick_params(labelsize=8)
            if i == 0 or (etiq in SIN_LADO and j == 0):
                a.legend(fontsize=7.5, loc="best")
            if j == 0:
                a.set_ylabel("grados", fontsize=8)
    for a in ax[-1]:
        a.set_xlabel("% del ciclo (0 = contacto inicial · línea = despegue)", fontsize=8.5)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(ruta, dpi=110)
    plt.close(fig)
    return ruta


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", nargs="?", default=None)
    p.add_argument("--todas", action="store_true")
    p.add_argument("--salida", default=DIR_SALIDA)
    p.add_argument("--mot", default=DIR_MOT,
                   help="carpeta con los tres .mot (default ../resultados)")
    a = p.parse_args()
    tomas = R.tomas_con_fecha() if a.todas else [a.toma]
    if not tomas or tomas == [None]:
        raise SystemExit("pasa una toma o --todas")
    for tag in sorted(tomas):
        print("\n%s" % tag)
        res = comparar(tag, dir_mot=a.mot)
        imprimir(res)
        p1, p2 = guardar(res, a.salida)
        p3 = figura(res, os.path.join(a.salida, tag + "_comparacion_mot.png"))
        for f in (p1, p2, p3):
            print("  -> %s" % f)


if __name__ == "__main__":
    main()
