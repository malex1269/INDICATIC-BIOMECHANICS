"""
fusionar_mot.py -- el .mot de OpenCap con los datos del IMU incrustados.

    python3 fusionar_mot.py testnorma2l9112026 --salida CARPETA
    python3 fusionar_mot.py --todas --salida CARPETA

LA IDEA
-------
El `.mot` de OpenCap es bueno: se ve bien en OpenSim y tiene las 33
coordenadas. La reconstruccion a partir del IMU tiene que INVENTAR casi todo,
porque el montaje S observa poco. Este programa hace lo del medio: **parte del
`.mot` de OpenCap y le mete SOLO lo que el IMU realmente mide**, dejando el
resto de OpenCap intacto.

Sirve para aislar el aporte del IMU. Comparando

    OpenCap  vs  OpenCap + IMU

se ve exactamente cuanto mueve la aguja el sensor inercial, sin que la
comparacion quede contaminada por todo lo que el prior tuvo que rellenar.
Es una pregunta distinta de la que responde `validar_reconstruccion.py`, que
mide "IMU + prior contra OpenCap".

QUE SE INCRUSTA -- solo lo medido, nada mas
-------------------------------------------
  la cadena sagital           `hip_flexion - knee_angle + ankle_angle` de cada
  (`cadena`)                  pierna. El IMU mide la SUMA, no los tres por
                              separado, asi que no se puede escribir cada
                              angulo: se REPARTE el residuo entre cadera y
                              rodilla conservando la suma medida, que es lo
                              mismo que hace `variedad.redistribuir`.
  la cadena del brazo         `arm_flex + elbow_flex` de cada lado, desde el
  (`brazos`)                  pitch del sensor de la MANO respecto de la
                              gravedad. Misma logica: el IMU mide la suma
                              hombro+codo, se reparte. Medido contra OpenCap
                              da r = 0.92-0.95 en el brazo derecho y 0.46-0.69
                              en el izquierdo (sesion 2026-09-10).
  pelvis_tilt, pelvis_list    inclinacion del sacro vs gravedad. OPCIONAL,
  (`pelvis`)                  ver abajo.

QUE NO SE TOCA
--------------
Todo lo demas queda como lo dejo OpenCap: rotacion de pelvis, traslaciones,
tobillo, subtalar, mtp, arm_add/arm_rot/pro_sup. El montaje S no los mide,
asi que meterlos seria inventar.

LA PELVIS, Y POR QUE VA CON COMPENSACION
----------------------------------------
`pelvis_tilt` de OpenCap NO es una referencia valida en estas tomas: deriva
25 deg en 12 s de marcha (ROM 36-91 deg) y correlaciona r = -0.98 con
`lumbar_extension`, o sea que OpenCap reparte la inclinacion entre pelvis y
lumbar de forma arbitraria manteniendo el torso quieto (SD del torso 4 deg
contra 9 del tilt). Contra eso el sacro del IMU da r = 0.1, pero el numero
no mide al IMU: mide el artefacto. Lo mismo, mas suave, con los muslos
(r(tilt, hip_flexion) = -0.8).

Por eso, cuando se inyecta la pelvis, se CONSERVA lo que OpenCap si observa
bien: la inclinacion del torso (`tilt + lumbar_extension`) y la de los
muslos (`tilt + hip_flexion`). Cada grado que cambia la pelvis se descuenta
de lumbar y caderas. Sin eso el modelo se hamaca 25 deg por culpa del
artefacto de OpenCap, y la comparacion no mide nada. En el plano frontal,
igual: `list + lumbar_bending`, `list + hip_adduction_r`,
`list - hip_adduction_l` (signos verificados en los propios .mot de OpenCap,
r = -0.85, -0.67, +0.70).

EL NIVEL ABSOLUTO
-----------------
Sin pose estatica de calibracion el IMU mide bien la FORMA de onda pero no el
cero absoluto: el marco funcional sale de una PCA y el offset es arbitrario.
Por eso cada senal incrustada se lleva a la media de la de OpenCap antes de
escribirla (`--sin-offset` lo desactiva). Lo que se compara entonces es la
forma, que es lo que el IMU aporta de verdad.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sparse6 as s6
import opencap_io as oc
import sincronizar as S
import reconstruir_toma as R
import mot_io

# Cuanto del residuo de la cadena se le da a la cadera y cuanto a la rodilla.
# La cadena es hip - knee + ankle, asi que para subir la cadena en `r` grados
# se puede subir la cadera o bajar la rodilla. El reparto 50/50 es el que menos
# deforma cada articulacion por separado; `variedad.redistribuir` usa el mismo
# criterio.
REPARTO_CADERA = 0.5


# La toma CRUDA, sin recortar. La fusion tiene que usarla: el trial de OpenCap
# cubre la sesion entera, asi que con la version recortada el solape se achica
# y la sincronizacion se cae (r pasa de 0.93 a 0.29 en caminata03). El recorte
# sirve para la app y para el .mot reconstruido, no para esto.
DIRS_CRUDO = R.carpetas_imu_en_opencap()


def csv_crudo(tag):
    for carpeta in DIRS_CRUDO:
        if not os.path.isdir(carpeta):
            continue
        cand = [os.path.join(carpeta, f) for f in sorted(os.listdir(carpeta))
                if f.startswith(tag) and f.endswith(".csv")
                and "run_log" not in f]
        if cand:
            return cand[0]
    raise SystemExit("no encuentro la toma cruda de '%s' en %s"
                     % (tag, " ni ".join(DIRS_CRUDO)))


def cadena_imu(csv, verbose=True):
    """Cadena sagital, brazos y pelvis medidos por el IMU, en su grilla.

    Devuelve (t, dict) con `pelvis_tilt`, `pelvis_list`, `cadena_r/_l` y
    `brazo_r/_l`. El brazo es el pitch ABSOLUTO de la mano (vs gravedad), no
    relativo al sacro: el torso no se mide, y en marcha se inclina poco; contra
    OpenCap el absoluto da r = 0.92 y el relativo al sacro 0.76.
    El signo del eje AP sale de una PCA y queda indeterminado; aca se resuelve
    probando los dos signos: se elige el que da mas
    correlacion con OpenCap. Eso lo hace `fusionar()`, que es quien tiene la
    referencia a mano.
    """
    D, t = R.preparar_imu(csv)
    if "espalda" not in D:
        raise SystemExit("falta el sensor de sacro: sin el no hay ni pelvis "
                         "ni cadena")
    out = {"pelvis_tilt": D["espalda"]["pitch"],
           "pelvis_list": D["espalda"]["roll"]}
    for seg, suf in (("pie_der", "r"), ("pie_izq", "l")):
        if seg in D:
            out["cadena_" + suf] = D[seg]["pitch"] - D["espalda"]["pitch"]
        elif verbose:
            print("  aviso: falta %s, esa pierna queda 100 %% OpenCap" % seg)
    for seg, suf in (("mano_der", "r"), ("mano_izq", "l")):
        if seg in D:
            out["brazo_" + suf] = D[seg]["pitch"]
        elif verbose:
            print("  aviso: falta %s, ese brazo queda 100 %% OpenCap" % seg)
    return t, out


def _a_grilla(x, t_imu, t_oc, off):
    """Lleva una senal del IMU a la grilla de tiempo de OpenCap.

    `off` es el arranque que devuelve `sincronizar`: el instante t de OpenCap
    cae en el instante (t + off) del IMU.
    """
    return np.interp(t_oc + off, t_imu, x)


def _alinear_signo(x, ref):
    """Devuelve (x_con_signo, signo) eligiendo el que correlaciona con ref."""
    r = np.corrcoef(x - x.mean(), ref - ref.mean())[0, 1]
    return (x, 1.0) if r >= 0 else (-x, -1.0)


def _quitar_offset(x, ref, m=None):
    if m is None:
        return x - (x.mean() - ref.mean())
    return x - (x[m].mean() - ref[m].mean())


def _repartir(res, a, b, signo_b, lim_b):
    """Reparte el residuo `res` de una cadena `a + signo_b * b`.

    Reparto CONSCIENTE DEL RANGO: `b` (rodilla, codo) ya viene pegado a su
    limite inferior en buena parte del trial, asi que un 50/50 ciego lo
    empuja fuera y `escribir_mot` lo recorta -- y una rodilla recortada se ve
    como pierna quebrada. Se le da a `b` solo lo que aguanta dentro de
    `lim_b` y el sobrante va entero a `a`, que tiene rango de sobra.
    Devuelve (a_nuevo, b_nuevo) con a + signo_b * b == cadena + res.
    """
    d_b = signo_b * (1.0 - REPARTO_CADERA) * res
    d_b = np.clip(b + d_b, *lim_b) - b
    sobra = (1.0 - REPARTO_CADERA) * res - signo_b * d_b
    return a + REPARTO_CADERA * res + sobra, b + d_b


def peso_marcha(csv, t_imu, t_oc, off, rampa=0.25):
    """Peso 0..1 por muestra de OpenCap: 1 caminando, 0 parado.

    Fuera de la marcha ninguno de los dos sistemas esta diciendo nada util
    -- OpenCap se va (su `pelvis_tilt` llega a 90 deg de rango en los tramos
    de giro y agacharse) y el IMU tampoco tiene ciclo. Inyectar ahi mete
    correcciones de 15 deg que sacan la rodilla del rango del modelo en el
    30 % de las muestras. Se inyecta SOLO donde hay marcha, con una rampa
    coseno de `rampa` segundos a cada lado para que no salte.
    """
    import recortar_toma as RT
    _t, marcha, _tr, _res = RT.analizar_tramos(csv, verbose=False)
    w = np.interp(t_oc + off, t_imu, marcha.astype(float))
    w = (w > 0.5).astype(float)
    n = max(1, int(rampa / np.median(np.diff(t_oc))))
    k = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, 2 * n + 1))
    k /= k.sum()
    return np.clip(np.convolve(w, k, mode="same"), 0.0, 1.0)


def fusionar(csv, trc, ruta_mot, verbose=True, con_offset=True,
             canales=("cadena", "brazos")):
    """Devuelve (t, datos, informe) del .mot fusionado."""
    sync = S.sincronizar(csv, trc, verbose=False)
    off = sync["arranque_s"]
    t_oc, df = oc.leer_mot(ruta_mot)
    t_imu, med = cadena_imu(csv, verbose)
    w = peso_marcha(csv, t_imu, t_oc, off)
    m = w > 0.5                                # donde de verdad hay marcha

    datos = {c: df[c].to_numpy(float).copy() for c in mot_io.COLUMNAS
             if c in df.columns}
    for c in mot_io.COLUMNAS:
        datos.setdefault(c, np.zeros(len(t_oc)))

    informe = {"sync_r": sync["r"], "arranque_s": off, "n": len(t_oc),
               "pct_marcha": float(100 * m.mean()), "canales": {}}
    informe["sync_metodo"] = sync["metodo"]
    if sync["metodo"] == "golpes":
        informe.update(sync_precision_ms=sync["precision_ms"],
                       sync_n_golpes=sync["n_golpes"])
    if verbose:
        if sync["metodo"] == "golpes":
            print("  sincronizacion por %d golpes de talon: OpenCap arranca "
                  "%+.3f s despues (precision %.0f ms; la correlacion daba "
                  "%+.3f s, r=%.2f)" % (sync["n_golpes"], off,
                                        sync["precision_ms"],
                                        sync["arranque_correlacion_s"],
                                        sync["r"]))
        else:
            print("  sincronizacion por correlacion: r=%.2f, OpenCap arranca "
                  "%+.2f s despues" % (sync["r"], off))
        print("  se inyecta en el %.0f %% del trial (los tramos de marcha)"
              % (100 * m.mean()))

    # ---- pelvis: se reemplaza, conservando torso y muslos
    #
    # POR OMISION NO SE INYECTA, porque no hay contra que validarla: el
    # `pelvis_tilt` de OpenCap deriva 25 deg en marcha y se compensa 1:1 con
    # `lumbar_extension` (r = -0.98). Ver el docstring. Se activa con
    # `--canales pelvis,...` para ver cuanto cambia el .mot.
    #
    # Cada grado que cambia la pelvis se DESCUENTA de las articulaciones que
    # cuelgan de ella, para que el torso y los muslos --que OpenCap si ve--
    # queden donde estaban: canal -> ((coordenada que compensa, signo), ...).
    COMPENSA = {"pelvis_tilt": (("lumbar_extension", -1.0),
                                ("hip_flexion_r", -1.0),
                                ("hip_flexion_l", -1.0)),
                "pelvis_list": (("lumbar_bending", -1.0),
                                ("hip_adduction_r", -1.0),
                                ("hip_adduction_l", +1.0))}
    for canal in (("pelvis_tilt", "pelvis_list") if "pelvis" in canales
                  else ()):
        x = _a_grilla(med[canal], t_imu, t_oc, off)
        ref = datos[canal]
        x, sg = _alinear_signo(x[m] if m.any() else x, ref[m] if m.any() else ref)
        x = sg * _a_grilla(med[canal], t_imu, t_oc, off)
        if con_offset:
            x = _quitar_offset(x, ref, m)
        r = float(np.corrcoef(x[m] - x[m].mean(), ref[m] - ref[m].mean())[0, 1])
        delta = w * (x - ref)                  # lo que cambia la pelvis
        informe["canales"][canal] = dict(
            signo=sg, r=r, rmse=float(np.sqrt(np.mean(delta[m] ** 2))),
            rom_imu=float(x[m].max() - x[m].min()),
            rom_oc=float(ref[m].max() - ref[m].min()),
            correccion_media=float(np.mean(np.abs(delta[m]))))
        datos[canal] = ref + delta
        for c, sgn in COMPENSA[canal]:
            datos[c] = datos[c] + sgn * delta
        if verbose:
            print("    %-12s signo %+.0f  r=%+.3f  RMSE=%5.1f deg  "
                  "ROM %5.1f (IMU) vs %5.1f (OpenCap)  -> compensado en %s"
                  % (canal, sg, r, np.sqrt(np.mean(delta[m] ** 2)),
                     x[m].max() - x[m].min(), ref[m].max() - ref[m].min(),
                     ", ".join(c for c, _ in COMPENSA[canal])))

    # ---- cadenas: se reparte el residuo entre las dos articulaciones
    #
    # (clave del IMU, canal que lo activa, coord a, coord b, signo de b,
    #  rango de b, coord extra). La cadena es a + signo_b * b (+ extra).
    CADENAS = [("cadena_r", "cadena", "hip_flexion_r", "knee_angle_r", -1.0,
                (0.0, 140.0), "ankle_angle_r"),
               ("cadena_l", "cadena", "hip_flexion_l", "knee_angle_l", -1.0,
                (0.0, 140.0), "ankle_angle_l"),
               ("brazo_r", "brazos", "arm_flex_r", "elbow_flex_r", +1.0,
                (0.0, 150.0), None),
               ("brazo_l", "brazos", "arm_flex_l", "elbow_flex_l", +1.0,
                (0.0, 150.0), None)]
    for clave, canal, ca, cb, sgb, lim_b, extra in CADENAS:
        if clave not in med or canal not in canales:
            continue
        cad_oc = datos[ca] + sgb * datos[cb]
        if extra:
            cad_oc = cad_oc + datos[extra]
        x = _a_grilla(med[clave], t_imu, t_oc, off)
        x, sg = _alinear_signo(x[m] if m.any() else x,
                               cad_oc[m] if m.any() else cad_oc)
        x = sg * _a_grilla(med[clave], t_imu, t_oc, off)
        if con_offset:
            x = _quitar_offset(x, cad_oc, m)
        res = w * (x - cad_oc)                 # lo que el IMU dice de mas
        datos[ca], datos[cb] = _repartir(res, datos[ca], datos[cb], sgb, lim_b)
        r = float(np.corrcoef(x[m] - x[m].mean(),
                              cad_oc[m] - cad_oc[m].mean())[0, 1])
        informe["canales"][clave] = dict(
            signo=sg, r=r, rmse=float(np.sqrt(np.mean(res[m] ** 2))),
            rom_imu=float(x[m].max() - x[m].min()),
            rom_oc=float(cad_oc[m].max() - cad_oc[m].min()),
            correccion_media=float(np.mean(np.abs(res[m]))))
        if verbose:
            print("    %-9s signo %+.0f  r=%+.3f  RMSE=%5.1f deg  "
                  "ROM %5.1f (IMU) vs %5.1f (OpenCap)  correccion media %4.1f"
                  % (clave, sg, r, np.sqrt(np.mean(res[m] ** 2)),
                     x[m].max() - x[m].min(), cad_oc[m].max() - cad_oc[m].min(),
                     np.mean(np.abs(res[m]))))
    return t_oc, datos, informe


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", nargs="?", default=None,
                   help="tag de la toma (testnorma2l9112026)")
    p.add_argument("--todas", action="store_true",
                   help="todas las tomas con par de OpenCap")
    p.add_argument("--salida", default=R.DIR_SALIDAS)
    p.add_argument("--canales", default="cadena,brazos",
                   help="que incrustar, separado por comas: `cadena` (pierna, "
                        "r=0.5-0.7), `brazos` (r=0.9 der / 0.5-0.7 izq), "
                        "`pelvis` (sin referencia valida, ver docstring). "
                        "DEFAULT cadena,brazos")
    p.add_argument("--sufijo", default="",
                   help="se agrega al nombre del .mot, p. ej. `_con_pelvis`")
    p.add_argument("--sin-offset", action="store_true", dest="sin_offset",
                   help="no llevar cada senal a la media de OpenCap")
    a = p.parse_args()

    tomas = R.tomas_con_fecha() if a.todas else [a.toma]
    if not tomas or tomas == [None]:
        raise SystemExit("pasa una toma o --todas")
    if not os.path.isdir(a.salida):
        os.makedirs(a.salida)

    import validar_reconstruccion as VR
    for tag in sorted(tomas):
        trial = R.PARES.get(tag)
        if not trial:
            print("%s: no tiene par de OpenCap, no se puede fusionar" % tag)
            continue
        csv = csv_crudo(tag)
        trc, ruta_mot = VR.rutas_oc(tag, trial)
        print("\n%s  <->  %s" % (tag, trial))
        t, datos, _inf = fusionar(csv, trc, ruta_mot,
                                  con_offset=not a.sin_offset,
                                  canales=tuple(a.canales.split(",")))
        osim = R.osim_de(tag)
        lim = mot_io.limites_osim(osim) if os.path.isfile(osim) else None
        destino = os.path.join(a.salida,
                               tag + "_opencap_mas_imu" + a.sufijo + ".mot")
        destino, faltan, recortado = mot_io.escribir_mot(destino, t, datos,
                                                         limites=lim)
        print("  -> %s" % os.path.basename(destino))
        for c, pct in sorted(recortado.items(), key=lambda kv: -kv[1]):
            print("     recortado al rango del modelo: %-16s %.1f %%" % (c, pct))


if __name__ == "__main__":
    main()
