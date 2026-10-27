"""
parametros_marcha.py -- parametros de marcha del montaje S. SOLO LO MEDIDO.

    python parametros_marcha.py testnorma2l9112026
    python parametros_marcha.py testnorma2l9112026 --figuras --salida CARPETA

REGLA DE ESTE ARCHIVO
---------------------
Aca **solo entra lo que el montaje S mide de verdad**. Nada de cadera, rodilla
ni tobillo por separado: con 6 sensores en cabeza, sacro, manos y pies no hay
sensor en muslo ni en pierna, asi que esos angulos NO son observables -- de la
pierna solo se mide la SUMA `hip_flexion - knee_angle + ankle_angle`.

Esos angulos si se pueden RECONSTRUIR apoyandose en un prior de marcha del
sujeto, y para eso esta `reconstruir_toma.py`, que escribe el `.mot` para
animar en OpenSim. Pero eso es reconstruccion, no medicion, y validado da
paridad con reproducir el ciclo medio del sujeto -- asi que **no tiene lugar
en una tabla de parametros extraidos**. Los dos programas estan separados a
proposito.

Sin magnetometro el rumbo deriva ~1 deg/s, asi que tampoco entra nada que
dependa del rumbo: ni rotacion de pelvis, ni angulo de progresion del pie, ni
trayectoria curva.

DE DONDE SALE LA MAQUINARIA
---------------------------
De `Mythos MECH` (el trabajo previo del IIT), que ya tiene la biblioteca hecha
y apoyada en literatura: fusion VQF (Laidig & Seel 2023), eventos de marcha
por giroscopio (Aminian 2002 / Salarian 2004), deteccion de giros, parametros
temporales y espaciales, y el formato de resultados. Este programa **adapta
nuestros datos a esa biblioteca**; no la reescribe.

Lo unico que no se puede reusar es su calculo de angulos articulares, porque
su montaje (sacro + muslo + pierna + pie) cierra la cadena cinematica y el
nuestro no.

LAS TOMAS NO SON CAMINATAS LIMPIAS
-----------------------------------
En las tomas del 2026-09-01 el sujeto para, arranca, acelera, gira y se
agacha. Por eso el programa **segmenta la actividad antes de medir nada** y
calcula los parametros SOLO sobre los tramos de marcha recta. El reparto del
tiempo (parado / marcha / giro / otro) se reporta siempre: si la fraccion de
marcha es baja, los parametros salen de pocas zancadas y hay que decirlo.

SALIDAS
-------
    <salida>/<toma>_parametros.json    parametros (solo medidos)
    <salida>/<toma>_series.csv         trazas + banderas de evento
    <salida>/<toma>_eventos.csv        tabla de eventos
    <salida>/<toma>_figura.png         con --figuras (la figura de la app)
    <salida>/<toma>_figura_detalle.png con --detalle (analisis completo)
"""
import argparse
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
# Mythos MECH se importa, no se copia (en el repositorio); la copia de entrega
# lleva `mythos_mech/` al lado. `rutas.py` decide cual de las dos.
from rutas import RAIZ, DIR_MYTHOS, DIR_SALIDAS

import sparse6 as s6
from mapa_sensores import mapa_de

if not os.path.isdir(DIR_MYTHOS):
    raise SystemExit(
        "No encuentro la biblioteca de Mythos MECH en:\n  %s\n"
        "Pone la carpeta ahi (o `mythos_mech/` al lado de este script), "
        "o exporta MYTHOS_MECH=/ruta/a/Mythos MECH"
        % os.path.abspath(DIR_MYTHOS))
sys.path.insert(0, DIR_MYTHOS)
try:
    from core.fusion_vqf import fuse_segment
    from kinematics import gait_events as G
    from kinematics import joint_angles as A
    from kinematics import parameters as P
    from kinematics.results import KinematicResults
except ImportError as e:
    raise SystemExit("Falta una dependencia de Mythos MECH (%s).\n"
                     "Instala:  pip install vqf scipy matplotlib" % e)

G_MS2 = 9.81
FS = 100.0

PIES = [("pie_der", "r", "derecha"), ("pie_izq", "l", "izquierda")]
# Segmentos cuya inclinacion respecto de la gravedad SI es medicion directa.
INCLINACIONES = [("espalda", "pelvis"), ("cabeza", "cabeza"),
                 ("mano_der", "brazo_der"), ("mano_izq", "brazo_izq"),
                 ("pie_der", "pie_der"), ("pie_izq", "pie_izq")]


# --------------------------------------------------------------- adaptador
def cargar_streams(csv, mapa=None):
    """CSV nuestro -> el formato de stream que espera Mythos.

    Mythos pide un CSV por sensor con acc en m/s2 y gyro en rad/s; nosotros
    tenemos un CSV con los 6 sensores en filas y columna `mac`, acc en g y
    gyro en deg/s. Esta funcion es todo el pegamento que hace falta.
    """
    mapa = mapa or mapa_de(csv)
    R, t = s6.cargar_toma(csv)
    streams = {}
    for mac, seg in mapa.items():
        if mac not in R:
            continue
        _, acc_g, gyro_dps = R[mac]
        # Marco funcional del segmento (x anterior, y izquierda, z superior),
        # deducido de la propia senal. Sin esto, "sagital" y "frontal" quedan
        # en el marco CRUDO del sensor, o sea que dependen de como quedo
        # pegado: la cabeza daba 0.9 deg sagital y 48 deg frontal, con los
        # ejes cambiados. No es cosmetico, invalida el ROM.
        rep = s6.detectar_reposo(acc_g, gyro_dps)
        Rseg, _ = s6.marco_segmento(acc_g, rep, ~rep)
        streams[seg] = {"t": t,
                        "acc": acc_g * G_MS2,          # g     -> m/s2
                        "gyr": np.radians(gyro_dps),   # deg/s -> rad/s
                        "mag": None, "mag_present": False,
                        "acc_g": acc_g, "gyro_dps": gyro_dps,
                        "Rseg": Rseg, "reposo": rep}
    return streams, t


def fusionar(streams, fs=FS):
    """Orientacion VQF de cada segmento. Modo 6D: no hay magnetometro."""
    for st in streams.values():
        q, eff = fuse_segment(st, mode="6D", imu_hz=fs)
        st["q"] = np.asarray(q, float)
        st["modo"] = eff
    return streams


# ------------------------------------------------------------------ eventos
def eventos_dos_metodos(st, t, fs=FS):
    """Eventos por el metodo de Mythos (giroscopio) y por el nuestro (ZUPT).

    NO detectan el mismo evento: Mythos marca el contacto inicial (la
    inversion de velocidad angular tras el balanceo) y el ZUPT marca el
    arranque de la ventana de pie plano, que llega despues -- lo que dura la
    respuesta de carga, ~10-15 % del ciclo. Un desfase POSITIVO de ese orden
    confirma que los dos funcionan; uno negativo o erratico avisa que alguno
    fallo. Es verificacion cruzada sin sistema de referencia.
    """
    ev = G.detect_events(st["gyr"], fs)
    ic, fo, reposo = s6.eventos_marcha(t, st["acc_g"], st["gyro_dps"])
    a = np.asarray(ev["foot_strike"], int)
    ic = np.asarray(ic, int)
    if len(a) and len(ic):
        d = np.array([ic[np.argmin(np.abs(ic - x))] - x for x in a], float)
        desfase_ms = float(np.median(d)) / fs * 1000.0
        cerca = int(np.sum(np.abs(d - np.median(d)) < 0.10 * fs))
    else:
        desfase_ms, cerca = float("nan"), 0
    ev["zupt_foot_strike"] = ic
    ev["zupt_toe_off"] = np.asarray(fo, int)
    ev["reposo"] = reposo
    ev["acuerdo"] = {"n_giroscopio": int(len(a)), "n_zupt": int(len(ic)),
                     "consistentes_100ms": cerca,
                     "desfase_mediano_ms": desfase_ms}
    return ev


# Duracion admisible de una zancada. El maximo era 2.0 s hasta la sesion
# 2026-09-11, donde el sujeto camino a 0.35 m/s con zancadas de 2.0-2.4 s y
# el 80 % de la marcha quedaba clasificada como "otro" (y la fusion no
# inyectaba ahi). La marcha lenta / patologica que se quiere medir es
# justamente esa, asi que el tope tiene que dejarla pasar. Los pasitos de las
# vueltas no se filtran por duracion sino por forma (`ciclos_del_lado`).
DUR_ZANCADA = (0.7, 3.0)


def ciclos_validos(t, ic, dur_min=DUR_ZANCADA[0], dur_max=DUR_ZANCADA[1]):
    """Ciclos (i0, i1) entre contactos consecutivos con duracion fisiologica."""
    return [(int(ic[p]), int(ic[p + 1])) for p in range(len(ic) - 1)
            if dur_min < t[ic[p + 1]] - t[ic[p]] < dur_max]


# ------------------------------------------------------ actividad
def segmentar_actividad(streams, eventos, t, giros, fs=FS):
    """Reparte cada muestra en parado / marcha / giro / otro.

    Hace falta porque estas tomas NO son caminatas limpias: el sujeto para,
    arranca, gira y se agacha. Medir cadencia sobre la toma entera mezclaria
    todo eso.

      parado  los dos pies en reposo (ZUPT) al mismo tiempo
      giro    lo que marca la deteccion de giros del sacro
      marcha  dentro de un ciclo de marcha valido de cualquiera de los pies
      otro    se mueve, pero no es ni marcha ni giro: agacharse, estirarse,
              acomodarse. Es la categoria honesta para lo que no sabemos leer.
    """
    n = len(t)
    marcha = np.zeros(n, bool)
    for lado, ev in eventos.items():
        for i0, i1 in ciclos_validos(t, np.asarray(ev["zupt_foot_strike"], int)):
            marcha[i0:i1] = True

    reposos = [ev["reposo"] for ev in eventos.values()]
    parado = np.logical_and.reduce(reposos) if reposos else np.zeros(n, bool)

    giro = np.zeros(n, bool)
    for a, b, _ in giros:
        giro[a:b] = True

    parado &= ~marcha
    giro &= ~marcha
    otro = ~(marcha | parado | giro)
    return {"marcha": marcha, "parado": parado, "giro": giro, "otro": otro}


# ------------------------------------------------------------------ ciclos
# El ciclo promedio se ancla en el CONTACTO INICIAL y no en el arranque del
# pie plano (ZUPT), que llega 100-400 ms despues y con jitter; ni en los
# eventos del giroscopio de Mythos, que en estas tomas mezclan los pasitos de
# los giros con las zancadas (zancadas de 0.7 y 3.7 s en caminata01_4). Cada
# ciclo se ancla en el pico de medio balanceo del pie (el rasgo mas nitido de
# la senal: uno por zancada) y de ahi se busca el contacto: el arranque del
# "golpe" del pie (la caida rapida de la punta tras apoyar el talon). Con eso
# el tiempo de zancada sale con CV 5-12 %, contra el 31-38 % de la tabla
# temporal en la misma toma.
N_CICLO = 101


def contacto_inicial(pitch_pie, util, fs=FS, umbral_dps=-30.0, max_adelante_s=0.6):
    """Contactos iniciales del pie, uno por balanceo.

    `pitch_pie` con el signo fisiologico (punta abajo = negativo, ver
    `_signo_pie`). Primero el pico positivo de velocidad angular de cada
    balanceo (el pie sube la punta); despues, hacia adelante, la velocidad mas
    negativa (el golpe del talon) y se retrocede hasta donde arranca. Si no
    hay golpe (pisada plana), el contacto es donde el pie deja de subir.
    """
    from scipy.signal import find_peaks
    w = np.gradient(np.asarray(pitch_pie, float)) * fs
    ref = w[util] if util.any() else w
    picos, _ = find_peaks(w, height=0.4 * np.percentile(ref, 97),
                          distance=int(0.5 * fs))
    out = []
    for p in picos:
        seg = w[p:p + int(max_adelante_s * fs)]
        if len(seg) < 3:
            continue
        j = p + int(np.argmin(seg))
        if w[j] < umbral_dps:
            while j > p and w[j] < umbral_dps:
                j -= 1
        else:
            z = np.flatnonzero(seg < 0)
            j = p + (int(z[0]) if len(z) else 0)
        out.append(j)
    return np.unique(np.asarray(out, int))


def ciclos_limpios(t, ic, util, dur_min=DUR_ZANCADA[0], dur_max=DUR_ZANCADA[1],
                   tol=0.25):
    """Ciclos (i0, i1) entre contactos consecutivos, en marcha recta, de
    duracion fisiologica y a +-`tol` de la mediana (saca pausas y eventos
    perdidos)."""
    cic = [(int(i0), int(i1)) for i0, i1 in zip(ic[:-1], ic[1:])
           if dur_min < t[i1] - t[i0] < dur_max and util[i0:i1].mean() > 0.9]
    if tol and len(cic) >= 3:
        T = np.array([t[i1] - t[i0] for i0, i1 in cic])
        med = np.median(T)
        cic = [c for c, d in zip(cic, T) if abs(d - med) <= tol * med]
    return cic


def _remuestrear(x, cic, n=N_CICLO):
    g = np.linspace(0, 1, n)
    return np.array([np.interp(g, np.linspace(0, 1, i1 - i0), x[i0:i1])
                     for i0, i1 in cic])


def ciclos_del_lado(res, lado, r_min=0.6):
    """Ciclos limpios de un lado, y cuales son tipicos.

    El pie define la calidad del ciclo: se descartan los ciclos cuya forma no
    se parece a la mediana (r < `r_min`). Sin esto, en caminata01_4 se
    promediaban 7 s de arranque arrastrando los pies con la marcha normal, y
    la media salia aplastada (ROM 14 grados cuando cada zancada tiene 60-90).
    En caminata01_4 el pasillo mide 3.2 m: son 3 zancadas, vuelta con pasitos,
    3 zancadas. Quedan ~5 zancadas tipicas de 14 ciclos por lado.
    Devuelve dict con `cic` (los usados), `n_total`, `T` (duraciones),
    `despegue_pct` (despegue de la punta, mediana), `despegue_talon_pct`
    (fin del pie plano por ZUPT) o None.
    """
    nm = "pie_der_sagital" if lado == "derecha" else "pie_izq_sagital"
    if lado not in res.eventos_por_lado or nm not in res.joints:
        return None
    ev = res.eventos_por_lado[lado]
    pit = res.joints[nm]["flexion"]
    t, util = res.t, res.steady_state
    ic = contacto_inicial(pit, util, res.fs)
    # Sin filtro de duracion todavia: se aplica DESPUES de elegir las
    # zancadas grandes, relativo a la mediana de esas. Con la mediana de
    # todos los candidatos, en testnorma2l (2026-09-11) los pasitos de
    # entrar en camara y de las vueltas (1.0-1.3 s) eran mayoria, la mediana
    # caia en 1.27 s y las zancadas de la marcha lenta (1.7-2.2 s) quedaban
    # todas afuera: se comparaba OpenCap con el IMU justo donde OpenCap no
    # veia al sujeto.
    cic = ciclos_limpios(t, ic, util, tol=None)
    if not cic:
        return None
    C = _remuestrear(pit, cic)
    T = np.array([t[i1] - t[i0] for i0, i1 in cic])
    ok = np.ones(len(cic), bool)
    if len(cic) >= 4:
        # La referencia sale de las zancadas GRANDES: en un pasillo corto la
        # mitad de los ciclos son los pasitos de la vuelta (ROM 15-25 grados
        # contra 60-90 de la zancada), y la mediana de todo mezclaria ambos.
        rom = np.ptp(C, axis=1)
        grandes = rom >= 0.6 * np.percentile(rom, 75)
        med_T = np.median(T[grandes])
        grandes &= np.abs(T - med_T) <= 0.25 * med_T
        medn = np.median(C[grandes], axis=0)
        ok = np.array([np.corrcoef(c, medn)[0, 1] >= r_min for c in C]) & grandes
        if ok.sum() < 3:
            ok[:] = True
    elif len(cic) >= 3:
        med_T = np.median(T)
        ok = np.abs(T - med_T) <= 0.25 * med_T
    usados = [c for c, k in zip(cic, ok) if k]
    # Despegue de la punta ~ el pico de plantiflexion rapida del pie (la
    # velocidad angular mas negativa entre el 35 y el 90 % del ciclo). El fin
    # del ZUPT es el despegue del TALON, que llega antes (~50 %); se guarda
    # aparte.
    w = np.gradient(pit) * res.fs
    fo = np.asarray(ev["zupt_toe_off"], int)
    desp, talon = [], []
    for i0, i1 in usados:
        a, b = i0 + int(0.35 * (i1 - i0)), i0 + int(0.90 * (i1 - i0))
        desp.append(100.0 * (a + int(np.argmin(w[a:b])) - i0) / (i1 - i0))
        f = fo[(fo > i0) & (fo < i1)]
        if len(f):
            talon.append(100.0 * (f[0] - i0) / (i1 - i0))
    return {"cic": usados, "n_total": len(cic),
            "T": np.array([t[i1] - t[i0] for i0, i1 in usados]),
            "despegue_pct": float(np.median(desp)) if desp else None,
            "despegue_talon_pct": float(np.median(talon)) if talon else None}


def ciclo_promedio(x, cic, centrar=False):
    """(grid 0-100, ciclos [n, N], media, DE) de una traza sobre `cic`."""
    C = _remuestrear(np.asarray(x, float), cic)
    if centrar:
        C = C - C.mean(axis=1, keepdims=True)
    return np.linspace(0, 100, N_CICLO), C, C.mean(0), C.std(0)


def _signo_pie(pit, util):
    """Signo fisiologico del pitch del pie: en marcha la excursion grande es
    el despegue con la punta hacia abajo, que tiene que ser NEGATIVA. El eje
    AP sale de una PCA que no fija el sentido; esta regla si lo fija."""
    x = pit[util] if util.any() else pit
    return -1.0 if np.nanmax(x) > -np.nanmin(x) else 1.0


# ---------------------------------------------------------------- simetria
def indice_simetria(x_der, x_izq):
    """SI = 100*(der - izq)/(0.5*(der + izq)). Robinson et al. (1987).

    0 % = simetrico; positivo = la derecha es mayor.
    """
    if not (np.isfinite(x_der) and np.isfinite(x_izq)):
        return float("nan")
    m = 0.5 * (x_der + x_izq)
    return float("nan") if abs(m) < 1e-9 else float(100.0 * (x_der - x_izq) / m)


# ---------------------------------------------------------------- analisis
def analizar(csv, fs=FS, min_turn_deg=60.0, rate_thresh_dps=30.0, verbose=True):
    """Corre todo y devuelve un `KinematicResults` de Mythos."""
    streams, t = cargar_streams(csv)
    if "espalda" not in streams:
        raise SystemExit("falta el sensor de sacro: sin el no hay nada")
    fusionar(streams, fs)
    n = len(t)
    sac = streams["espalda"]

    # Giros, desde el sacro. Los umbrales de Mythos (120 deg, 50 deg/s) estan
    # pensados para un 2-min walk con vueltas de 180 deg; estas tomas giran
    # menos y mas suave, asi que se bajan.
    giros, _ = G.detect_turnarounds(sac["acc"], sac["gyr"], fs,
                                    min_turn_deg=min_turn_deg,
                                    rate_thresh_dps=rate_thresh_dps)

    eventos = {}
    for seg, suf, lado in PIES:
        if seg in streams:
            eventos[lado] = eventos_dos_metodos(streams[seg], t, fs)
        elif verbose:
            print("  aviso: falta %s" % seg)
    if not eventos:
        raise SystemExit("no hay ningun sensor de pie utilizable")

    act = segmentar_actividad(streams, eventos, t, giros, fs)
    # Los parametros se miden SOLO sobre marcha que ademas no sea giro.
    util = act["marcha"] & ~act["giro"]

    # --- trazas MEDIDAS: inclinacion respecto de la gravedad. Es lo unico que
    # el acelerometro ancla y que por lo tanto no deriva sin magnetometro.
    trazas = {}
    for seg, nombre in INCLINACIONES:
        if seg not in streams:
            continue
        pit, rol = s6.inclinacion(streams[seg]["q"], streams[seg]["Rseg"])
        if seg.startswith("pie"):
            pit = _signo_pie(pit, util) * pit
        trazas[nombre + "_sagital"] = pit
        trazas[nombre + "_frontal"] = rol
    # La cadena sagital pelvis->pie: la UNICA informacion de la pierna que
    # este montaje mide. Es la suma hip - knee + ankle, no sus partes. Se
    # calcula como pitch del pie menos pitch del sacro, las dos inclinaciones
    # respecto de la gravedad: exactamente lo que incrusta `fusionar_mot.py`
    # y lo que da r = 0.8-0.9 contra OpenCap por ciclo.
    #
    # Antes salia de `joint_flexion` de Mythos, que proyecta la gravedad sobre
    # el plano perpendicular al eje funcional (PCA del giroscopio) de cada
    # segmento. Con el sacro eso es fragil: casi no cabecea, la PCA es poco
    # nitida y cuando el eje sale vertical la proyeccion es ruido, el atan2
    # devuelve angulos al azar en +-180 y el `unwrap` suma una vuelta por
    # zancada (testnorma2l del 2026-09-11: la cadena se iba a 2500 grados).
    # El signo del pitch del sacro es el de su marco funcional (PCA, ver
    # `sparse6.marco_segmento`): ambiguo, pero aporta <= 10 grados sobre los
    # 60-90 del pie, y es la misma convencion que la fusion.
    for seg, suf, lado in PIES:
        if seg not in streams or "pelvis_sagital" not in trazas:
            continue
        trazas["cadena_" + suf] = trazas[seg + "_sagital"] - trazas["pelvis_sagital"]

    # Ciclos limpios por lado (ver `ciclos_del_lado`). La pelvis y la cabeza
    # se refieren al pie derecho.
    res_tmp = type("R", (), {})()
    res_tmp.t, res_tmp.fs, res_tmp.steady_state = t, fs, util
    res_tmp.eventos_por_lado = eventos
    res_tmp.joints = {k: {"flexion": v} for k, v in trazas.items()}
    ciclos = {lado: ciclos_del_lado(res_tmp, lado) for lado in eventos}
    ppal = "derecha" if ciclos.get("derecha") else next(
        (l for l in ciclos if ciclos[l]), None)

    articulaciones = {}
    for nombre, x in trazas.items():
        vel = A.derivative(x, fs)
        pr = P.joint_parameters(x, vel, util, fs)
        # ("pie_izq_sagital" termina en "_sagital": mirar el segmento, no el
        # final del nombre)
        lado = ("derecha" if ("_der" in nombre or nombre.endswith("_r")) else
                "izquierda" if ("_izq" in nombre or nombre.endswith("_l")) else
                ppal)
        cc = ciclos.get(lado)
        if cc:
            _g, C, medio, _s = ciclo_promedio(x, cc["cic"])
            pr["rom_cycle_deg"] = float(np.nanmax(medio) - np.nanmin(medio))
            pr["rom_por_ciclo_media_deg"] = float(np.ptp(C, axis=1).mean())
            pr["rom_por_ciclo_de_deg"] = float(np.ptp(C, axis=1).std())
            pr["n_ciclos_promediados"] = int(len(C))
        articulaciones[nombre] = {"flexion": x, "flexion_gravity_only": x,
                                  "ang_vel": vel, "params": pr}

    temporales, espaciales = {}, {}
    for seg, suf, lado in PIES:
        if lado not in eventos:
            continue
        temporales[lado] = P.temporal_parameters(eventos[lado], util, fs)
        espaciales[lado] = P.stride_length_zupt(streams[seg]["q"], streams[seg]["acc"],
                                                streams[seg]["gyr"], eventos[lado],
                                                util, fs)

    cad = [temporales[l]["cadence_steps_per_min"] for l in temporales]
    temporal = {"cadence_steps_per_min": float(np.nanmean(cad)),
                "derecha": temporales.get("derecha", {}),
                "izquierda": temporales.get("izquierda", {})}
    if len(temporales) == 2:
        # Simetria SOLO sobre cantidades medidas (tiempos y braceo). Nada de
        # simetria de ROM articular: esos angulos no se miden.
        sim = {}
        for k, etiq in (("stride_time_mean_s", "tiempo_de_zancada"),
                        ("stance_pct", "porcentaje_de_apoyo"),
                        ("cadence_steps_per_min", "cadencia")):
            sim["si_" + etiq + "_pct"] = indice_simetria(
                temporales["derecha"].get(k, float("nan")),
                temporales["izquierda"].get(k, float("nan")))
        for a, b, etiq in (("brazo_der_sagital", "brazo_izq_sagital", "braceo"),
                           ("cadena_r", "cadena_l", "cadena_sagital")):
            if a in articulaciones and b in articulaciones:
                sim["si_rom_" + etiq + "_pct"] = indice_simetria(
                    articulaciones[a]["params"].get("rom_cycle_deg", float("nan")),
                    articulaciones[b]["params"].get("rom_cycle_deg", float("nan")))
        temporal["simetria"] = sim

    dur = float(t[-1] - t[0])
    actividad = {k: float(v.mean() * 100.0) for k, v in act.items()}
    actividad["n_giros"] = len(giros)
    actividad["duracion_s"] = dur
    espacial = {"por_zupt_ESTIMACION": espaciales, "actividad_pct": actividad}

    principal = eventos.get("derecha") or list(eventos.values())[0]
    res = KinematicResults(
        session_id=os.path.basename(csv).split("_")[0], fs=fs, t=t, side="ambos",
        joints=articulaciones, events=principal, temporal=temporal,
        spatial=espacial, steady_state=util, turnarounds=giros,
        modes={s: streams[s]["modo"] for s in streams},
        orientations={s: streams[s]["q"] for s in streams},
        foot_node="pie_der",
        warnings=[
            "Todas las trazas de este archivo son MEDIDAS (inclinacion vs "
            "gravedad). No hay angulos de cadera/rodilla/tobillo: el montaje S "
            "no los observa, solo mide su suma (cadena_r / cadena_l).",
            "Los parametros se calculan solo sobre marcha recta: %.0f %% de la "
            "toma." % actividad["marcha"],
            "El largo de zancada por ZUPT es una ESTIMACION y en este proyecto "
            "no resulto confiable.",
        ])
    res.eventos_por_lado = eventos
    res.actividad = act
    res.streams = streams
    res.ciclos = ciclos
    return res


# ----------------------------------------------------------------- resumen
def _lado_tipicas(res, lado):
    cc = (getattr(res, "ciclos", {}) or {}).get(lado)
    return cc if cc and len(cc["T"]) else None


def resumen(res):
    """Los parametros que muestra la app: SOLO los que pasaron la validacion.

    Devuelve filas (seccion, parametro, valor, que significa). Lo que se dejo
    afuera a proposito (sigue en el .json para quien lo quiera revisar):
      - CV y DE del tiempo de zancada sobre TODAS las zancadas: en pasillos de
        3 m mezclan los pasitos de las vueltas y salen 20-40 %; no es
        variabilidad de la marcha. Se reporta la zancada de las tipicas.
      - largo de zancada y velocidad por ZUPT: no resultaron confiables.
      - pelvis, cabeza y braceo: la referencia de pelvis de OpenCap no es
        valida y el eje de la mano subestima la amplitud del braceo.
      - el desfase entre los dos detectores de eventos: es un control interno.
    """
    F = []
    act = res.spatial["actividad_pct"]
    F.append(("Toma", "duración", "%.1f s" % act["duracion_s"], ""))
    F.append(("Toma", "tiempo caminando", "%.0f %%" % act["marcha"],
              "los parámetros salen solo de la marcha en línea recta"))

    tp = res.temporal
    F.append(("Tiempos de la marcha", "cadencia",
              "%.0f pasos/min" % tp["cadence_steps_per_min"], "adulto: ~100-120"))
    T = {}
    for lado in ("derecha", "izquierda"):
        cc = _lado_tipicas(res, lado)
        if cc:
            T[lado] = float(cc["T"].mean())
            F.append(("Tiempos de la marcha", "zancada %s" % lado,
                      "%.2f ± %.2f s" % (cc["T"].mean(), cc["T"].std()),
                      "%d zancadas típicas de %d" % (len(cc["T"]), cc["n_total"])))
    for lado in ("derecha", "izquierda"):
        x = tp.get(lado, {})
        if x and np.isfinite(x.get("stance_pct", np.nan)):
            F.append(("Tiempos de la marcha", "apoyo / balanceo %s" % lado,
                      "%.0f / %.0f %%" % (x["stance_pct"], x["swing_pct"]),
                      "norma: ~60 / 40"))

    sim = tp.get("simetria", {})
    if len(T) == 2:
        F.append(("Simetría derecha / izquierda", "tiempo de zancada",
                  "%+.1f %%" % indice_simetria(T["derecha"], T["izquierda"]),
                  "0 % = simétrico; + = la derecha es mayor"))
    if np.isfinite(sim.get("si_porcentaje_de_apoyo_pct", np.nan)):
        F.append(("Simetría derecha / izquierda", "% de apoyo",
                  "%+.1f %%" % sim["si_porcentaje_de_apoyo_pct"], ""))

    for k, lab, txt in (("pie_der_sagital", "pie derecho", "inclinación del pie"),
                        ("pie_izq_sagital", "pie izquierdo", "inclinación del pie"),
                        ("cadena_r", "cadena pelvis→pie der", "≈ cadera − rodilla + tobillo"),
                        ("cadena_l", "cadena pelvis→pie izq", "≈ cadera − rodilla + tobillo")):
        if k in res.joints:
            p = res.joints[k]["params"]
            if np.isfinite(p.get("rom_cycle_deg", np.nan)):
                F.append(("Rango de movimiento (ciclo promedio)", lab,
                          "%.0f°" % p["rom_cycle_deg"], txt))
    return F


# ----------------------------------------------------------------- figuras
def figura(res, ruta=None):
    """La figura de la app: actividad, dos ciclos promedio y el resumen."""
    import matplotlib
    if ruta is not None:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, act = res.t, res.actividad
    fig = plt.figure(figsize=(14, 8.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.25], hspace=0.38, wspace=0.22)
    fig.suptitle("%s — parámetros de la marcha (6 sensores: cabeza, sacro, muñecas y pies)"
                 % res.session_id, fontsize=13)

    # A -- que hace el sujeto. Sin esto ningun otro numero se interpreta.
    a = fig.add_subplot(gs[0, 0])
    colores = {"marcha": "#2e7d32", "parado": "#9e9e9e",
               "giro": "#ef6c00", "otro": "#c62828"}
    for i, (k, c) in enumerate(colores.items()):
        a.fill_between(t, i, i + 0.85, where=act[k], color=c, step="mid")
        a.text(t[-1] * 1.01, i + 0.42, "%s  %.0f %%" % (k, act[k].mean() * 100),
               va="center", fontsize=9.5, color=c)
    a.set_yticks([]); a.set_xlim(t[0], t[-1] * 1.17)
    a.set_title("A · Qué hace el sujeto durante la toma", loc="left")
    a.set_xlabel("tiempo (s)"); a.grid(alpha=.3, axis="x")

    # B, C -- ciclo promedio sobre zancadas tipicas, anclado al contacto inicial
    LADOS = (("derecha", "tab:blue"), ("izquierda", "tab:orange"))
    for pos, titulo, nombres, centrar, ylab in (
            (gs[1, 0], "B · Pie: inclinación sagital durante el ciclo",
             {"derecha": "pie_der_sagital", "izquierda": "pie_izq_sagital"},
             False, "grados (0 = pie plano)"),
            (gs[1, 1], "C · Cadena pelvis→pie (≈ cadera − rodilla + tobillo)",
             {"derecha": "cadena_r", "izquierda": "cadena_l"},
             True, "grados (centrado)")):
        c = fig.add_subplot(pos)
        desp = []
        for lado, col in LADOS:
            cc = _lado_tipicas(res, lado)
            nm = nombres[lado]
            if not cc or nm not in res.joints:
                continue
            g, C, m, sd = ciclo_promedio(res.joints[nm]["flexion"], cc["cic"],
                                         centrar=centrar)
            c.plot(g, m, color=col, lw=2, label="%s (%d zancadas)" % (lado, len(C)))
            c.fill_between(g, m - sd, m + sd, color=col, alpha=.15)
            if cc.get("despegue_pct") is not None:
                desp.append(cc["despegue_pct"])
                c.axvline(cc["despegue_pct"], color=col, ls="--", lw=.9)
        if desp:
            dd = float(np.mean(desp))
            yl = c.get_ylim()
            c.text(dd / 2, yl[1], "apoyo", ha="center", va="top", color="gray", fontsize=9)
            c.text((100 + dd) / 2, yl[1], "balanceo", ha="center", va="top",
                   color="gray", fontsize=9)
        c.set_xlim(0, 100); c.grid(alpha=.3)
        c.set_title(titulo, loc="left", fontsize=11)
        c.set_xlabel("% del ciclo de marcha (0 = contacto del talón)")
        c.set_ylabel(ylab); c.legend(fontsize=8.5, loc="lower left")

    # D -- resumen
    d = fig.add_subplot(gs[0, 1]); d.axis("off")
    d.set_title("D · Resumen", loc="left")
    filas = [f for f in resumen(res) if f[0] != "Toma"]
    paso = min(0.075, 0.95 / (len(filas) + 4))
    y, sec_ant = 1.0, None
    for sec, par, val, _txt in filas:
        if sec != sec_ant:
            y -= 0.4 * paso
            d.text(0.0, y, sec.upper(), fontsize=8.5, weight="bold",
                   color="#444", transform=d.transAxes)
            y -= paso; sec_ant = sec
        d.text(0.03, y, par, fontsize=9.5, transform=d.transAxes)
        d.text(0.62, y, val, fontsize=9.5, transform=d.transAxes)
        y -= paso

    fig.subplots_adjust(left=0.06, right=0.97, top=0.9, bottom=0.08)
    if ruta is not None:
        fig.savefig(ruta, dpi=130)
        plt.close(fig)
        return ruta
    return fig


def figura_detallada(res, ruta=None):
    """La figura de analisis completa (incluye braceo, pelvis y controles
    internos). No es la de la app: `parametros_marcha.py --detalle`."""
    import matplotlib
    if ruta is not None:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, ev, act = res.t, res.eventos_por_lado, res.actividad
    fig, ax = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle("%s — parámetros medidos, montaje S" % res.session_id, fontsize=13)

    # A -- reparto de la actividad. Va primero a proposito: sin esto, ningun
    # otro numero de la figura se puede interpretar.
    a = ax[0, 0]
    colores = {"marcha": "#2e7d32", "parado": "#9e9e9e",
               "giro": "#ef6c00", "otro": "#c62828"}
    for i, (k, c) in enumerate(colores.items()):
        a.fill_between(t, i, i + 0.85, where=act[k], color=c, step="mid")
        a.text(t[-1] * 1.01, i + 0.42, "%s  %.0f %%" % (k, act[k].mean() * 100),
               va="center", fontsize=9, color=c)
    a.set_yticks([]); a.set_xlim(t[0], t[-1] * 1.18)
    a.set_title("A · Qué hace el sujeto (los parámetros salen solo de «marcha»)")
    a.set_xlabel("t (s)", loc="right"); a.grid(alpha=.3, axis="x")

    # B -- cadena sagital medida + contactos
    b = ax[0, 1]
    for suf, col, lab in (("r", "tab:blue", "derecha"), ("l", "tab:orange", "izquierda")):
        if "cadena_" + suf in res.joints:
            b.plot(t, res.joints["cadena_" + suf]["flexion"], col, lw=.8,
                   label="cadena " + lab)
    ppal = "derecha" if "derecha" in ev else list(ev)[0]
    ic = np.asarray(ev[ppal]["foot_strike"], int)
    ic = ic[ic < len(t)]
    b.plot(t[ic], np.zeros(len(ic)), "kv", ms=4, label="contacto inicial")
    b.fill_between(t, -200, 200, where=res.steady_state, color="k", alpha=.06,
                   label="marcha recta")
    if res.steady_state.any():
        x = np.concatenate([res.joints[k]["flexion"][res.steady_state]
                            for k in res.joints if k.startswith("cadena")])
        lo, hi = np.percentile(x, [1, 99]); pad = .25 * (hi - lo)
        b.set_ylim(lo - pad, hi + pad)
    b.set_title("B · Cadena sagital pelvis→pie — MEDIDA (= cadera − rodilla + tobillo)")
    b.set_xlabel("t (s)"); b.set_ylabel("grados"); b.legend(fontsize=8); b.grid(alpha=.3)

    # C -- ciclo promedio, una magnitud por panel, derecha vs izquierda.
    # Anclado al contacto inicial y sobre ciclos tipicos (ver
    # `ciclos_del_lado`). El pie va en grados absolutos (0 = pie plano); lo
    # demas centrado, porque su nivel depende de como quedo pegado el sensor.
    ax[1, 0].remove()
    sub = ax[1, 0].get_subplotspec().subgridspec(2, 2, hspace=0.6, wspace=0.28)
    cic = getattr(res, "ciclos", {}) or {}
    LADOS = (("derecha", "tab:blue", "der"), ("izquierda", "tab:orange", "izq"))
    paneles = [("pie sagital", {"derecha": "pie_der_sagital",
                                "izquierda": "pie_izq_sagital"},
                False, "grados  (0 = pie plano)"),
               ("cadena pelvis→pie", {"derecha": "cadena_r", "izquierda": "cadena_l"},
                True, "grados (centrado)"),
               ("braceo", {"derecha": "brazo_der_sagital",
                           "izquierda": "brazo_izq_sagital"},
                True, "grados (centrado)"),
               ("pelvis (ciclo del pie der)", None, True, "grados (centrado)")]
    for k, (titulo, nombres, centrar, ylab) in enumerate(paneles):
        c = fig.add_subplot(sub[k // 2, k % 2])
        despegues = []
        if nombres is None:
            series = [("pelvis_sagital", "tab:green", "sagital", "derecha"),
                      ("pelvis_frontal", "tab:olive", "frontal", "derecha")]
        else:
            series = [(nombres[lado], col, lab, lado) for lado, col, lab in LADOS]
        for nombre, col, lab, lado in series:
            cc = cic.get(lado)
            if nombre not in res.joints or not cc:
                continue
            g, C, m, sd = ciclo_promedio(res.joints[nombre]["flexion"], cc["cic"],
                                         centrar=centrar)
            c.plot(g, m, col, lw=1.8, label="%s (n=%d)" % (lab, len(C)))
            c.fill_between(g, m - sd, m + sd, color=col, alpha=.15)
            if cc.get("despegue_pct") is not None:
                despegues.append((cc["despegue_pct"], col))
        for pct, col in despegues:
            c.axvline(pct, color=col, ls="--", lw=.9, alpha=.8)
        if despegues:
            pct = np.mean([d for d, _ in despegues])
            yl = c.get_ylim()
            c.text(pct / 2, yl[1], "apoyo", ha="center", va="top", fontsize=7.5, color="gray")
            c.text((100 + pct) / 2, yl[1], "balanceo", ha="center", va="top", fontsize=7.5, color="gray")
        c.set_xlim(0, 100); c.set_title(titulo, fontsize=9.5)
        c.set_ylabel(ylab, fontsize=7.5); c.tick_params(labelsize=7.5)
        c.grid(alpha=.3); c.legend(fontsize=7, loc="lower left")
        if k // 2 == 1:
            c.set_xlabel("% del ciclo (0 = contacto inicial)", fontsize=8)
    sub_axes = [a for a in fig.axes if a.get_subplotspec() is not None
                and a.get_subplotspec().get_gridspec() is sub]

    # D -- tabla
    d = ax[1, 1]; d.axis("off")
    tp = res.temporal
    filas = [("MEDIDO", ""), ("cadencia", "%.1f pasos/min" % tp["cadence_steps_per_min"])]
    for lado in ("derecha", "izquierda"):
        x = tp.get(lado, {})
        if x and np.isfinite(x.get("stride_time_mean_s", np.nan)):
            filas.append(("zancada %s" % lado, "%.2f ± %.2f s  (CV %.1f %%)"
                          % (x["stride_time_mean_s"], x["stride_time_std_s"],
                             x["stride_time_cv_pct"])))
            filas.append(("apoyo/balanceo %s" % lado,
                          "%.0f / %.0f %%  ·  %d zancadas"
                          % (x["stance_pct"], x["swing_pct"], x["n_steady_strides"])))
    sim = tp.get("simetria", {})
    if sim:
        filas.append(("", ""))
        filas.append(("SIMETRÍA I/D", "(0 % = simétrico)"))
        for k, lab in (("si_tiempo_de_zancada_pct", "tiempo de zancada"),
                       ("si_porcentaje_de_apoyo_pct", "% de apoyo"),
                       ("si_rom_braceo_pct", "ROM de braceo"),
                       ("si_rom_cadena_sagital_pct", "ROM cadena sagital")):
            if np.isfinite(sim.get(k, np.nan)):
                filas.append(("  " + lab, "%+.1f %%" % sim[k]))
    filas.append(("", ""))
    filas.append(("CICLOS DEL PANEL C", "(zancadas típicas)"))
    for lado in ("derecha", "izquierda"):
        cc = (getattr(res, "ciclos", {}) or {}).get(lado)
        if cc and len(cc["T"]):
            T = cc["T"]
            filas.append(("  " + lado, "%d de %d · %.2f ± %.2f s (CV %.0f %%)"
                          % (len(T), cc["n_total"], T.mean(), T.std(),
                             100 * T.std() / T.mean())))
    filas.append(("", ""))
    filas.append(("ROM MEDIDOS (ciclo medio)", ""))
    for k, lab in (("pie_der_sagital", "pie der"), ("pie_izq_sagital", "pie izq"),
                   ("cadena_r", "cadena der"), ("cadena_l", "cadena izq"),
                   ("brazo_der_sagital", "braceo der"), ("brazo_izq_sagital", "braceo izq"),
                   ("pelvis_sagital", "pelvis sagital"),
                   ("pelvis_frontal", "pelvis frontal")):
        if k in res.joints:
            p = res.joints[k]["params"]
            if np.isfinite(p.get("rom_cycle_deg", np.nan)):
                filas.append(("  " + lab, "%.1f°  (por ciclo %.0f ± %.0f)"
                              % (p["rom_cycle_deg"], p["rom_por_ciclo_media_deg"],
                                 p["rom_por_ciclo_de_deg"])))
    ac = ev[ppal]["acuerdo"]
    filas.append(("", ""))
    filas.append(("eventos giro/ZUPT (%s)" % ppal, "%d / %d · desfase %+.0f ms"
                  % (ac["n_giroscopio"], ac["n_zupt"], ac["desfase_mediano_ms"])))
    y = 0.98
    for k, val in filas:
        d.text(0.02, y, k, fontsize=9.2, transform=d.transAxes,
               weight="bold" if (k and k == k.upper() and val in ("", "(0 % = simétrico)")) else "normal")
        d.text(0.55, y, val, fontsize=9.2, transform=d.transAxes)
        y -= 0.043
    d.set_title("D · Parámetros extraídos — todos medidos", loc="left")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    top = max(a.get_position().y1 for a in sub_axes)
    fig.text(0.02, top + 0.035,
             "C · Ciclo promedio ± 1 DE — zancadas típicas · línea = despegue",
             fontsize=11.5, va="bottom")
    if ruta is not None:
        fig.savefig(ruta, dpi=130)
        plt.close(fig)
        return ruta
    return fig


# -------------------------------------------------------------------- CLI
def imprimir(res):
    act = res.spatial["actividad_pct"]
    print("\n  fusión VQF 6D · %d segmentos · %.1f s" % (len(res.modes), res.duration_s))
    print("  actividad: marcha %.0f %% · parado %.0f %% · giro %.0f %% · otro %.0f %%"
          "   (%d giros)" % (act["marcha"], act["parado"], act["giro"],
                             act["otro"], act["n_giros"]))
    for lado, e in res.eventos_por_lado.items():
        a = e["acuerdo"]
        print("  eventos %-9s giroscopio %2d · ZUPT %2d · desfase %+.0f ms"
              % (lado, a["n_giroscopio"], a["n_zupt"], a["desfase_mediano_ms"]))
    for lado, cc in (getattr(res, "ciclos", {}) or {}).items():
        if cc and len(cc["T"]):
            T = cc["T"]
            print("  ciclos %-9s %2d típicos de %2d · zancada %.2f±%.2f s · CV %.0f %%"
                  " · despegue %s %%"
                  % (lado, len(T), cc["n_total"], T.mean(), T.std(),
                     100 * T.std() / T.mean(),
                     "%.0f" % cc["despegue_pct"] if cc["despegue_pct"] else "?"))
    print("\n  %-24s %8s %10s" % ("traza medida", "ROM ciclo", "n ciclos"))
    for nombre, d in sorted(res.joints.items()):
        p = d["params"]
        if np.isfinite(p.get("rom_cycle_deg", np.nan)):
            print("  %-24s %7.1f° %10d"
                  % (nombre, p["rom_cycle_deg"], p.get("n_ciclos_promediados", 0)))
    tp = res.temporal
    print("\n  cadencia %.1f pasos/min" % tp["cadence_steps_per_min"])
    for lado in ("derecha", "izquierda"):
        x = tp.get(lado)
        if x and np.isfinite(x.get("stride_time_mean_s", np.nan)):
            print("  %-10s zancada %.2f±%.2f s · CV %.1f %% · apoyo %.0f %% · %d zancadas"
                  % (lado, x["stride_time_mean_s"], x["stride_time_std_s"],
                     x["stride_time_cv_pct"], x["stance_pct"], x["n_steady_strides"]))
    sim = tp.get("simetria", {})
    if sim:
        print("  simetría I/D: zancada %+.1f %% · apoyo %+.1f %% · braceo %+.1f %%"
              % (sim.get("si_tiempo_de_zancada_pct", float("nan")),
                 sim.get("si_porcentaje_de_apoyo_pct", float("nan")),
                 sim.get("si_rom_braceo_pct", float("nan"))))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", help="tag (testnorma2l9112026) o ruta a un CSV de los sensores")
    p.add_argument("--salida", default=DIR_SALIDAS)
    p.add_argument("--figuras", action="store_true")
    p.add_argument("--detalle", action="store_true",
                   help="ademas, la figura de analisis completa (<toma>_figura_detalle.png)")
    a = p.parse_args()

    import reconstruir_toma as RT           # solo para localizar el CSV
    csv = RT.buscar_csv(a.toma)
    # El tag es lo que va antes de `_log_` (con la fecha), igual que en
    # `reconstruir_toma.py`, asi las salidas de una toma se llaman igual en
    # todas las carpetas.
    base = os.path.basename(csv)
    tag_largo = base.split("_log_")[0]
    tag = tag_largo if "_" in tag_largo and tag_largo[-8:].isdigit() else base.split("_")[0]
    print("toma: %s" % os.path.basename(csv))
    res = analizar(csv)
    imprimir(res)

    if not os.path.isdir(a.salida):
        os.makedirs(a.salida)
    print()
    for f in (res.save_summary_json(os.path.join(a.salida, tag + "_parametros.json")),
              res.save_timeseries_csv(os.path.join(a.salida, tag + "_series.csv")),
              res.save_events_csv(os.path.join(a.salida, tag + "_eventos.csv"))):
        print("  %s" % f)
    if a.figuras:
        print("  %s" % figura(res, os.path.join(a.salida, tag + "_figura.png")))
    if a.detalle:
        print("  %s" % figura_detallada(res, os.path.join(a.salida,
                                                         tag + "_figura_detalle.png")))
    print("\n  Para el .mot de OpenSim (reconstrucción, no medición):")
    print("    python3 reconstruir_toma.py %s" % tag)


if __name__ == "__main__":
    main()
