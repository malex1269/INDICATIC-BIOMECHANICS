"""
sincronizar.py -- alineacion temporal IMU (modo log) <-> OpenCap.

Objetivo 2 del paper. La senal comun es la MAGNITUD de la velocidad angular
del pie: |w| no depende de la convencion de ejes de ninguno de los dos
sistemas, asi que no hace falta resolver la alineacion de marcos para
sincronizar (eso viene despues).

Dos etapas, para evitar el aliasing con el periodo de zancada (~1.3 s):
  1. Grueso: correlacion de la ENVOLVENTE de actividad (|w| pasa-bajos a
     0.2 Hz). El patron de "camina / para / camina" a lo largo de la toma es
     unico, no periodico -> el pico es inequivoco aunque el desfase sea de
     decenas de segundos.
  2. Fino: correlacion de |w| filtrada a 6 Hz, buscando solo +-1 s alrededor
     del grueso -> resolucion de sub-muestra por interpolacion parabolica.

  3. Golpes de talon (desde la sesion 2026-09-11): antes de caminar el sujeto
     da 3 golpes con el talon derecho contra el piso, parado, dentro del campo
     de las camaras. Cada golpe es un evento seco que los dos sistemas ven en
     el mismo pie: se localiza en el IMU (pico de |a| con el otro pie
     plantado) y alrededor de cada uno se busca el desfase que mejor
     superpone |w| del IMU con |w| de OpenCap (+-0.3 s en torno al fino). La
     mediana de esos desfases es el arranque; su dispersion es la PRECISION
     de la sincronizacion, cosa que la correlacion sola no da. Si hay golpes
     al principio y al final, la pendiente del desfase con el tiempo es la
     deriva de reloj medida con eventos, no con correlacion. Si no hay
     golpes (sesiones anteriores) queda el resultado de la etapa 2.

Despues, correlacion por ventanas -> lag(t) = a*t + b: `b` es el desfase de
arranque y `a` la diferencia de ritmo de reloj entre la Raspberry y el iPhone
(objetivo 4, exactitud de alineacion IMU-camara).

Uso:  python sincronizar.py                       # cruza cada toma contra los trials de su sesion
      python sincronizar.py <csv> <trc>             # desfase y r de una toma
      python sincronizar.py <csv> <trc> <figura.png>   # y la figura de los golpes de talon
"""
import os
import sys
import numpy as np
from scipy import signal, stats

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sparse6 as s6
import opencap_io as oc
from mapa_sensores import mapa_de

FS = 60.0                      # grilla comun = la de OpenCap


def _pasabajos(x, fc, fs=FS, orden=2):
    b, a = signal.butter(orden, fc / (fs / 2), "low")
    return signal.filtfilt(b, a, x)


def senal_imu(csv, lado="pie_der"):
    """|w| del pie desde el CSV de modo log, remuestreada a 60 Hz."""
    R, t = s6.cargar_toma(csv)
    inv = {v: k for k, v in mapa_de(csv).items()}
    mac = inv[lado]
    if mac not in R:
        return None, None
    _, _, gyro = R[mac]
    w = np.linalg.norm(gyro, axis=1)                  # deg/s a 100 Hz
    tn = np.arange(0.0, t[-1], 1.0 / FS)
    return tn, np.interp(tn, t, w)


def senal_opencap(trc, lado="calcn_r"):
    """|w| del mismo pie desde los marcadores de OpenCap, en deg/s a 60 Hz."""
    t, M = oc.leer_trc(trc)
    R = oc.marcos_segmento(M)
    w = np.degrees(np.linalg.norm(oc.velocidad_angular(R[lado]), axis=1))
    return t, w


def _xcorr(a, b, lag_min, lag_max):
    """lag (en muestras) tal que b(t) ~ a(t - lag), y el r del pico."""
    a = (a - a.mean()) / (a.std() + 1e-12)
    b = (b - b.mean()) / (b.std() + 1e-12)
    c = signal.correlate(b, a, mode="full") / min(len(a), len(b))
    lg = signal.correlation_lags(len(b), len(a), mode="full")
    m = (lg >= lag_min) & (lg <= lag_max)
    c, lg = c[m], lg[m]
    i = int(np.argmax(c))
    d = 0.0
    if 0 < i < len(c) - 1:
        den = c[i - 1] - 2 * c[i] + c[i + 1]
        if abs(den) > 1e-12:
            d = 0.5 * (c[i - 1] - c[i + 1]) / den
    return lg[i] + d, c[i]


def golpes_imu(csv, lado="pie_der", otro="pie_izq", fs=s6.FS,
               umbral_g=0.8, quieto_dps=30.0, ventana_s=0.75, sep_s=0.4):
    """Instantes (s, tiempo del IMU) de los golpes de talon de `lado`.

    Un golpe es un pico de |a| - 1 de mas de `umbral_g` (el talon contra el
    piso da 1.3-2.5 g; caminando, 0.3-0.8) con el OTRO pie plantado: |w| del
    otro pie por debajo de `quieto_dps` en promedio en +-`ventana_s`. Eso
    descarta los contactos de la marcha normal, donde el otro pie siempre
    esta balanceandose a menos de 0.75 s. Tambien pasan la levantada del pie
    justo antes de cada golpe y algun pisoton con el otro pie quieto (en
    marcha lenta, caminata03 del 2026-09-10): son eventos secos igual de
    utiles para alinear, asi que no se filtran. Devuelve un array (vacio si
    no hay ninguno).
    """
    R, t = s6.cargar_toma(csv)
    inv = {v: k for k, v in mapa_de(csv, aviso=False).items()}
    if inv.get(lado) not in R or inv.get(otro) not in R:
        return np.array([])
    _, acc, _g = R[inv[lado]]
    _, _a, gyro_o = R[inv[otro]]
    a = np.abs(np.linalg.norm(acc, axis=1) - 1.0)
    w_o = np.linalg.norm(gyro_o, axis=1)
    picos, _ = signal.find_peaks(a, height=umbral_g, distance=int(sep_s * fs))
    n = int(ventana_s * fs)
    out = []
    for i in picos:
        if w_o[max(0, i - n):i + n].mean() < quieto_dps:
            out.append(t[i])
    return np.array(out)


def _ajustar_en_golpes(ti, fi, to, fo, golpes, arranque, semi_s=0.5,
                       busca_s=0.3, fs=FS):
    """Para cada golpe, el arranque que mejor superpone |w| IMU y OpenCap.

    Alrededor del golpe (+-`semi_s`) se prueba cada arranque en
    `arranque` +- `busca_s` y se queda con el de mayor correlacion, refinado
    por interpolacion parabolica. `busca_s` es menor que la separacion entre
    golpes (~0.75 s), asi que no puede saltar al golpe vecino.
    Devuelve listas (arranque_k, r_k) del mismo largo que `golpes`.
    """
    cand = arranque + np.arange(-busca_s, busca_s + 0.5 / fs, 1.0 / fs)
    arr, rs = [], []
    for g in golpes:
        m = (ti >= g - semi_s) & (ti <= g + semi_s)
        if m.sum() < int(semi_s * fs):
            arr.append(np.nan); rs.append(np.nan)
            continue
        x = fi[m] - fi[m].mean()
        cc = []
        for off in cand:
            y = np.interp(ti[m] - off, to, fo, left=np.nan, right=np.nan)
            if np.isnan(y).any() or y.std() < 1e-9:
                cc.append(-1.0)
                continue
            y = y - y.mean()
            cc.append(float(np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-12)))
        cc = np.array(cc)
        i = int(np.argmax(cc))
        d = 0.0
        if 0 < i < len(cc) - 1:
            den = cc[i - 1] - 2 * cc[i] + cc[i + 1]
            if abs(den) > 1e-12:
                d = 0.5 * (cc[i - 1] - cc[i + 1]) / den
        arr.append(cand[i] + d / fs); rs.append(cc[i])
    return np.array(arr), np.array(rs)


def sincronizar(csv, trc, lado_imu="pie_der", lado_oc="calcn_r", verbose=True,
                golpes=True):
    ti, wi = senal_imu(csv, lado_imu)
    if ti is None:
        return None
    to, wo = senal_opencap(trc, lado_oc)

    # etapa 1: envolvente de actividad (que no es periodica)
    ei = _pasabajos(wi, 0.2)
    eo = _pasabajos(wo, 0.2)
    lag_g, r_g = _xcorr(ei, eo, -len(eo), len(ei))

    # etapa 2: refinamiento con la forma de onda
    fi = _pasabajos(wi, 6.0)
    fo = _pasabajos(wo, 6.0)
    L = int(1.0 * FS)
    lag_f, r_f = _xcorr(fi, fo, int(lag_g) - L, int(lag_g) + L)

    # _xcorr devuelve lag con la convencion oc[n] ~ imu[n - lag], asi que
    # el instante t de OpenCap cae en el instante (t - lag) del IMU:
    # arranque_s > 0 = OpenCap arranco DESPUES de los IMU (lo que pide el
    # protocolo: arrancar los IMU primero y pararlos despues).
    res = dict(lag_grueso_s=lag_g / FS, r_grueso=r_g,
               lag_s=lag_f / FS, arranque_s=-lag_f / FS, r=r_f,
               ti=ti, wi=wi, to=to, wo=wo)

    # deriva: lag por ventanas sobre el solape
    off = int(round(-lag_f))                     # indice IMU del t=0 de OpenCap
    n = min(len(fo), len(fi) - off)
    if n > int(20 * FS) and off >= 0:
        seg_i = fi[off:off + n]
        seg_o = fo[:n]
        win, paso, LL = int(10 * FS), int(5 * FS), int(0.5 * FS)
        ts, lags, rs = [], [], []
        for k in range(0, n - win, paso):
            l, r = _xcorr(seg_i[k:k + win], seg_o[k:k + win], -LL, LL)
            ts.append((k + win / 2) / FS); lags.append(l / FS); rs.append(r)
        ts, lags, rs = np.array(ts), np.array(lags), np.array(rs)
        ok = rs > 0.3
        if ok.sum() >= 4:
            fit = stats.theilslopes(lags[ok] * 1000, ts[ok])   # ms vs s
            res.update(deriva_ppm=fit[0] * 1e3, b_ms=fit[1],
                       n_ventanas=int(ok.sum()), r_ventanas=float(rs[ok].mean()),
                       lag_ventanas=(ts[ok], lags[ok]))

    # etapa 3: golpes de talon. Se prueba primero alrededor del fino y, si
    # ahi no aparecen (el fino puede haberse enganchado una zancada
    # corrida), alrededor del grueso con una busqueda mas ancha.
    res["metodo"] = "correlacion"
    g = golpes_imu(csv, lado_imu) if golpes else np.array([])
    if len(g) >= 2:
        fi8 = _pasabajos(wi, 8.0)
        fo8 = _pasabajos(wo, 8.0)
        for base, busca in ((res["arranque_s"], 0.3), (-lag_g / FS, 0.8)):
            arr, rk = _ajustar_en_golpes(ti, fi8, to, fo8, g, base, busca_s=busca)
            ok = np.isfinite(rk) & (rk > 0.6)
            if ok.sum() >= 2 and np.std(arr[ok]) < 0.1:
                break
        if ok.sum() >= 2 and np.std(arr[ok]) < 0.1:
            res.update(metodo="golpes",
                       arranque_correlacion_s=res["arranque_s"],
                       arranque_s=float(np.median(arr[ok])),
                       golpes=dict(t_imu=g, arranque=arr, r=rk, ok=ok),
                       n_golpes=int(ok.sum()),
                       precision_ms=float(1000 * np.std(arr[ok])),
                       r_golpes=float(rk[ok].mean()))
            # deriva entre el primer y el ultimo golpe, solo con eventos
            # bien alineados y separados: dos outliers en una toma con un
            # solo bloque de golpes inventaban 2000 ppm.
            bien = ok & (rk > 0.7)
            if bien.sum() >= 4 and g[bien].max() - g[bien].min() > 15.0:
                fit = stats.theilslopes(arr[bien] * 1000, g[bien])
                lapso = float(g[bien].max() - g[bien].min())
                res["deriva_golpes_ppm"] = float(fit[0] * 1e3)
                res["deriva_golpes_ms"] = float(fit[0] * lapso)   # sobre el lapso
                res["deriva_golpes_lapso_s"] = lapso
                # Medible solo si en ese lapso acumula mas que la precision
                # de un golpe: dentro de un mismo bloque el arranque ya se
                # mueve 30-60 ms de un golpe al siguiente.
                res["deriva_golpes_medible"] = bool(
                    abs(fit[0] * lapso) > 2 * res["precision_ms"])
        else:
            res.update(golpes=dict(t_imu=g, arranque=arr, r=rk, ok=ok))
    if verbose:
        if res["metodo"] == "golpes":
            print("  sincronizacion por %d golpes de talon: arranque %+.3f s, "
                  "precision %.0f ms, r=%.2f (correlacion daba %+.3f s, r=%.2f)"
                  % (res["n_golpes"], res["arranque_s"], res["precision_ms"],
                     res["r_golpes"], res["arranque_correlacion_s"], res["r"]))
        else:
            print("  sincronizacion por correlacion: arranque %+.3f s, r=%.2f"
                  % (res["arranque_s"], res["r"]))
    return res


def figura_sincronizacion(res, ruta, titulo=""):
    """Figura de la sincronizacion: |w| del pie en los dos sistemas ya
    alineados (toda la toma), zoom en cada bloque de golpes, y el arranque
    que da cada golpe (la dispersion es la precision; la pendiente, la
    deriva de reloj)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ti, wi, to, wo = res["ti"], res["wi"], res["to"], res["wo"]
    off = res["arranque_s"]
    g = res.get("golpes")
    # bloques de golpes: eventos a menos de 3 s entre si
    bloques = []
    if g is not None and len(g["t_imu"]):
        for tg in g["t_imu"]:
            if bloques and tg - bloques[-1][-1] < 3.0:
                bloques[-1].append(tg)
            else:
                bloques.append([tg])
    # se muestran los 3 bloques con mas golpes usados
    if g is not None and bloques:
        ok_t = set(np.round(g["t_imu"][g["ok"]], 3))
        bloques.sort(key=lambda bl: -sum(round(x, 3) in ok_t for x in bl))
        bloques = sorted(bloques[:3], key=lambda bl: bl[0])
    n_zoom = min(len(bloques), 3)
    fig = plt.figure(figsize=(15, 4 + 3.2 * (1 if n_zoom else 0) + 3))
    gs = fig.add_gridspec(2 + (1 if n_zoom else 0), max(n_zoom, 1))

    a = fig.add_subplot(gs[0, :])
    a.plot(ti, wi, "b", lw=.7, label="IMU |ω| pie derecho")
    a.plot(to + off, wo, "r", lw=.7, alpha=.8,
           label="OpenCap |ω| calcáneo derecho, desplazado %+.3f s" % off)
    if g is not None:
        for tg in g["t_imu"]:
            a.axvline(tg, color="gray", lw=.6, ls=":")
    a.set_xlabel("t IMU [s]"); a.set_ylabel("deg/s"); a.grid(alpha=.3)
    a.legend(loc="upper right", fontsize=8)
    a.set_title(titulo or "sincronización IMU ↔ OpenCap", fontsize=11)

    if n_zoom:
        for k, bl in enumerate(bloques[:n_zoom]):
            b = fig.add_subplot(gs[1, k])
            t0, t1 = bl[0] - 0.8, bl[-1] + 0.8
            m = (ti >= t0) & (ti <= t1)
            b.plot(ti[m], wi[m], "b", lw=1)
            mo = (to + off >= t0) & (to + off <= t1)
            b.plot(to[mo] + off, wo[mo], "r", lw=1)
            if "arranque_correlacion_s" in res:
                oc = res["arranque_correlacion_s"]
                mo2 = (to + oc >= t0) & (to + oc <= t1)
                b.plot(to[mo2] + oc, wo[mo2], "r", lw=.7, ls="--", alpha=.5,
                       label="con el arranque por correlación (%+.3f s)" % oc)
            for tg, ok in zip(g["t_imu"], g["ok"]):
                if t0 <= tg <= t1:
                    b.axvline(tg, color="k" if ok else "gray", lw=.8, ls=":")
            b.set_xlim(t0, t1); b.grid(alpha=.3)
            b.set_title("golpes %.0f-%.0f s" % (bl[0], bl[-1]), fontsize=9)
            if k == 0:
                b.set_ylabel("deg/s")
                if "arranque_correlacion_s" in res:
                    b.legend(fontsize=7, loc="upper right")

    c = fig.add_subplot(gs[-1, :])
    if g is not None:
        ok = g["ok"]
        c.plot(g["t_imu"][ok], g["arranque"][ok], "ko", ms=5,
               label="arranque por golpe (usados)")
        c.plot(g["t_imu"][~ok], g["arranque"][~ok], "o", mfc="none", mec="gray",
               ms=5, label="descartados (r < 0.6)")
        c.axhline(off, color="k", lw=1,
                  label="mediana %+.3f s · precisión %.0f ms · %d golpes"
                        % (off, res.get("precision_ms", float("nan")),
                           res.get("n_golpes", 0)))
        if "arranque_correlacion_s" in res:
            c.axhline(res["arranque_correlacion_s"], color="r", lw=1, ls="--",
                      label="correlación %+.3f s (r=%.2f)"
                            % (res["arranque_correlacion_s"], res["r"]))
        if "deriva_golpes_ppm" in res:
            c.text(0.01, 0.05, "deriva de reloj entre bloques: %+.0f ppm = %+.0f ms en %.0f s%s"
                   % (res["deriva_golpes_ppm"], res["deriva_golpes_ms"],
                      res["deriva_golpes_lapso_s"],
                      "" if res["deriva_golpes_medible"]
                      else " → por debajo de la precisión: no medible"),
                   transform=c.transAxes, fontsize=9)
        lo = min(np.nanmin(g["arranque"][ok]), res.get("arranque_correlacion_s", off))
        hi = max(np.nanmax(g["arranque"][ok]), res.get("arranque_correlacion_s", off))
        c.set_ylim(lo - 0.1, hi + 0.1)
    else:
        c.axhline(off, color="r", lw=1, label="correlación %+.3f s (r=%.2f)" % (off, res["r"]))
        c.text(0.01, 0.5, "sin golpes de talón detectados", transform=c.transAxes)
    c.set_xlim(ti[0], ti[-1]); c.set_xlabel("t IMU del golpe [s]"); c.set_ylabel("arranque [s]")
    c.grid(alpha=.3); c.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(ruta, dpi=110)
    plt.close(fig)
    return ruta


def cruzar_todo():
    """Correlaciona cada toma de los sensores contra cada trial de OpenCap de
    su sesion (las registradas en reconstruir_toma.SESIONES).

    Si el emparejamiento de reconstruir_toma.PARES es correcto, cada toma
    tiene que dar su mejor r con su propio trial. Es la verificacion que evita
    comparar dos pasadas distintas.
    """
    import reconstruir_toma as R
    mejores = {}
    for clave, carpeta, _trials in R.SESIONES:
        dir_trc = os.path.join(carpeta, "MarkerData")
        trials = [f[:-4] for f in sorted(os.listdir(dir_trc))
                  if f.endswith(".trc") and f != "neutral.trc"]
        imus = {}
        for d in R.DIRS_IMU:
            for f in sorted(os.listdir(d)):
                if (f.endswith(".csv") and "_log_" in f and clave in f.split("_log_")[0]
                        and os.path.getsize(os.path.join(d, f)) > 100000):
                    imus[f.split("_log_")[0]] = os.path.join(d, f)
        print("Sesion %s: correlacion de |w| del pie derecho (r del pico fino)" % clave)
        print("%-22s" % "" + "".join("%10s" % t for t in trials))
        for nm, csv in imus.items():
            fila, best = [], (None, -1)
            for tr in trials:
                try:
                    r = sincronizar(csv, os.path.join(dir_trc, tr + ".trc"),
                                    verbose=False)
                    v = r["r"] if r else np.nan
                except Exception:
                    v = np.nan
                fila.append(v)
                if np.isfinite(v) and v > best[1]:
                    best = (tr, v)
            mejores[nm] = best
            print("%-22s" % nm + "".join("%10.3f" % v for v in fila)
                  + "   -> %s (%.3f)" % best)
        print()
    for nm, (tr, v) in mejores.items():
        esperado = R.PARES.get(nm)
        marca = "OK " if tr == esperado else "!! "
        print("  %s%-22s mejor=%-10s registrado=%-10s r=%.3f"
              % (marca, nm, tr, esperado, v))
    return mejores


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        r = sincronizar(sys.argv[1], sys.argv[2])
        print(r["lag_s"], r["r"])
        if len(sys.argv) == 4:                    # <csv> <trc> <figura.png>
            print(figura_sincronizacion(r, sys.argv[3]))
    else:
        cruzar_todo()
