"""
app_marcha.py -- app de escritorio con los parametros de marcha de una toma.

    python app_marcha.py                        # abre y elegis la toma
    python app_marcha.py testnorma2l9112026     # abre esa toma directo

Montaje de 6 sensores: cabeza, sacro, ambas munecas y ambos pies. La app
muestra SOLO lo que este montaje mide y que paso la validacion contra
OpenCap y contra la fisiologia (ver docs/3_analisis_y_validacion.md):

    Graficas     que hace el sujeto, el ciclo promedio del pie y de la
                 cadena pelvis->pie, y el resumen
    Parametros   la misma tabla, con que significa cada numero

Lo que quedo afuera a proposito (CV de zancada sobre todas las zancadas,
largo de zancada por ZUPT, pelvis, cabeza, braceo, controles internos) esta
explicado en `parametros_marcha.resumen` y sigue en el .json que escribe
`parametros_marcha.py`.

El boton "Generar .mot" escribe cadera, rodilla y tobillo RECONSTRUIDOS para
animar el modelo en OpenSim. No son una medicion: este montaje no tiene
sensores en muslo ni en pierna.
"""
import argparse
import os
import queue
import sys
import threading
import traceback

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from rutas import DIR_SALIDAS

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)

import parametros_marcha as PM
import reconstruir_toma as RT


class AppMarcha(tk.Tk):
    def __init__(self, toma=None):
        super().__init__()
        self.title("Parámetros de marcha — INDICATIC")
        self.geometry("1300x860")
        self.minsize(900, 620)
        self._res = None
        self._canvas = None
        self._csv = None
        self._q = queue.Queue()
        self._ui()
        if toma:
            self.after(200, lambda: self._abrir(RT.buscar_csv(toma)))

    # ------------------------------------------------------------------ UI
    def _ui(self):
        top = tk.Frame(self, pady=6, padx=10)
        top.pack(side=tk.TOP, fill=tk.X)
        tk.Button(top, text="Abrir CSV…", width=12,
                  command=self._elegir).pack(side=tk.LEFT, padx=(0, 6))
        self._combo = ttk.Combobox(top, width=22, state="readonly",
                                   values=self._tomas())
        self._combo.pack(side=tk.LEFT, padx=(0, 6))
        self._combo.bind("<<ComboboxSelected>>",
                         lambda e: self._abrir(RT.buscar_csv(self._combo.get())))
        self._ruta = tk.StringVar(value="(ninguna toma cargada)")
        tk.Label(top, textvariable=self._ruta, anchor="w",
                 fg="#333").pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self._btn_mot = tk.Button(top, text="Generar .mot para OpenSim", width=24,
                                  command=self._mot, state=tk.DISABLED)
        self._btn_mot.pack(side=tk.RIGHT)

        ttk.Separator(self, orient="horizontal").pack(fill=tk.X)
        self._nb = ttk.Notebook(self)
        self._nb.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self._tab_fig = tk.Frame(self._nb, bg="#f8f8f8")
        self._tab_par = tk.Frame(self._nb)
        self._nb.add(self._tab_fig, text="  Gráficas  ")
        self._nb.add(self._tab_par, text="  Parámetros  ")
        tk.Label(self._tab_fig, bg="#f8f8f8", fg="#888", justify="center",
                 font=("Segoe UI", 12),
                 text="Elegí una toma en el desplegable o abrí un CSV de los sensores."
                 ).pack(expand=True)
        self._tabla(self._tab_par)

        ttk.Separator(self, orient="horizontal").pack(fill=tk.X)
        self._estado = tk.StringVar(value="Listo")
        tk.Label(self, textvariable=self._estado, anchor="w", fg="#555",
                 padx=10, pady=3).pack(side=tk.BOTTOM, fill=tk.X)

    def _tabla(self, padre):
        cols = ("parametro", "valor", "significa")
        self._tv = ttk.Treeview(padre, columns=cols, show="tree headings")
        self._tv.heading("#0", text="Sección")
        self._tv.column("#0", width=280, anchor="w")
        for c, txt, w in (("parametro", "Parámetro", 260), ("valor", "Valor", 160),
                          ("significa", "Qué significa", 380)):
            self._tv.heading(c, text=txt)
            self._tv.column(c, width=w, anchor="w")
        sb = ttk.Scrollbar(padre, orient="vertical", command=self._tv.yview)
        self._tv.configure(yscrollcommand=sb.set)
        self._tv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _tomas(self):
        """Las tomas que hay en las carpetas de datos (lo que va hasta `_log_`
        en el nombre del CSV que escribe acquire.py)."""
        vistas = []
        for carpeta in RT.DIRS_IMU:
            if not os.path.isdir(carpeta):
                continue
            for f in sorted(os.listdir(carpeta)):
                if (f.endswith(".csv") and "_log_" in f and "run_log" not in f
                        and os.path.getsize(os.path.join(carpeta, f)) > 100000):
                    tag = f.split("_log_")[0]
                    if tag not in vistas:
                        vistas.append(tag)
        return vistas

    # --------------------------------------------------------------- carga
    def _elegir(self):
        ini = next((d for d in RT.DIRS_IMU if os.path.isdir(d)), AQUI)
        p = filedialog.askopenfilename(
            title="Elegí un CSV de los sensores", initialdir=ini,
            filetypes=[("CSV de los sensores", "*.csv"), ("Todos", "*.*")])
        if p:
            self._abrir(p)

    def _abrir(self, csv):
        self._csv = csv
        self._ruta.set(os.path.basename(csv))
        tag = os.path.basename(csv).split("_log_")[0]
        if tag in self._combo["values"]:
            self._combo.set(tag)
        self._estado.set("Calculando…  (orientación, eventos de la marcha, parámetros)")
        self._btn_mot.config(state=tk.DISABLED)
        threading.Thread(target=self._trabajar, args=(csv,), daemon=True).start()
        self.after(120, self._revisar)

    def _trabajar(self, csv):
        try:
            self._q.put(("ok", PM.analizar(csv, verbose=False)))
        except (ValueError, SystemExit) as e:
            self._q.put(("error", str(e)))
        except Exception:
            self._q.put(("error", traceback.format_exc()))

    def _revisar(self):
        try:
            tipo, carga = self._q.get_nowait()
        except queue.Empty:
            self.after(120, self._revisar)
            return
        if tipo == "error":
            self._estado.set("Falló el análisis")
            messagebox.showerror("Error", carga)
            return
        self._res = carga
        self._pintar()

    # ------------------------------------------------------------- pintado
    def _pintar(self):
        res = self._res
        for w in self._tab_fig.winfo_children():
            w.destroy()
        fig = PM.figura(res, ruta=None)
        self._canvas = FigureCanvasTkAgg(fig, master=self._tab_fig)
        self._canvas.draw()
        NavigationToolbar2Tk(self._canvas, self._tab_fig).update()
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._tv.delete(*self._tv.get_children())
        secciones = {}
        for sec, par, val, txt in PM.resumen(res):
            if sec not in secciones:
                secciones[sec] = self._tv.insert("", "end", text=sec, open=True)
            self._tv.insert(secciones[sec], "end", text="", values=(par, val, txt))

        act = res.spatial["actividad_pct"]
        aviso = ""
        if act["marcha"] < 25:
            aviso = "   ⚠ solo %.0f %% de la toma es marcha: pocas zancadas" % act["marcha"]
        self._estado.set("%s · %.1f s · marcha %.0f %% · cadencia %.0f pasos/min%s"
                         % (res.session_id, res.duration_s, act["marcha"],
                            res.temporal["cadence_steps_per_min"], aviso))
        self._btn_mot.config(state=tk.NORMAL)

    # ----------------------------------------------------------------- mot
    def _mot(self):
        if not self._csv:
            return
        if not messagebox.askokcancel(
                "Generar .mot para OpenSim",
                "El .mot lleva cadera, rodilla y tobillo RECONSTRUIDOS a partir de "
                "lo que miden los sensores y de la marcha del sujeto registrada "
                "con OpenCap.\n\nSirve para animar el modelo en OpenSim; no es "
                "una medición.\n\nPuede tardar un minuto."):
            return
        self._estado.set("Generando el .mot…")
        threading.Thread(target=self._trabajar_mot, daemon=True).start()

    def _trabajar_mot(self):
        try:
            import mot_io
            base = os.path.basename(self._csv)
            tag_largo = base.split("_log_")[0]
            tag = tag_largo if tag_largo in RT.PARES else base.split("_")[0]
            # Para animar se usa la variedad de la sesion completa, igual que
            # `pipeline.py` (`reconstruir_toma.py --sin-excluir`). Excluir el
            # trial pareado es para validar, y en la sesion del 11/09 deja la
            # variedad sin ciclos (TEST1_1 solo no alcanza).
            v = RT.obtener_variedad(None, excluir=None, verbose=False, tag=tag)
            t, datos = RT.reconstruir(self._csv, v, verbose=False)
            osim = RT.osim_de(tag)      # el .osim escalado de ESA sesion
            lim = mot_io.limites_osim(osim) if os.path.isfile(osim) else None
            os.makedirs(DIR_SALIDAS, exist_ok=True)
            ruta, _f, _r = mot_io.escribir_mot(
                os.path.join(DIR_SALIDAS, tag + "_reconstruido.mot"), t, datos,
                limites=lim)
            self.after(0, lambda: (self._estado.set("Escrito: %s" % ruta),
                                   messagebox.showinfo(
                                       "Listo",
                                       "%s\n\nEn OpenSim:\n"
                                       "1. File > Open Model > %s\n"
                                       "2. File > Load Motion > este archivo"
                                       % (ruta, osim))))
        except Exception:
            err = traceback.format_exc()
            self.after(0, lambda: (self._estado.set("Falló el .mot"),
                                   messagebox.showerror("Error", err)))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", nargs="?", default=None,
                   help="tag (testnorma2l9112026) o ruta a un CSV; opcional")
    a = p.parse_args()
    AppMarcha(a.toma).mainloop()


if __name__ == "__main__":
    main()
