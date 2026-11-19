"""Simulierte RGBD-Kamera: Farb- und Tiefenbild aus Sicht einer Kamera im MuJoCo-Modell (mujoco.Renderer).

Montage wie beim Lidar aus roboter_beschreibung/config/roboter.yaml (Eintrag kamera): Elternglied, Pose in ROS-Konvention
(x nach vorn, y nach links, z nach oben), Bildgroesse, vertikaler Oeffnungswinkel. fuege_kamera_hinzu() haengt daraus beim
Aufbau des Modells (szene.py, vor compile) ein <camera>-Element an das Elternglied; die Unitree-Datei bleibt unveraendert.
MuJoCo bewegt die Kamera dann mit dem Glied mit.

Anders als Sensoren stehen Bilder NICHT in data.sensordata: sie entstehen erst beim Rendern (OpenGL).
  update_scene(data, camera=...)  Szene aus dem aktuellen Zustand aus Sicht der Kamera aufbauen
  render()                        zeichnen und als numpy-Array auslesen (Farbe uint8 bzw. Tiefe float32 in m)

Rendern im eigenen Thread (KameraThread) mit eigener Kopie des Zustands: gemessen 11.11.2026 (WSL2 ohne GPU, Software-
Rendering llvmpipe, 320x240, Testwelt) 50 ms je Bild (Farbe + Tiefe) mit waagerechtem Blick, ~115 ms mit 0.3 rad Neigung
(viel texturierter Boden im Bild). Im Simulationsthread haette jedes Bild die Physik (Takt 2 ms) so lange angehalten.
Rendern gibt den Python-GIL frei (gemessen: ein 2-ms-Takt in einem anderen Thread hoechstens 2 ms verspaetet). Das
ANLEGEN der Kamera dagegen haelt ihn ~0.9 s (OpenGL-Kontext, alle Netze laden) - deshalb wartet KameraThread beim
Erzeugen, bis die Kamera bereit ist: das faellt in den Start des Knotens, bevor Physik und Sensordaten laufen. Vorher
blieben in dieser Zeit die Kommandos > 1 s aus und die Simulation ging in den Daempfungsmodus.

Achsen: eine MuJoCo-Kamera blickt entlang ihrer -z-Achse, x nach rechts, y nach oben. ROS erwartet fuer Bilder den
optical frame (z nach vorn, x nach rechts, y nach unten); den liefert der TF-Baum (beschreibung.launch.py:
kamera_link -> kamera_optical_frame). Das Tiefenbild ist die Tiefe entlang der Blickachse (z im optical frame), nicht die
Strahllaenge - wie bei echten Tiefenkameras und wie ROS (REP 118) es erwartet.
"""
import threading

import mujoco
import numpy as np

from roboter_beschreibung.geometrie import rotmat_aus_rpy

KAMERA_NAME = "kopfkamera"

# Achsen der MuJoCo-Kamera, ausgedrueckt in kamera_link (ROS: x vorn, y links, z oben), als Spalten:
#   Kamera-x (rechts) = -y,  Kamera-y (oben) = +z,  Kamera-z (zeigt nach hinten, Blick entlang -z) = -x
R_LINK_ZU_MUJOCO_KAMERA = np.array([[0.0, 0.0, -1.0],
                                    [-1.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0]])

# Rendereinstellungen, die bei Software-Rendering viel Zeit kosten, fuer eine Tiefenkamera aber nichts bringen
# (gemessen 11.11.2026, 320x240 Farbe: alles an ~120 ms, ohne Schatten ~60 ms, ohne alle vier ~30 ms).
AUS = (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION,
       mujoco.mjtRndFlag.mjRND_SKYBOX, mujoco.mjtRndFlag.mjRND_HAZE)


def kamera_quat(rpy) -> np.ndarray:
    """Orientierung der MuJoCo-Kamera relativ zum Elternglied [w, x, y, z] aus der Montage (rpy in ROS-Konvention)."""
    R = rotmat_aus_rpy(*rpy) @ R_LINK_ZU_MUJOCO_KAMERA
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(-1))
    return q


def fuege_kamera_hinzu(spec: mujoco.MjSpec, montage: dict) -> None:
    """Kamera nach roboter.yaml (Eintrag kamera) an das Elternglied haengen; vor spec.compile() aufrufen."""
    eltern = spec.body(montage["parent"])
    if eltern is None:
        raise ValueError(f"Kamera-Elternglied {montage['parent']!r} nicht im Modell")
    breite, hoehe = int(montage["breite"]), int(montage["hoehe"])
    eltern.add_camera(name=KAMERA_NAME, pos=list(montage["xyz"]), quat=list(kamera_quat(montage["rpy"])),
                      fovy=float(montage["fovy_grad"]), resolution=[breite, hoehe])
    # Der Renderer zeichnet in einen Puffer fester Groesse (Standard 640x480); groessere Bilder brauchen einen groesseren.
    g = spec.visual.global_
    g.offwidth, g.offheight = max(g.offwidth, breite), max(g.offheight, hoehe)


def intrinsik(breite: int, hoehe: int, fovy_grad: float):
    """(fx, fy, cx, cy) in Pixeln, ROS/OpenCV-Konvention (Pixel (0, 0) = Mitte des ersten Pixels).

    MuJoCo kennt nur den vertikalen Oeffnungswinkel; die Pixel sind quadratisch (fx = fy), der Hauptpunkt liegt in der
    Bildmitte."""
    fy = (hoehe / 2.0) / np.tan(np.radians(fovy_grad) / 2.0)
    return float(fy), float(fy), (breite - 1) / 2.0, (hoehe - 1) / 2.0


def tiefe_aufbereiten(tiefe: np.ndarray, t_min: float, t_max: float, rng=None, rauschen: float = 0.0) -> np.ndarray:
    """Tiefe [m] wie von einer echten Tiefenkamera: ausserhalb [t_min, t_max] ungueltig (NaN; auch Hintergrund, den der
    Renderer mit der Fernebene fuellt); mit rng Rauschen sigma = rauschen * Tiefe^2 (typisch fuer Stereo-Tiefe)."""
    t = tiefe.astype(np.float32, copy=True)
    if rng is not None and rauschen > 0.0:
        t += (rng.standard_normal(t.shape) * rauschen * t * t).astype(np.float32)
    t[(t < t_min) | (t > t_max) | ~np.isfinite(t)] = np.nan
    return t


class Kamera:
    """Rendert Farb- und Tiefenbild aus Sicht der Modellkamera. Nur in dem Thread benutzen, der sie erzeugt hat
    (der OpenGL-Kontext gehoert diesem Thread)."""

    def __init__(self, model: mujoco.MjModel, name: str = KAMERA_NAME):
        self.id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if self.id < 0:
            raise ValueError(f"Kamera {name!r} nicht im Modell (szene.py: fuege_kamera_hinzu)")
        self.name = name
        self.breite, self.hoehe = (int(v) for v in model.cam_resolution[self.id])
        self.fovy_grad = float(model.cam_fovy[self.id])
        self.renderer = mujoco.Renderer(model, height=self.hoehe, width=self.breite)   # legt den OpenGL-Kontext an
        self._aus_gesetzt = False

    def intrinsik(self):
        return intrinsik(self.breite, self.hoehe, self.fovy_grad)

    def bilder(self, data: mujoco.MjData):
        """(rgb, tiefe): (hoehe, breite, 3) uint8 und (hoehe, breite) float32 in m (Hintergrund = Fernebene)."""
        r = self.renderer
        r.update_scene(data, camera=self.name)         # 1. Szene aus dem aktuellen Zustand aufbauen
        if not self._aus_gesetzt:                      # die Schalter bleiben in der Szene gesetzt
            for f in AUS:
                r.scene.flags[f] = False
            self._aus_gesetzt = True
        r.disable_depth_rendering()
        rgb = r.render()                               # 2a. Farbbild zeichnen
        r.enable_depth_rendering()
        tiefe = r.render()                             # 2b. Tiefenbild zeichnen (dieselbe Szene)
        r.disable_depth_rendering()
        return rgb, tiefe

    def close(self):
        self.renderer.close()


class KameraThread:
    """Rendert in einem eigenen Thread mit eigener Kopie des Zustands (MjData), damit die Physik nie wartet.

    Der Simulationsthread ruft aufnehmen(qpos, stempel) im Kameratakt auf (kopiert nur qpos, ~0,4 KB). Der Kamerathread
    rendert die jeweils NEUESTE Aufnahme; ist er noch mit einem Bild beschaeftigt, wird die vorige Aufnahme ersetzt
    (Bilder entfallen statt sich zu stauen). Fertige Bilder gehen an bei_bild(rgb, tiefe, stempel); ROS kennt diese
    Klasse nicht (die Nachrichten baut mujoco_node)."""

    def __init__(self, model: mujoco.MjModel, bei_bild, t_min: float, t_max: float, rng=None, rauschen: float = 0.0,
                 log_fehler=print, name: str = KAMERA_NAME, bereit_timeout_s: float = 30.0):
        """Kehrt erst zurueck, wenn die Kamera im Thread angelegt ist (oder das Anlegen gescheitert ist), siehe oben."""
        self._model, self._name = model, name
        self._data = mujoco.MjData(model)
        self._bei_bild, self._log_fehler = bei_bild, log_fehler
        self._t_min, self._t_max, self._rng, self._rauschen = t_min, t_max, rng, rauschen
        self._sperre = threading.Lock()
        self._neu = threading.Event()
        self._aufnahme = None
        self._laeuft = True
        self._bereit = threading.Event()
        self.verfuegbar = False
        self._thread = threading.Thread(target=self._schleife, name="kamera", daemon=True)
        self._thread.start()
        if not self._bereit.wait(timeout=bereit_timeout_s):
            self._log_fehler(f"Kamera nach {bereit_timeout_s:.0f} s noch nicht bereit - Start geht ohne Warten weiter.")

    def aufnehmen(self, qpos, stempel) -> None:
        with self._sperre:
            self._aufnahme = (np.array(qpos, copy=True), stempel)
        self._neu.set()

    def stoppen(self, timeout_s: float = 2.0) -> None:
        self._laeuft = False
        self._neu.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout_s)

    def _schleife(self):
        try:
            kamera = Kamera(self._model, self._name)   # hier, nicht im Konstruktor: OpenGL-Kontext dieses Threads
        except Exception as e:
            self._log_fehler(f"Kamera nicht verfuegbar (OpenGL? MUJOCO_GL=glfw|egl|osmesa): {type(e).__name__}: {e}")
            self._bereit.set()
            return
        self.verfuegbar = True
        self._bereit.set()
        try:
            while self._laeuft:
                if not self._neu.wait(timeout=0.1):
                    continue
                self._neu.clear()
                with self._sperre:
                    qpos, stempel = self._aufnahme
                # Nur die Lage aller Koerper und Kameras ausrechnen (keine Dynamik): mehr braucht update_scene nicht.
                self._data.qpos[:] = qpos
                mujoco.mj_kinematics(self._model, self._data)
                mujoco.mj_comPos(self._model, self._data)
                mujoco.mj_camlight(self._model, self._data)
                rgb, tiefe = kamera.bilder(self._data)
                tiefe = tiefe_aufbereiten(tiefe, self._t_min, self._t_max, self._rng, self._rauschen)
                if self._laeuft:
                    self._bei_bild(rgb, tiefe, stempel)
        except Exception as e:
            # Beim Beenden kann das Publizieren auf den schon zerstoerten Knoten werfen - das ist kein Fehler.
            if self._laeuft:
                self._log_fehler(f"Kamerathread beendet: {type(e).__name__}: {e}")
        finally:
            kamera.close()
