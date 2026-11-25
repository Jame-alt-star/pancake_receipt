"""Baut eine RoboCasa-Kueche (Layout + Style) und speichert die komplette Szene als eine MJCF-Datei.

Laeuft NICHT in der ROS-Umgebung, sondern mit dem Python der RoboCasa-Umgebung (robosuite + robocasa,
mujoco 3.3.1, numpy 2.2.5), z. B.:
    ~/tools/robocasa_venv/bin/python -E src/roboter_welten/roboter_welten/robocasa_export.py 7 7 /tmp/kueche7
(-E: PYTHONPATH der ROS-Umgebung ignorieren.) Normalerweise ruft robocasa_import.py dieses Skript selbst auf.

Was passiert (RoboCasa-Code, environments/kitchen/kitchen.py -> _setup_model):
  KitchenArena(layout, style)  liest kitchen_layouts/.../layoutNNN.yaml + kitchen_styles/.../styleNNN.yaml
  create_fixtures()            je Eintrag die Fixture-Klasse (Counter, HingeCabinet, Sink, Fridge..., Accessory),
                               MJCF laden (Vorlage bzw. Geraet aus dem Asset-Archiv), Groesse/Textur, Position
  ManipulationTask(...)        Arena + Roboter + Fixtures zu EINEM XML-Baum, robosuite kompiliert ihn mit MuJoCo
Die Umgebung "Kitchen" ist die Kueche ohne Aufgabe (keine Aufgabenobjekte); RoboCasa verlangt genau einen Roboter
(PandaOmron), den robocasa_import.py wieder entfernt.

Ausgabe in <ziel>/:
  szene_roh.xml   env.sim.model.get_xml(): komplette Szene inkl. Roboter, Dateipfade absolut (~/tools/robocasa/...)
  fixtures.json   je Fixture Name, Klasse, Position, Groesse, Drehung (fuer Orte und Pruefung)
"""
import json
import os
import sys


def exportiere(layout: int, style: int, ziel: str, seed: int = 0) -> str:
    import numpy as np
    import robosuite
    import robocasa  # noqa: F401  (registriert die Kitchen-Umgebungen bei robosuite)
    from robosuite.controllers import load_composite_controller_config

    os.makedirs(ziel, exist_ok=True)
    env = robosuite.make(
        env_name="Kitchen",
        robots="PandaOmron",
        controller_configs=load_composite_controller_config(robot="PandaOmron"),
        layout_and_style_ids=[[layout, style]],
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        ignore_done=True,
        seed=seed,                 # fester Seed: zufaellige Varianten (z. B. Glas-Modell) jedes Mal gleich
        translucent_robot=False,
    )
    env.reset()
    assert (env.layout_id, env.style_id) == (layout, style), (env.layout_id, env.style_id)

    with open(os.path.join(ziel, "szene_roh.xml"), "w") as f:
        f.write(env.sim.model.get_xml())

    def liste(x):
        try:
            return [float(v) for v in np.asarray(x).ravel()]
        except Exception:
            return None

    fixtures = {}
    for name, fx in env.fixtures.items():
        fixtures[name] = dict(klasse=type(fx).__name__, pos=liste(getattr(fx, "pos", None)),
                              groesse=liste(getattr(fx, "size", None)), rot=liste(getattr(fx, "rot", None)),
                              root_body=getattr(fx, "root_body", None))
    with open(os.path.join(ziel, "fixtures.json"), "w") as f:
        json.dump(dict(layout=layout, style=style, seed=seed, fixtures=fixtures), f, indent=1)
    env.close()
    print(f"Layout {layout} / Style {style}: {len(fixtures)} Fixtures -> {ziel}/szene_roh.xml")
    return os.path.join(ziel, "szene_roh.xml")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        sys.exit("Aufruf: robocasa_export.py <layout> <style> <zielordner> [seed]")
    exportiere(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3], int(sys.argv[4]) if len(sys.argv) > 4 else 0)
