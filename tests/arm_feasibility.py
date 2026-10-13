#!/usr/bin/env python3
"""Machbarkeitstest: bleibt der Roboter mit der Lauf-Policy stehen, wenn die Arme greifen?

Synchroner Kreis ohne ROS2 (echter Controller + echtes Modell). Die Policy haelt Beine/Taille
(cmd = Kurshaltung + Positionshaltung wie im Knoten, Zustand Balancieren); die Armgelenke
(Index 15..28 in MuJoCo-Reihenfolge) werden von einer Trajektorie ueberschrieben. Die Policy
SIEHT die Armbewegung weiterhin in ihren Beobachtungen.

Ablauf: 10 s Stehen -> Arme in Greifpose (3 s) -> halten (4 s) -> Anheben +dz (2 s) -> halten
(4 s) -> Ablegen (2 s) -> halten (3 s) -> zurueck zur Ruhepose (3 s) -> 5 s Stehen.
Last: Zusatzmasse je Hand (Punktmasse am Handgelenk-Koerper, Traegheit vernachlaessigt).

Aufruf: python3 tests/arm_feasibility.py [--reach 0.30] [--loads 0 0.5 1.0] [--hold 60]
"""
import argparse
import sys
import os

import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_helpers import load_config, load_controller29, BRINGUP, yaw_deg  # noqa: E402

LARM = list(range(15, 22))
RARM = list(range(22, 29))


def smooth(a):
    return 0.5 - 0.5 * np.cos(np.pi * np.clip(a, 0.0, 1.0))


def solve_ik(m, base_qpos, q_start, hand_bodies, targets_world, arm_idx_sets, iters=200):
    """Gedaempfte kleinste Quadrate auf die Armgelenke; Basis und uebrige Gelenke fest."""
    d = mujoco.MjData(m)
    q = q_start.copy()
    dofs = [np.array([6 + i for i in s]) for s in arm_idx_sets]
    lo = m.jnt_range[1:, 0]
    hi = m.jnt_range[1:, 1]
    for _ in range(iters):
        d.qpos[:7] = base_qpos
        d.qpos[7:] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        err_all = 0.0
        for body, tgt, dof, idx in zip(hand_bodies, targets_world, dofs, arm_idx_sets):
            e = tgt - d.xpos[body]
            err_all += float(e @ e)
            jacp = np.zeros((3, m.nv))
            mujoco.mj_jacBody(m, d, jacp, None, body)
            J = jacp[:, dof]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), e)
            q[idx] = np.clip(q[idx] + 0.5 * dq, lo[idx], hi[idx])
        if err_all < 1e-8:
            break
    return q, float(np.sqrt(err_all))


def run(reach, load, dz, hold_extra, use_poshold=True, verbose=False, obs_mask=False, obs_blend=0.0, tscale=1.0,
        waist=(0.0, 0.0, 0.0), y_lat=0.17, z_off=0.0, elbow_seed=0.0):
    cfg = load_config()
    ctl = load_controller29(cfg)
    m = mujoco.MjModel.from_xml_path(BRINGUP + cfg["xml_path"])
    m.opt.timestep = float(cfg["simulation_dt"])
    lw = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
    rw = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
    m.body_mass[lw] += load
    m.body_mass[rw] += load
    d = mujoco.MjData(m)
    kps = np.array(cfg["kps"], dtype=np.float32)
    kds = np.array(cfg["kds"], dtype=np.float32)
    home = np.array(cfg["default_angles"], dtype=np.float32)
    tgt = home.copy()
    d.qpos[7:] = home
    mujoco.mj_forward(m, d)
    pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    base0 = d.qpos[:7].copy()
    p0 = d.xpos[pel].copy()
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
            for n in ("left_ankle_roll_link", "right_ankle_roll_link")]

    # Greifposen per IK (Welt-Ziel relativ zur Beckenposition im Stand)
    t_l = p0 + np.array([reach, y_lat, z_off])
    t_r = p0 + np.array([reach, -y_lat, z_off])
    q_seed = home.copy()
    q_seed[18] += elbow_seed      # linker Ellbogen
    q_seed[25] += elbow_seed      # rechter Ellbogen
    q_grasp, e1 = solve_ik(m, base0, q_seed, [lw, rw], [t_l, t_r], [LARM, RARM])
    q_lift, e2 = solve_ik(m, base0, q_grasp.copy(), [lw, rw],
                          [t_l + [0, 0, dz], t_r + [0, 0, dz]], [LARM, RARM])
    ik_err = max(e1, e2)

    # Phasen: (Dauer, von, nach)
    ts = tscale
    seq = [(10, home, home), (3 * ts, home, q_grasp), (4, q_grasp, q_grasp), (2 * ts, q_grasp, q_lift),
           (4, q_lift, q_lift), (2 * ts, q_lift, q_grasp), (3, q_grasp, q_grasp), (3 * ts, q_grasp, home),
           (5 + hold_extra, home, home)]
    names = ["Stehen", "Greifen", "halten", "Anheben", "oben", "Ablegen", "unten", "Zurueck", "Ende"]
    if hold_extra:   # Dauer-Test: lange in der Greifpose mit Last halten statt zurueck
        seq = [(10, home, home), (3 * ts, home, q_grasp), (hold_extra, q_grasp, q_grasp)]
        names = ["Stehen", "Greifen", "halten"]

    t_edges = np.cumsum([0] + [s[0] for s in seq])
    total = t_edges[-1]
    tp = None
    ty = None
    res = dict(fell=None, ik_err=ik_err, drift=0.0, tilt=0.0, minz=9.0, lift=0.0, legdq=0.0,
               ph={n: dict(drift=0.0, tilt=0.0, legdq=0.0) for n in names})
    dec = int(cfg["control_decimation"])
    cmd = np.zeros(3, dtype=np.float32)
    for i in range(int(total / m.opt.timestep)):
        d.ctrl[:] = (tgt - d.qpos[7:]) * kps - d.qvel[6:] * kds
        mujoco.mj_step(m, d)
        t = d.time
        if (i + 1) % dec == 0:
            q = d.qpos[3:7]
            yaw = np.radians(yaw_deg(q))
            if ty is None:
                ty = yaw
            cmd[:] = 0.0
            cmd[2] = np.clip(1.0 * ((ty - yaw + np.pi) % (2 * np.pi) - np.pi), -0.2, 0.2)
            if use_poshold:
                if tp is None and t >= 5:
                    tp = d.qpos[:2].copy()
                if tp is not None:
                    e = tp - d.qpos[:2]
                    cmd[0] = np.clip(np.cos(yaw) * e[0] + np.sin(yaw) * e[1], -0.15, 0.15)
                    cmd[1] = np.clip(-np.sin(yaw) * e[0] + np.cos(yaw) * e[1], -0.15, 0.15)
            jp = d.qpos[7:].copy()
            jv = d.qvel[6:].copy()
            if obs_mask:          # Policy sieht die Arme in Ruhepose (Armbewegung = unbekannte Stoerung)
                lam = obs_blend   # 0 = blind, 1 = echte Beobachtung
                jp[15:29] = home[15:29] + lam * (jp[15:29] - home[15:29])
                jv[15:29] = lam * jv[15:29]
            tgt = ctl.compute_joint_targets(jp, jv, q.copy(), d.qvel[3:6].copy(), cmd)
            k = int(np.searchsorted(t_edges, t, side="right") - 1)
            k = min(k, len(seq) - 1)
            dur, a, b = seq[k]
            s = smooth((t - t_edges[k]) / dur)
            arm = (1 - s) * a + s * b
            tgt = tgt.copy()
            tgt[15:29] = arm[15:29]
            alpha = float(np.linalg.norm(arm[15:29] - home[15:29]) / (np.linalg.norm(q_grasp[15:29] - home[15:29]) + 1e-9))
            tgt[12:15] = tgt[12:15] + np.clip(alpha, 0, 1) * np.array(waist)
        # Metriken
        k = min(int(np.searchsorted(t_edges, t, side="right") - 1), len(seq) - 1)
        ph = res["ph"][names[k]]
        g_z = 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)   # z-Komponente der Gravitationsrichtung im Koerper
        tilt = float(np.degrees(np.arccos(np.clip(g_z, -1, 1))))
        drift = float(np.linalg.norm(d.qpos[:2] - (tp if tp is not None else d.qpos[:2])))
        legdq = float(np.abs(d.qvel[6:18]).mean()) if t > 2.0 else 0.0   # Einschwingen ausnehmen
        flift = float(max(d.xpos[f][2] for f in feet) - 0.034)
        res["tilt"] = max(res["tilt"], tilt)
        res["drift"] = max(res["drift"], drift)
        res["minz"] = min(res["minz"], d.qpos[2])
        res["lift"] = max(res["lift"], flift)
        res["legdq"] = max(res["legdq"], legdq)
        ph["drift"] = max(ph["drift"], drift)
        ph["tilt"] = max(ph["tilt"], tilt)
        ph["legdq"] = max(ph["legdq"], legdq)
        if d.qpos[2] < 0.4:
            res["fell"] = t
            break
    res["end_drift"] = float(np.linalg.norm(d.qpos[:2] - tp)) if tp is not None else 0.0
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reach", type=float, default=0.30, help="Hand nach vorn rel. Becken [m]")
    ap.add_argument("--loads", type=float, nargs="+", default=[0.0, 0.5, 1.0],
                    help="Zusatzmasse JE HAND [kg]")
    ap.add_argument("--dz", type=float, default=0.15, help="Anheben [m]")
    ap.add_argument("--hold", type=float, default=0.0, help="Dauertest: so lange in Greifpose [s]")
    ap.add_argument("--no-poshold", action="store_true")
    ap.add_argument("--empfohlen", action="store_true",
                    help="Betriebspunkt: Policy blind fuer Arme, Taille -0.15 rad, ylat 0.15")
    ap.add_argument("--phases", action="store_true", help="Ergebnisse je Phase")
    ap.add_argument("--obs-mask", action="store_true", help="Policy sieht Arme in Ruhepose")
    ap.add_argument("--obs-blend", type=float, default=0.0, help="mit --obs-mask: Anteil echter Arm-Beobachtung")
    ap.add_argument("--tscale", type=float, default=1.0, help="Faktor auf Dauer der Armbewegungen")
    ap.add_argument("--waist", type=float, nargs=3, default=[0, 0, 0], help="Taille yaw roll pitch Offset [rad]")
    ap.add_argument("--ylat", type=float, default=0.17)
    ap.add_argument("--zoff", type=float, default=0.0)
    ap.add_argument("--elbow-seed", type=float, default=0.0)
    a = ap.parse_args()
    if a.empfohlen:
        a.obs_mask, a.obs_blend, a.waist, a.ylat = True, 0.0, [0, 0, -0.15], 0.15
    print(f"Greifpose: Haende {a.reach:.2f} m vor dem Becken, Anheben {a.dz:.2f} m, "
          f"Positionshaltung {'aus' if a.no_poshold else 'an'}")
    print(f"{'Last/Hand':>9} | {'IK-Fehler':>9} | {'Sturz':>8} | {'max Drift':>9} | {'Ende':>7} | "
          f"{'max Neigung':>11} | {'min z':>6} | {'Fuss hoch':>9} | {'max |dq| Beine':>14}")
    for load in a.loads:
        r = run(a.reach, load, a.dz, a.hold, use_poshold=not a.no_poshold, obs_mask=a.obs_mask, obs_blend=a.obs_blend,
                tscale=a.tscale, waist=tuple(a.waist), y_lat=a.ylat, z_off=a.zoff, elbow_seed=a.elbow_seed)
        fell = f"{r['fell']:.1f}s" if r["fell"] else "nein"
        print(f"{load:7.1f}kg | {r['ik_err']*100:7.1f}cm | {fell:>8} | {r['drift']*100:7.1f}cm | "
              f"{r['end_drift']*100:5.1f}cm | {r['tilt']:9.1f}° | {r['minz']:6.3f} | "
              f"{r['lift']*100:7.1f}cm | {r['legdq']:11.3f} rad/s")
        if a.phases:
            for n, p in r["ph"].items():
                print(f"             {n:9s} drift {p['drift']*100:5.1f} cm  neigung {p['tilt']:4.1f}°  "
                      f"legdq {p['legdq']:.3f}")


if __name__ == "__main__":
    main()
