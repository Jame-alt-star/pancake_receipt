"""Armfuehrung (armfuehrung.py) und Mischung mit der Policy (greifen/manipulation.py), ohne ROS."""
import numpy as np

from roboter_steuerung.armfuehrung import AUS, AUSBLENDEN, FUEHREN, HALTEN, Armfuehrung
from roboter_steuerung.greifen.manipulation import ARM_INDIZES, maskiere_arme, mische

NAMEN = [f"g{i}" for i in range(14)]
DT = 0.02


def neu(**kw):
    return Armfuehrung(NAMEN, **kw)


def test_aus_gibt_nichts_zurueck():
    a = neu()
    assert a.takt(np.zeros(14), np.zeros(14), 0.0, DT) is None and not a.aktiv


def test_fuehren_mit_ratenbegrenzung_und_einblenden():
    a = neu(max_rate=2.0, einblenden_s=0.5)
    assert a.empfange(1, NAMEN, np.full(14, 1.0), 1.0, 0.0) is None
    arme, gewicht, taille = a.takt(np.zeros(14), np.zeros(14), 0.0, DT)
    assert np.allclose(arme, 2.0 * DT)               # hoechstens max_rate*dt je Takt vom Messwert aus
    assert np.isclose(gewicht, DT / 0.5) and taille == 1.0
    for i in range(100):
        arme, gewicht, _ = a.takt(np.zeros(14), np.zeros(14), 0.01 * i, DT)
    assert np.allclose(arme, 1.0) and gewicht == 1.0


def test_falsche_namen_laenge_oder_nan_werden_abgelehnt():
    a = neu()
    assert a.empfange(1, NAMEN[::-1], np.zeros(14), 0.0, 0.0) is not None
    assert a.empfange(1, NAMEN, np.zeros(13), 0.0, 0.0) is not None
    assert a.empfange(1, NAMEN, np.full(14, np.nan), 0.0, 0.0) is not None
    assert a.empfange(9, NAMEN, np.zeros(14), 0.0, 0.0) is not None
    assert a.modus == AUS


def test_timeout_haelt_statt_loszulassen():
    a = neu(timeout_s=0.5)
    a.empfange(1, NAMEN, np.full(14, 0.1), 0.0, 0.0)
    for i in range(50):
        a.takt(np.zeros(14), np.zeros(14), 0.01 * i, DT)
    a.takt(np.zeros(14), np.zeros(14), 1.0, DT)      # keine Nachricht seit 1 s
    assert a.modus == HALTEN
    arme, gewicht, _ = a.takt(np.full(14, 5.0), np.zeros(14), 2.0, DT)
    assert gewicht == 1.0 and np.allclose(arme, 0.1)  # die Policy-Arme (5.0) werden nicht uebernommen


def test_halten_von_aussen_und_ausblenden_nur_auf_befehl():
    a = neu(ausblenden_s=0.2)
    a.empfange(1, NAMEN, np.full(14, 0.1), 1.0, 0.0)
    for i in range(30):
        a.takt(np.zeros(14), np.zeros(14), 0.01 * i, DT)
    a.halten()                                      # z. B. NotAus
    assert a.modus == HALTEN
    a.empfange(1, NAMEN, np.full(14, 0.5), 1.0, 0.5)  # neues Fuehren erlaubt (der Knoten sperrt es im NotAus)
    assert a.modus == FUEHREN
    a.empfange(3, [], [], 0.0, 0.6)
    assert a.modus == AUSBLENDEN
    for i in range(20):
        r = a.takt(np.zeros(14), np.zeros(14), 0.6 + 0.02 * i, DT)
    assert r is None and a.modus == AUS and a.taille_anteil == 0.0


def test_halten_und_ausblenden_ohne_aktive_fuehrung_bleiben_aus():
    a = neu()
    a.empfange(2, [], [], 0.0, 0.0)
    a.empfange(3, [], [], 0.0, 0.0)
    a.halten()
    assert a.modus == AUS


def test_maskieren_und_mischen():
    ruhe = np.arange(29, dtype=np.float32) * 0.01
    q, dq = np.full(29, 1.0, dtype=np.float32), np.full(29, 2.0, dtype=np.float32)
    mq, mdq = maskiere_arme(q, dq, ruhe)
    assert np.allclose(mq[ARM_INDIZES], ruhe[ARM_INDIZES]) and np.all(mdq[ARM_INDIZES] == 0)
    assert np.all(mq[:15] == 1.0) and np.all(q == 1.0)          # Eingabe unveraendert, Beine/Taille bleiben
    policy = np.zeros(29, dtype=np.float32)
    z = mische(policy, np.full(14, 1.0), 1.0, -0.15, 1.0)
    assert np.allclose(z[ARM_INDIZES], 1.0) and np.isclose(z[14], -0.15) and np.all(z[:14] == 0)
    z = mische(policy, np.full(14, 1.0), 0.5, -0.15, 0.0)
    assert np.allclose(z[ARM_INDIZES], 0.5) and z[14] == 0.0
