# Fingerkommando fuer die beiden Dex3-1-Haende (Layer 3 -> Layer 4), Topic /hand/kommando.
# Wie Unitrees Dex3-HandCmd: je Fingergelenk Zielwinkel und PD-Verstaerkungen. Reihenfolge der 14 Gelenke:
# roboter_beschreibung/config/roboter.yaml (haende.gelenke): links Daumen 0-2, Mittelfinger 0-1, Zeigefinger 0-1,
# dann rechts ebenso. Die Simulation haelt das letzte Kommando, bis ein neues kommt (kein Loslassen bei Ausfall).
float32[] q          # Zielwinkel [rad]
float32[] kp         # Steifigkeit [Nm/rad]
float32[] kd         # Daempfung [Nm s/rad]
