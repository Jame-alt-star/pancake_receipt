# Armfuehrung (Layer 3): Greif-Dienst -> Steuereinheit, Topic /arme/kommando.
# Solange die Arme gefuehrt werden, ersetzt die Steuereinheit die Policy-Ausgabe fuer die 14 Armgelenke durch diese
# Ziele, gibt der Policy die Arme als Ruhepose zu sehen ("blind fuer die Arme") und neigt die Taille als Gegengewicht.
# Der Modus ist ausdruecklich: Ausfall oder Abbruch des Senders soll die Arme halten, nicht loslassen; die Arme
# gehen nur auf AUSBLENDEN zurueck an die Policy (siehe roboter_steuerung/armfuehrung.py).
uint8 FUEHREN=1      # Ziele folgen (Ratenbegrenzung in der Steuereinheit)
uint8 HALTEN=2       # letzte Ziele halten (Ziele dieser Nachricht werden ignoriert)
uint8 AUSBLENDEN=3   # Arme weich an die Policy zurueckgeben, danach aus
uint8 modus
string[] namen       # die 14 Armgelenke (links, rechts; Reihenfolge wie MUJOCO_JOINT_ORDER 15..28), wird geprueft
float32[] ziele      # Zielwinkel [rad], gleiche Reihenfolge wie namen
float32 taille_anteil   # 0..1: Anteil der Taillenneigung (manipulieren_taille_pitch in steuerung.yaml)
