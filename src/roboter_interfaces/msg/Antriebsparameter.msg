# Antriebsparameter, die Layer 3 (Steuerung) an Layer 4 (Simulation/Motortreiber) gibt. Gehoeren zur
# trainierten Policy (roboter_steuerung/config/policy_29dof.yaml), nicht zur Simulation. Auf echter Hardware
# sendet die Bridge kp/kd mit jedem Gelenkkommando (Unitree LowCmd); die Simulation bekommt sie einmal
# ueber /antrieb/parameter (latched) und startet erst danach.
# Alle Felder in Gelenkreihenfolge des Modells (MUJOCO_JOINT_ORDER), Laenge = Anzahl Gelenke.
float32[] kp          # Steifigkeit [Nm/rad]
float32[] kd          # Daempfung [Nm s/rad]
float32[] startpose   # Gelenkwinkel der Ruhepose [rad]
