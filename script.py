import numpy as np
import time
import matplotlib.pyplot as plt
from pyniryo import *

# Fonction PID
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Gain proportionnel
        self.Ki = Ki  # Gain intégral
        self.Kd = Kd  # Gain dérivé
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        # Calcul de l'erreur
        error = setpoint - current_value

        # Intégrale de l'erreur
        self.integral += error

        # Dérivée de l'erreur
        derivative = error - self.previous_error

        # Calcul du contrôle PID
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Mémoriser l'erreur pour le prochain calcul
        self.previous_error = error

        return output

# Tracer les résultats avec Matplotlib
plt.figure(figsize=(10, 8))

# Graphique de la position du robot
plt.subplot(3, 1, 1)
plt.plot(time_steps, z_values, label='Position du robot (Z)', color='b')
plt.axhline(y=setpoint_z, color='r', linestyle='--', label='Position cible (setpoint)')
plt.title("Position du robot vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Position Z (m)")
plt.legend()

# Graphique de l'erreur
plt.subplot(3, 1, 2)
plt.plot(time_steps, errors, label='Erreur', color='g')
plt.title("Erreur du robot vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Erreur (m)")
plt.legend()

# Graphique de l'entrée de contrôle
plt.subplot(3, 1, 3)
plt.plot(time_steps, control_inputs, label="Entrée de contrôle PID", color='orange')
plt.title("Entrée de contrôle PID vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Entrée de contrôle")
plt.legend()

# Ajuster les espacements entre les graphiques
plt.tight_layout()
plt.show()
# Connexion au robot
robot = NiryoRobot("10.10.10.10")
robot.calibrate_auto()
robot.update_tool()

# Initialisation du contrôleur PID
Kp = 1.0  # Ajuster ces valeurs
Ki = 0.1
Kd = 0.05
pid = PID(Kp, Ki, Kd)

# Position cible pour l'axe Z
setpoint_z = 0.3  # Position cible (en mètre)

# Position initiale (à adapter selon la position actuelle)
initial_position = robot.get_current_pose()
current_z = initial_position[2]  # Récupère la position Z actuelle

# Listes pour enregistrer les données
time_steps = []
z_values = []
errors = []
control_inputs = []

# Boucle PID pour ajuster la position du robot sur l'axe Z
for i in range(50):  # 50 itérations pour ajuster la position
    # Calcul du PID pour la correction de l'axe Z
    correction = pid.compute(setpoint_z, current_z)
    
    # Appliquer la correction à la position du robot
    new_z = current_z + correction
    
    # Limiter la correction si elle est trop grande (safety check)
    new_z = max(0.0, min(0.5, new_z))  # Contrainte sur la plage de l'axe Z, à ajuster

    # Déplacer le robot avec la nouvelle position corrigée sur l'axe Z
    robot.move_pose(initial_position[0], initial_position[1], new_z, initial_position[3], initial_position[4], initial_position[5])

    # Enregistrer les résultats pour les graphiques
    time_steps.append(i)
    z_values.append(new_z)
    errors.append(setpoint_z - new_z)
    control_inputs.append(correction)

    # Mettre à jour la position actuelle
    current_z = new_z

    print(f"Iteration {i+1}: Current Z = {current_z}, Target Z = {setpoint_z}")

    # Attente avant la prochaine itération
    time.sleep(0.1)

# Libérer le robot et fermer la connexion
robot.release_with_tool()
robot.close_connection()

# Tracer les résultats avec Matplotlib
plt.figure(figsize=(10, 8))

# Graphique de la position du robot
plt.subplot(3, 1, 1)
plt.plot(time_steps, z_values, label='Position du robot (Z)', color='b')
plt.axhline(y=setpoint_z, color='r', linestyle='--', label='Position cible (setpoint)')
plt.title("Position du robot vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Position Z (m)")
plt.legend()

# Graphique de l'erreur
plt.subplot(3, 1, 2)
plt.plot(time_steps, errors, label='Erreur', color='g')
plt.title("Erreur du robot vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Erreur (m)")
plt.legend()

# Graphique de l'entrée de contrôle
plt.subplot(3, 1, 3)
plt.plot(time_steps, control_inputs, label="Entrée de contrôle PID", color='orange')
plt.title("Entrée de contrôle PID vs Temps")
plt.xlabel("Temps (itérations)")
plt.ylabel("Entrée de contrôle")
plt.legend()

# Ajuster les espacements entre les graphiques
plt.tight_layout()
plt.show()
