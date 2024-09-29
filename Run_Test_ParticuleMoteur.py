# -*- coding: utf-8 -*-

from particule import *
from random import random

"""
Ce fichier permet de simuler le système moteur+ressort+particule
Par défaut, la vitesse du moteur augmente de 1 rad/s toutes les secondes
Si vous vouler imposer une vitesse particulière, à modifier dans le fichier particule.py (ligne 147)
"""

# Paramètres du moteur
R = 1  
L = 0.001  
kc = 0.01  
ke = 0.01  
J = 0.01  
f = 0.1  

# Création du moteur et du contrôleur
center=Vecteur3D(400, 250, 0)
moteur_bf = MoteurCC(R, L, kc, ke, J, f, center=center)
control_PID = ControlPID_vitesse(moteur_bf, Kp=5, Ki=50, Kd=0.1)

# Création des particules
P1 = Particule(mass=1, p0=Vecteur3D(350, 150, 0), v0=Vecteur3D(0, 0, 0), name="Particule 1", color="green")
P_fix = Particule(p0=center, fix=True, name="Fixie", color="red")

# Création de l'univers
espace_temps = Univers(moteur=control_PID,name='Système moteur+ressort+particule')
espace_temps.addAgent( P1, P_fix)

# Ajout des forces
f_down = Gravity(Vecteur3D(0, -10, 0), active=True)
prismatic = Prismatic(P_fix, P1, axis=Vecteur3D(0, 0, 1))
espace_temps.addGenerators(f_down, prismatic)

# Simulation en temps réel
espace_temps.simulateRealTime(scale=1, background=(30, 30, 30))

# Affichage des résultats
espace_temps.plot()
espace_temps.plot_distance_vs_motor_speed()
