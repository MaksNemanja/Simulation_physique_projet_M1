# -*- coding: utf-8 -*-

from particule import *
from random import random
import matplotlib.pyplot as plt
import numpy as np

"""
Ce fichier permet de simuler le système masse + (ressort+amortisseur) 
avec application au clavier d’une force constante (exercie 3 q.1)
Pendant la simulation, vous pouvez émettre une force constante sur la masse en utilisant
les flèches sur votre clavier
Le fichier affiche la position selon y de la masse durant la simulation et la position théorique (selon les valeurs qu'on a attribué)
"""
                                                                                            
def systeme_theorique_3_1(m, c, k, x0, v0, t):
    """
    m : masse (kg)
    c : amortisseur (Ns/m)
    k : ressort (N/m)
    x0 : position initiale (m)
    v0 : vitesse initiale (m/s)
    t : temps (s)
    """
    # Fréquence propre non amortie
    omega_0 = np.sqrt(k / m)
    
    # Facteur d'amortissement
    csi = c / (2 * np.sqrt(m * k))
    
    # Fréquence propre amortie
    omega_d = omega_0 * np.sqrt(1 - csi**2)
    
    # Réponse théorique sous-amortie
    A = x0
    B = (v0 + csi * omega_0 * x0) / omega_d
    
    x_t = np.exp(-csi * omega_0 * t) * (A * np.cos(omega_d * t) + B * np.sin(omega_d * t))
    
    return x_t


#Création des particules
P_fix = Particule(p0=Vecteur3D(400, 200, 0), fix=True, name="Fixie", color="red")
masse = Particule(mass=1, name='masse',p0=Vecteur3D(400,300, 0),color='blue')

# Création de l'univers
espace_temps = Univers(name='Système masse+ressort+amortisseur+force constante')
espace_temps.addAgent(masse, P_fix)

# Ajout des forces   
spring = SpringDamper(P_fix,masse, k=1, c=0.1, l0=10)
Fc = ForceSelect(particules=[masse])
f_down = Gravity(Vecteur3D(0, -10, 0), active=True)
espace_temps.addGenerators(f_down, spring)
espace_temps.keyControlled=[Fc]

# Simulation en temps réel
espace_temps.simulateRealTime(scale=1, background=(30, 30, 30))

# Affichage du déplacement de la particule (a décommenter si vous voulez voir l'affichage)
#espace_temps.plot()

#Position selon y de la particule au cours de la simulation
Y = [p.y for p in masse.position]

# Paramètres du système pour la réponse théorique
m = 1
c = 0.1
k = 1
x0 = 100
v0 = 0

# Temps de simulation pour la réponse théorique
t = np.linspace(0, 50, 1000)

# Réponse théorique
x_t = systeme_theorique_3_1(m, c, k, x0, v0, t)

#Affichage de la réponse simulée et théorique
plt.figure(figsize=(12, 6))
plt.title('Oscillations masse/ressort')
plt.grid()
plt.xlabel('temps (s)')
plt.ylabel('position en y (m)')
plt.plot(t, x_t, label='Réponse théorique')
plt.plot(espace_temps.time,Y, label='Réponse simulée')
plt.legend()
plt.show()
