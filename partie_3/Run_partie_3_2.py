# -*- coding: utf-8 -*-

from particule import *
from random import random
import matplotlib.pyplot as plt

"""
Ce fichier permet de simuler le système de 3 pendules avec l0=10, l0=20 et l0=30
"""

P_fix = Particule(p0=Vecteur3D(400, 200, 0), fix=True, name="Fixie", color="red")
particule1 = Particule(name='masse',p0=Vecteur3D(400,300, 0),color='blue')
particule2 = Particule(name='masse',p0=Vecteur3D(400,300, 0),color='green')
particule3 = Particule(name='masse',p0=Vecteur3D(400,300, 0),color='yellow')

# Création de l'univers
espace_temps = Univers(name='3 pendules')
espace_temps.addAgent(particule1, particule2, particule3, P_fix)

f_down = Gravity(Vecteur3D(0, -10, 0), active=True)
spring1 = SpringDamper(P_fix,particule1,k=1, c=0.1, l0=10)
spring2 = SpringDamper(P_fix,particule2,k=1, c=0.1, l0=20)
spring3 = SpringDamper(P_fix,particule3,k=1, c=0.1, l0=30)
espace_temps.addGenerators(f_down, spring1, spring2, spring3)

# Simulation en temps réel
espace_temps.simulateRealTime(scale=1, background=(30, 30, 30))

# Affichage des résultats
espace_temps.plot()

Y1 = [p.y for p in particule1.position]
Y2 = [p.y for p in particule2.position]
Y3 = [p.y for p in particule3.position]

#Affichage de la réponse simulée et théorique
plt.figure(figsize=(12, 6))
plt.title('Système de 3 pensules')
plt.grid()
plt.xlabel('temps (s)')
plt.ylabel('position en y (m)')
plt.plot(espace_temps.time,Y1, label='l0=10')
plt.plot(espace_temps.time,Y2, label='l0=20')
plt.plot(espace_temps.time,Y3, label='l0=30')
plt.legend()
plt.show()