# -*- coding: utf-8 -*-
"""
Ce fichier contient la définition de la classe MoteurCC
Une simulation est disponible. Elle permet d'afficher la réponse indicielle d'un moteur en BO,
avec aussi la réponse théorique
"""

import numpy as np
import matplotlib.pyplot as plt
from vecteur3D import Vecteur3D

class MoteurCC:
    def __init__(self, R=0, L=0, kc=0, ke=0, J=0, f=0, J_charge=0, gamma_ext=0, center=Vecteur3D()):
        self.R = R #résistance (Ω)
        self.L = L #inductance de l'induit (H)
        self.kc = kc #conctante de couple (Nm/A)
        self.ke = ke #constante contre-électromotrice (Vs)
        self.J = J + J_charge #inertie du rotor + inertie de la charge (kg.m^2)
        self.f = f #frottements visqueux (Nm.s)
        self.Um = 0 # tension au bornes du moteur (V)
        self.Omega = [0] #tableau contenant la vitesse du rotor au fil du temps (rad/s)
        self.i = [0] #tableau contenant le courant au fil du temps (A)
        self.gamma_ext = gamma_ext #couple extérieur (Nm)
        self.gamma = [gamma_ext] #tableau contenant le couple moteur au fil du temps (Nm)
        self.position = [0] #tableau contenant la position du moteur au fil du temps (rad)
        self.center = center #définition du centre du moteur dans l'espace (Vecteur3D)
        
    def __str__(self):
        return f"Moteur CC en boucle ouverte"
    
    def __repr__(self):
        return self.__str__()
    
    #Méthode pour imposer une tension
    def setVoltage(self, V):
        self.Um = V
        
    #Methode pour récupérer la dernière valeur de la vitesse
    def getSpeed(self):
        return self.Omega[-1]
    
    #Methode pour récupérer la dernière valeur du courant
    def getIntensity(self):
        return self.i[-1]
    
    #Methode pour récupérer la dernière valeur du couple moteur
    def getTorque(self):
        return self.gamma[-1]
    
    #Methode pour récupérer la dernière valeur de la position
    def getPosition(self):
        return self.position[-1]
    
    #Méthode pour simuler le moteur    
    def simule(self, dt):
        Omega = self.getSpeed()
        
        # Calcul de l'accélération angulaire
        dOmega_dt = (self.kc / self.R * self.Um - (self.ke * self.kc / self.R + self.f) * Omega) / self.J
        new_Omega = Omega + dOmega_dt * dt
        self.Omega.append(new_Omega)

        # Calcul du courant
        new_i = (self.Um - self.ke * new_Omega) / self.R
        self.i.append(new_i)

        # Calcul du couple moteur
        new_gamma = self.gamma_ext + new_i * self.kc
        self.gamma.append(new_gamma)
        
        # Mise à jour de la position
        new_position = self.getPosition() + new_Omega * dt
        self.position.append(new_position)
    

if __name__ == "__main__":
    
    # Paramètres du moteur
    R = 1  
    L = 0.001  
    kc = 0.01
    ke = 0.01
    J = 0.01
    f = 0.1
    
    # Création du moteur
    moteur = MoteurCC(R, L, kc, ke, J, f)

    t = 0
    step = 0.01
    temps = [t]
    Omega_theorique = [0]
    
    while t < 2:
        t += step
        temps.append(t)
        moteur.setVoltage(1)
        moteur.simule(step)
        Omega_theorique.append((1 / 10.01) * (1 - np.exp(-10.01 * t)))
        
    # Affichage de la réponse indicielle simulée et numérique
    plt.figure(figsize=(12, 6))
    plt.plot(temps, moteur.Omega, label='Simulation numérique')
    plt.plot(temps, Omega_theorique, label='Réponse théorique', linestyle='--')
    plt.xlabel('Temps (s)')
    plt.ylabel('Vitesse $\Omega$ (rad/s)')
    plt.title('Comparaison de la réponse indicielle')
    plt.legend()
    plt.grid(True)
    plt.show()
