# -*- coding: utf-8 -*-

import numpy as np
from Run_moteurCC import *
import matplotlib.pyplot as plt
import pickle

"""
Ce fichier contient la définition de la classe ControlPID_position
Une simulation est disponible. Elle permet d'afficher la réponse indicielle d'un moteur en BF
en imposant une position cible
"""

class ControlPID_position:
    def __init__(self, moteur, Kp=0, Ki=0, Kd=0):
        self.moteur = moteur  #moteur (MoteurCC)
        self.Kp = Kp #gain proportionnel
        self.Ki = Ki #gain intégral
        self.Kd = Kd #gain dérivé
        self.target_position = 0 #vitesse cible
        self.previous_error = 0 # rreur de vitesse à l'itération précédente
        self.integral = 0 #somme accumulée des erreurs
        self.voltages = [] #liste des tensions appliquées au moteur au cours du temps
    
    def __str__(self):
        return f"ControlPID_position(Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd})"
    
    def __repr__(self):
        return self.__str__()
    
    #Méthode qui permet d'imposer une position cible
    def setTarget(self, position):
        self.target_position = position
    
    #Méthode pour récupérer la dernière valeur de la tension
    def getVoltage(self):
        return self.voltages[-1] if self.voltages else 0
    
    #Méthode pour simuler le système
    def simule(self, dt):
        current_position = self.moteur.getPosition()
        
        #Calcul de l'erreur
        error = self.target_position - current_position
        
        #Calcul du terme intégral
        self.integral += error * dt
        
        #Calcul du terme dérivé
        derivative = (error - self.previous_error) / dt
        
        #Calcul de la tension à appliquer au moteur
        voltage = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.moteur.setVoltage(voltage)
        
        #Simulation du moteur
        self.moteur.simule(dt)
        self.voltages.append(voltage)
        
        #Enregistrement de l'erreur
        self.previous_error = error
    
    #Méthode pour afficher la position du moteur et la tension au cours du temps
    def plot(self):
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(self.moteur.position, label='Position $x$(t)')
        plt.xlabel('Temps (s)')
        plt.ylabel('Position $x$ (m)')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(self.voltages, label='Tension $U_m$(t)', color='g')
        plt.xlabel('Temps (s)')
        plt.ylabel('Tension $U_m$ (V)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def save(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self, f)
    
    @staticmethod
    def load(filename):
        with open(filename, 'rb') as f:
            return pickle.load(f)


if __name__ == "__main__":
    # Paramètres du moteur
    R = 1
    L = 0.001  
    kc = 0.01  
    ke = 0.01
    J = 0.01
    f = 0.1
    
    # Création des moteurs
    m_bf1 = MoteurCC(R, L, kc, ke, J, f)
    m_bf2= MoteurCC(R, L, kc, ke, J, f)
    m_bf3 = MoteurCC(R, L, kc, ke, J, f)
    
    # Paramètres des contrôleurs
    Kp1 = 10  # gain proportionnel
    Kp2 = 40  # gain proportionnel
    Kp3 = 300  # gain proportionnel
    Ki = 0  # gain intégral
    Kd = 0 # gain dérivé
    control1 = ControlPID_position(m_bf1, Kp1, Ki, Kd)
    control2 = ControlPID_position(m_bf2, Kp2, Ki, Kd)
    control3 = ControlPID_position(m_bf3, Kp3, Ki, Kd)
    
    t = 0
    step = 0.01
    temps = [t]
    
    while t < 5:
        t += step
        temps.append(t)
        
        control1.setTarget(1)  # Position cible
        control1.simule(step)
        control2.setTarget(1)  # Position cible
        control2.simule(step)
        control3.setTarget(1)  # Position cible
        control3.simule(step)
    
    # Tracé de la réponse indicielle
    plt.figure(figsize=(12, 6))
    plt.plot(temps, m_bf1.position, label='Kp=10')
    plt.plot(temps, m_bf2.position, label='Kp=40')
    plt.plot(temps, m_bf3.position, label='Kp=300')
    plt.xlabel('Temps (s)')
    plt.ylabel('Position $x$ (m)')
    plt.title('Réponse indicielle avec un controleur en position pour différents Kp')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    #Affichage de la réponse indicielle et de la tension du deuxième controleur (à décommenter si vouloir l'affichage)
    #control2.plot()