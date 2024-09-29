# -*- coding: utf-8 -*-

from Run_moteurCC import *
import numpy as np
import matplotlib.pyplot as plt
import pickle

"""
Ce fichier contient la définition de la classe ControlPID_vitesse
Une simulation est disponible. Elle permet d'afficher la réponse indicielle d'un moteur en BF
en imposant une vitesse cible
"""

class ControlPID_vitesse:
    def __init__(self, moteur=MoteurCC(), Kp=0, Ki=0, Kd=0):
        self.moteur = moteur  #moteur (MoteurCC)
        self.Kp = Kp #gain proportionnel
        self.Ki = Ki #gain intégral
        self.Kd = Kd #gain dérivé
        self.target_speed = 0 #vitesse cible
        self.previous_error = 0 # rreur de vitesse à l'itération précédente
        self.integral = 0 #somme accumulée des erreurs
        self.voltages = [] #liste des tensions appliquées au moteur au cours du temps
    
    def __str__(self):
        return f"ControlPID_vitesse(Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd})"
    
    def __repr__(self):
        return self.__str__()
    
    #Méthode qui permet d'imposer une vitesse cible
    def setTarget(self, vitesse):
        self.target_speed = vitesse
    
    #Méthode pour récupérer la dernière valeur de la tension
    def getVoltage(self):
        return self.voltages[-1] if self.voltages else 0
    
    #Méthode pour simuler le système
    def simule(self, dt):
        current_speed = self.moteur.getSpeed()
        
        #Calcul de l'erreur
        error = self.target_speed - current_speed
        
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
    
    #Méthode pour afficher la vitesse du moteur et la tension au cours du temps
    def plot(self):
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(self.moteur.Omega, label='Vitesse $\Omega$(t)')
        plt.xlabel('Temps (s)')
        plt.ylabel('Vitesse $\Omega$ (rad/s)')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(self.voltages, label='Tension $U_m$(t)', color='g')
        plt.xlabel('Temps (s)')
        plt.ylabel('Tension $U_m$ (V)')
        plt.legend()
        plt.grid(True)
        
    
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
    
    # Création du moteur
    m_bf = MoteurCC(R, L, kc, ke, J, f)
    
    # Paramètres du contrôleur
    Kp = 50  # gain intégral
    Ki = 500 # gain proportionnel
    Kd =  0 # gain dérivé
    control = ControlPID_vitesse(m_bf, Kp, Ki, Kd)
    
    t = 0
    step = 0.01
    temps = [t]
    
    while t < 1:
        t += step
        temps.append(t)
        control.setTarget(1)  # vitesse cible
        control.simule(step)
    
    
    # Tracé de la réponse indicielle
    plt.figure(figsize=(12, 6))
    plt.plot(temps, m_bf.Omega)
    plt.xlabel('Temps (s)')
    plt.ylabel('Vitesse $\Omega$ (rad/s)')
    plt.title('Réponse indicielle avec un controleur PID')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    #Affichage de la réponse indicielle et de la tension (à décommenter si vouloir l'affichage)
    #control.plot()
