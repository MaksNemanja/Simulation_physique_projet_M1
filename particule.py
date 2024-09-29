# -*- coding: utf-8 -*-

from vecteur3D import Vecteur3D
from Run_ControlPID_vitesse import *
from Run_ControlPID_position import *
from Run_moteurCC import *
from math import pi,atan2
import pygame
from pygame.locals import *
from time import time
import math

"""
Ce fichier reprend les codes faits en TP avec l'intégration du moteur dans la classe Univers
"""

class Particule(object):

    def __init__(self, mass=1, p0=Vecteur3D(), v0=Vecteur3D(), a0=Vecteur3D(), fix=False, name="paf", color='red'):
        self.mass = mass
        self.position = [p0]
        self.speed = [v0]
        self.acceleration = [a0]
        self.name = name
        self.color = color
        self.forces = Vecteur3D()
        self.fix = fix

    def __str__(self):
        msg = 'Particule ('+str(self.mass)+', '+str(self.position[-1])+', '+str(self.speed[-1])+', '+str(self.acceleration[-1])+', "'+self.name+'", "'+str(self.color)+'" )'
        return msg

    def __repr__(self):
        return str(self)

    def applyForce(self, *args):
        for f in args:
            self.forces += f

    def pfd(self, step):
        
        if not(self.fix):
            a = self.forces * (1/self.mass)
            v = self.speed[-1]+a*step
        else :
            a = Vecteur3D()
            v = Vecteur3D()

        p = self.position[-1]+0.5*a*step**2 + self.speed[-1]*step

        self.acceleration.append(a)
        self.speed.append(v)
        self.position.append(p)
        self.forces = Vecteur3D()

    def plot(self):
        from pylab import plot
        X=[]
        Y=[]
        for p in self.position:
            X.append(p.x)
            Y.append(p.y)
    
        return plot(X,Y,color=self.color,label=self.name)+plot(X[-1],Y[-1],'o',color=self.color)    

    def getPosition(self):
        return self.position[-1]
    
    def getSpeed(self):
        return self.speed[-1]
    
    def setSpeed(self, new_speed):
        self.speed[-1] = new_speed
    
    def gameDraw(self,scale,screen):
        
        W , H =  screen.get_size()     
        X = int(scale*self.getPosition().x)
        Y = int(scale*self.getPosition().y)
        Z = int(scale*self.getPosition().z)
        
        vit = self.getSpeed()
        VX = int(scale*vit.x)
        VY = int(scale*vit.y) 
        VZ = int(scale*vit.z) 
        
        size=2
        
        if type(self.color) is tuple:
            color = (self.color[0]*255,self.color[1]*255,self.color[2]*255)
        else:
            color=self.color
        Xp = X+Z*(2**0.5)/2
        Yp = (H-Y) - Z*(2**0.5)/2
        VXp = Xp + VX + VZ*(2**0.5)/2
        VYp = H - ((Y +Z*(2**0.5)/2) +(VY+VZ*(2**0.5)/2))

        pygame.draw.circle(screen,color,(Xp,Yp),size*2,size)
        
        pygame.draw.line(screen,color,(Xp,Yp),(VXp,VYp))
    
        
class Univers(object):
    
    def __init__(self,moteur=ControlPID_vitesse(), t0=0,step=0.1,name="plage",population=[],dimensions=(1024,512)):
        
        self.name=name
        self.time=[t0]
        self.population=population
        self.step=step
        
        self.dimensions=dimensions
        
        self.mouseControlled = []        
        self.keyControlled = []
        
        self.generators = []
        
        self.moteur = moteur
        self.distances = [] #liste de la distance entre le moteur et une particule au fil du temps
        self.motor_speeds = [] #liste de la vitesse du moteur au fil du temps
       
        
    def __str__(self):
        ret = "Univers (" + str(self.time[-1]) + ","  + str(self.step) + ", " + self.name + ", " + str(self.population) + ")"
        return ret
    
    def __repr__(self):
        return str(self)
    
    def addAgent(self,*args):
        for agent in args:
            self.population.append(agent)
            
        return len(self.population)
    
    def addGenerators(self,*args):
        for g in args:
            self.generators.append(g)
            
    def simulate(self):
        #Simulation du moteur
        self.moteur.simule(self.step)
        
        #Vitesse cible du moteur (par défaut, la vitesse augmente d'1 rad/s à chaque seconde,
        #à modifier si on veut imposer une vitesse particulière)
        self.moteur.setTarget(self.now)
        
        #Couple moteur
        Cm = self.moteur.moteur.getTorque()
        
        for agent in self.population:
            #Calcul de la distance entre l'agent et le centre du moteur
            vec_dir = agent.getPosition() - self.moteur.moteur.center
            distance = vec_dir.mod()
            
            #On enregistre la vitesse du moteur et la distance selon le nom de la particle qui nous intéresse
            if agent.name == "Particule 1":
                self.distances.append(distance)  
                self.motor_speeds.append(self.moteur.moteur.getSpeed())
                
            if distance != 0:
                #Calcul de la force du moteur sur l'agent
                direction = vec_dir.norm()
                moteur_force = direction * (Cm / distance) * 1000    #on rajoute un facteur 1000 pour une meilleur visualisation de la simulation
                #Rajout de la force du moteur dans les forces de l'agent
                agent.forces += moteur_force
                
            agent.pfd(self.step)
        
        self.time.append(self.time[-1] +  self.step)
        

    
    def simulateTo(self,tFin):
        while self.time[-1]<tFin:
            self.simulate()
     
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
     
    #Méthode qui va afficher la distance entre le moteur et une particule en fonction de la vitesse de rotation
    def plot_distance_vs_motor_speed(self):
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(12, 6))
        plt.plot(self.motor_speeds, self.distances)
        plt.xlabel('Vitesse du moteur (rad/s)')
        plt.ylabel('Distance (m)')
        plt.title('Distance d en fonction de la vitesse du moteur')
        plt.legend()
        plt.grid(True)
        plt.show()
        
    def gameInit(self,scale=1,fps=60,background=(0,0,0)):
        
        """- initialiser pygame
        - dessiner la fenetre
        """
        
        pygame.init()
        self.t0= time()
        
        self.clock = pygame.time.Clock()
        self.background=background
        self.fps=fps
        self.scale=scale
        self.running=True
        
        self.W = self.dimensions[0]*self.scale
        self.H = self.dimensions[1]*self.scale
        
        self.targetX = 0
        self.targetY = 0
       
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption(self.name)
        
        
        
    
    def gameUpdate(self):
               
        
        self.now = time()-self.t0
        self.gameInteraction()
        
        while self.time[-1] < self.now:
             
            for p in self.population:
                for generator in self.generators:
                    generator.setForce(p)
            
            
            self.simulate()
            
        self.screen.fill(self.background)
        
        font_obj = pygame.font.Font('freesansbold.ttf', 12)
        text_surface_obj = font_obj.render(('time: %.2f' % self.now), True, 'green', self.background)
        text_rect_obj = text_surface_obj.get_rect()
        text_rect_obj.topleft = (0, 0)
        
        self.screen.blit(text_surface_obj, text_rect_obj)

        for agent in self.population:
            agent.gameDraw(self.scale,self.screen)
            

        pygame.display.update()
        self.clock.tick(self.fps)
        
    def gameInteraction(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
        pygame.event.pump() # process event queue

        self.gameKeys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
        self.gameMouseClick = pygame.mouse.get_pressed()
        self.gameMousePos = pygame.mouse.get_pos()

        
        if self.now > self.gameEndTime or self.gameKeys[K_ESCAPE]:
            self.running=False        
        
        if self.gameKeys[ord(' ')] or self.gameKeys[pygame.K_SPACE]: # And if the key is K_DOWN:
            self.generators[0].active =not(self.generators[0].active) 
            print("PRESS")
            
        
        if self.gameMouseClick[0]:
            self.targetX = self.gameMousePos[0] / self.scale
            self.targetY = (self.H - self.gameMousePos[1]) / self.scale
            
        for generator in self.mouseControlled:
            pass
            
        for generator in self.keyControlled:
            generator.active = False
            if self.gameKeys[ord(' ')] or self.gameKeys[pygame.K_SPACE]: # And if the key is K_DOWN:
                generator.active =not(generator.active) 
                                      
            if self.gameKeys[ord('z')] or self.gameKeys[pygame.K_UP]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('s')] or self.gameKeys[pygame.K_DOWN]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('q')] or self.gameKeys[pygame.K_LEFT]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('d')] or self.gameKeys[pygame.K_RIGHT]: # And if the key is K_DOWN:
                pass
  
    def simulateRealTime(self,scale=1,fps=60,background=(0,0,0),tfin=50000000):

        self.gameEndTime = tfin
        self.gameInit(scale,fps,background)
     
        while self.running:
            self.gameUpdate()
                        
        pygame.quit()   
        
class Force(object):
    
    def __init__(self,force=Vecteur3D(),name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        
    def __str__(self):
        return "Force ("+str(self.force)+', '+self.name+")"
        
    def __repr__(self):
        return str(self)

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.force)
    
class ForceSelect(Force):
    
    def __init__(self,force=Vecteur3D(),name='force',particules=[],active=True):
        Force.__init__(self,force,name,active)
        self.particules=particules
    
    def setForce(self, particule):
        if self.active and particule in self.particules:
            particule.applyForce(self.force)

class Gravity(Force):
    def __init__(self,force=Vecteur3D(0,-9.81,0),name='gravity',active=True):
        Force.__init__(self,force,name,active)

    def setForce(self, particule):
        if self.active :
            particule.applyForce(self.force*particule.mass)

class Damping(Force):
    def __init__(self,c=0,name='damp',active=True):
        Force.__init__(self,Vecteur3D(),name,active)
        self.c=c
        
    def setForce(self, particule):
        if self.active :
            particule.applyForce(-self.c*particule.getSpeed())
        

class SpringDamper(Force):
    def __init__(self,P0,P1,k=0,c=0,l0=0,active=True,name="boing"):
        Force.__init__(self,Vecteur3D(),name,active)
        self.k = k
        self.c = c
        self.P0 = P0
        self.P1 = P1
        self.l0 = l0
    
    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        v_n = vec_dir.norm()
        flex = vec_dir.mod()-self.l0
        
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
        
        force = (self.k * flex + vit_n)* v_n
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)
        else:
            pass
        
class Link(SpringDamper):
    def __init__(self,P0,P1):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,1,0.3,l0,True,"link")

class Prismatic(SpringDamper):
    def __init__(self,P0,P1,axis=Vecteur3D(1,0,0)):
        self.axis = axis.norm()
        l = (P0.getPosition()-P1.getPosition())
        l0 = l-(l**axis) * axis
        SpringDamper.__init__(self,P0, P1,1,0.3,l0.mod(),True,"link")
        
    
    def setForce(self, particule):
        
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        vec_dir_proj = vec_dir - (vec_dir ** self.axis) * self.axis
        v_n = vec_dir_proj.norm()
        
        flex = vec_dir_proj.mod()-self.l0
        
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
               
        force = (self.k * flex + vit_n) * v_n
                
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)
        else:
            pass
        
    