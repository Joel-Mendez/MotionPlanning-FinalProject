#To-Do
#Parameters for Legs
#Control Rotation through ThighAngle
#MinJerk
#FootPlacement - Inverse Kinematics?
#Obstacle Generation - ranges for height and width?
#Second Leg - Problems with Overlapping?


import sys, random
import pygame
from pygame.locals import *
import pymunk
import pymunk.pygame_util
from math import pi

def deg(degree):#quick function to help me with degrees to radians
    return degree*pi/180

def generateObstacle ():
    hmin = 1 #min and max to be decided later
    hmax = 10
    wmin = 1
    wmax = 10

    h=random.randint(hmin,hmax)
    w=random.randint(wmin,wmax)

    #create Box with height h and width w
