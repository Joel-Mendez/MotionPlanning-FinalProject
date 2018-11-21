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

def add_limbs(space):
    #Creating Limbs#####################################################
    thigh = pymunk.Body(10,10000)
    thigh.position=(300,550)
    thigh.KINEMATIC

    shank=pymunk.Body(10,10000)
    shank.position=(300,350)
    shank.KINEMATIC

    foot=pymunk.Body(10,10000)
    foot.position=(300,150)
    foot.KINEMATIC

    ###pymunk.Segment(Instance of Body, Start, End, Thickness)
    thighSegment=pymunk.Segment(thigh, (0, 0), (0, -200), 5.0)
    shankSegment=pymunk.Segment(shank, (0, 0), (0, -200), 5.0)
    footSegment=pymunk.Segment(foot, (0,0), (50,0), 5.0)
    #footSegment=pymunk.Segment(shank,(0,-200),(50,-200),5.0)

    space.add(thigh,shank,foot,thighSegment,shankSegment,footSegment)

    #Creating Joints####################################################
    thighJoint=pymunk.Body(body_type = pymunk.Body.STATIC)
    thighJoint.position = (300,550)
    kneeJoint=pymunk.Body(body_type = pymunk.Body.STATIC)
    kneeJoint.position = (300,350)
    ankleJoint=pymunk.Body(body_type = pymunk.Body.STATIC)
    ankleJoint.position = (300,150)

    thighRotation = pymunk.PivotJoint(thigh, thighJoint, (0,0), (0,0))#Rotation of Thigh about thigh joint
    shankRotation = pymunk.PivotJoint(shank,thigh,(0,0),(0,-200)) #Rotation of shank about knee (end of thigh segment)
    shankRotation.collide_bodies=False #To prevent Parkinson's
    footRotation = pymunk.PivotJoint(foot,shank,(0,0),(0,-200)) #Rotation of foot about ankle (end of shank segment)
    footRotation.collide_bodies=False

    space.add(thighRotation)
    space.add(shankRotation)
    space.add(footRotation)

    #Constraints##########################################################
    thighHipConstraint=pymunk.RotaryLimitJoint(thigh,thighJoint,deg(-20),deg(90))
    shankKneeConstraint=pymunk.RotaryLimitJoint(thigh,shank,deg(-60),deg(0))
    shankKneeConstraint.collide_bodies=False
    footAnkleConstraint=pymunk.RotaryLimitJoint(shank,foot,deg(-20),deg(20))
    footAnkleConstraint.collide_bodies=False


    space.add(thighHipConstraint)
    space.add(shankKneeConstraint)
    space.add(footAnkleConstraint)



    return thigh,shank,foot
