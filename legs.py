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

def add_ball(space):
    """Add a ball to the given space at a random position"""
    mass = 1
    radius = 14
    inertia = pymunk.moment_for_circle(mass, 0, radius, (0,0))
    body = pymunk.Body(mass, inertia)
    x = random.randint(120,380)
    body.position = x, 550
    shape = pymunk.Circle(body, radius, (0,0))
    space.add(body, shape)
    return shape

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

def main():
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Legs like jelly")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0.0, -900.0)

    lines=add_limbs(space)

    balls = []
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    ticks_to_next_ball = 10
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)

        ticks_to_next_ball -= 1
        if ticks_to_next_ball <= 0:
            ticks_to_next_ball = 25
            ball_shape = add_ball(space)
            balls.append(ball_shape)

        screen.fill((255,255,255))

        balls_to_remove = []
        for ball in balls:
            if ball.body.position.y < 150:
                balls_to_remove.append(ball)

        for ball in balls_to_remove:
            space.remove(ball, ball.body)
            balls.remove(ball)

        space.debug_draw(draw_options)

        space.step(1/50.0)

        pygame.display.flip()
        clock.tick(50)

if __name__ == '__main__':
    main()
