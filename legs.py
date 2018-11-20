import sys, random
import pygame
from pygame.locals import *
import pymunk
import pymunk.pygame_util

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

def add_L(space):
    """Add a inverted L shape with two joints"""
    rotation_center_body = pymunk.Body(body_type = pymunk.Body.STATIC)
    rotation_center_body.position = (300,300)

    rotation_limit_body = pymunk.Body(body_type = pymunk.Body.STATIC)
    rotation_limit_body.position = (200,300)

    body = pymunk.Body(100, 10000)
    body.position = (300,300)
    l1 = pymunk.Segment(body, (-150, 0), (255.0, 0.0), 5.0)
    l2 = pymunk.Segment(body, (-150.0, 0), (-150.0, 50.0), 5.0)

    rotation_center_joint = pymunk.PinJoint(body, rotation_center_body, (0,0), (0,0))
    joint_limit = 25
    rotation_limit_joint = pymunk.SlideJoint(body, rotation_limit_body, (-100,0), (0,0), 0, joint_limit)

    space.add(l1, l2, body, rotation_center_joint, rotation_limit_joint)
    return l1,l2
def add_limbs(space):


    #Creating Limbs#####################################################
    ###pymunk.Body(Mass,Moment)
    thigh = pymunk.Body(10,10000)
    thigh.position=(300,550)
    shank=pymunk.Body(10,10000)
    shank.position=(300,350)
    foot=pymunk.Body(300,150)
    ###pymunk.Segment(Instance of Body, Start, End, Thickness)
    thighSegment=pymunk.Segment(thigh, (0, 0), (0, -200), 5.0)
    shankSegment=pymunk.Segment(shank, (0, 0), (0, -200), 5.0)
    #footSegment=pymunk.Segment(foot, (0,0), (50,0), 5.0)
    footSegment=pymunk.Segment(shank,(0,-200),(50,-200),5.0)

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
    shankRotation.collide_bodies=False
    #footRotation = pymunk.PinJoint(foot,shank,(0,0),(0,-200))
    space.add(thighRotation,shankRotation)

    #constraints on translational motion
    thighHipConstraint=pymunk.SlideJoint(thigh,thighJoint,(0,0),(0,0),0,0) #So our thigh doesn't pop out of its socket
    thighKneeConstraint=pymunk.SlideJoint(thigh,kneeJoint,(0,-200),(0,0),0,200) #Constraining how far our thigh can rotate
    shankKneeConstraint=pymunk.SlideJoint(shank,thigh,(0,0),(0,-200),0,0) #So that our thigh and shank remain connected at the knee
    shankKneeConstraint.collide_bodies=False
    shankAnkleConstraint=pymunk.SlideJoint(shank,ankleJoint,(0,-200),(0,0),0,200)
    space.add(thighHipConstraint,thighKneeConstraint,shankKneeConstraint,shankAnkleConstraint)


    return thigh,shank,foot

def main():
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Legs like jelly")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0.0, -900.0)

    #lines = add_L(space)
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
