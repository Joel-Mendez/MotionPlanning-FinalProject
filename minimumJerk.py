# Compute a point to point minimum jerk trajectory
# x0 dx0 ddx0 are the location, velocity and acceleration at the start point
# xT dxT ddxT are the target location velocity and acceleration
# T is the time required to move from the start point to the target point
# The solution is a 6-D vector of coefficients a
# The minimum jerk trajectory takes the form x_t = \sum_{k=1}^6 a_k t^(k-1), for 0\leq t \leq T
import numpy as np

def minimumJerk(x0, dx0, ddx0,xT,dxT,ddxT,T):

    T2 = T*T #T squared
    T3 = T2*T #T cubed... and so on.
    T4 = T3*T
    T5= T4*T

    a = np.zeros((6,1))
    a[0] = x0
    a[1] = dx0
    a[2] = ddx0/2
    b=np.array([ [T3, T4, T5], [3*T2, 4*T3, 5*T4], [6*T, 12*T2, 20*T3] ])
    c=np.array([ [xT-a[0]-a[1]*T-a[2]*T2] , [dxT-a[1]-2*a[2]*T], [ddxT-2*a[2]] ])
    print 'b'
    print b
    print'c'
    print c
    d=np.linalg.pinv(b)
    print 'd'
    print d
    e=np.matmul(d,c)
    print 'e'
    print e
    # a(4:6,1)=pinv(b)*c;
    a[3:6,:1]=e

    return a

a=minimumJerk(60,0,0,0,0,0,2)

print a
