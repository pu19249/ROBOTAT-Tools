import numpy as np

def pid_controller(xi, goal):
    '''
    Using NUMPY import, PID controller is defined.

    Attributes:
    --------------
    xi: list
        State variables
    goal: list
        [x goal, y goal]

    Returns:
    ----------
    u: np.array
        It contains the linear and angular velocities calculated with this controller.
    '''
    kp = 0.10  
    ki = 0.01  
    kd = 0.1  

    x = xi[0]
    y = xi[1]
    theta = xi[2]

    e = np.array([goal[0] - x, goal[1] - y])
    thetag = np.arctan2(e[1], e[0])

    eP = np.linalg.norm(e)
    eO = thetag - theta
    eO = np.arctan2(np.sin(eO), np.cos(eO))

    v = kp * eP + ki * eP + kd * eO
    w = kp * eO + ki * eO + kd * eO

    u = np.array([v, w])

    return u
