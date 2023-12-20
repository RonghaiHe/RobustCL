from scipy.spatial.transform import Rotation as Rot
from math import pi, atan2, sin, cos
import numpy as np

NUM_ROBOTS = 6

DELTA_T = 0.1
Q = np.diag(np.tile([.7, .3], NUM_ROBOTS))  # coefficient of variables
Q_B = np.tile([0, 0], NUM_ROBOTS)

R_1 = np.diag([0.05, 0.05, (pi/180)])
R_0 = np.diag([0.08, (2*pi/180)])
P_INIT = np.diag([0.05**2, 0.05**2, (pi/180)**2])

R_ALL_1 = np.diag(np.tile([0.05, 0.05, (pi/180)], NUM_ROBOTS))
R_ALL_0 = np.diag(np.tile([0.08, (2*pi/180)], NUM_ROBOTS))
R_ALL_1_NT = np.diag(np.tile([0.05, 0.05], NUM_ROBOTS))  # no relative angle

P_ALL_INIT = np.diag(np.tile([0.05**2, 0.05**2, (pi/180)**2], NUM_ROBOTS))

MAESUREMENT_RANGE_BOUND = 5  # [m]
MAESUREMENT_BEARING_BOUND = pi  # ±[Rad]

E_V_ADD = np.zeros(NUM_ROBOTS)
E_OMEGA_ADD = np.zeros(NUM_ROBOTS)

V_MAX = 0.2  # [m]
V_MIN = 0  # [m]

OMEGA_MAX = 1  # 1/6 # pi/18 # [Rad]
OMEGA_MIN = -1  # -1/6 # -pi/18 # [Rad]

E_V, E_OMEGA = None, None
SIGMA_V_INPUT, SIGMA_OMEGA_INPUT = None, None
SIGMA_V2, SIGMA_OMEGA2 = np.zeros(NUM_ROBOTS), np.zeros(NUM_ROBOTS)
SIGMA0_V2, SIGMA0_OMEGA2 = None, None


def update_para():
    '''
    Update some global parameters, no use
    '''

    global E_V, E_OMEGA, SIGMA_V_INPUT, SIGMA_OMEGA_INPUT, SIGMA_V2, SIGMA_OMEGA2, SIGMA0_V2, SIGMA0_OMEGA2
    E_V = (V_MAX+V_MIN)/2 + E_V_ADD
    E_OMEGA = (OMEGA_MAX+OMEGA_MIN)/2 + E_OMEGA_ADD

    SIGMA_V_INPUT = (V_MAX-E_V)/3
    SIGMA_OMEGA_INPUT = (OMEGA_MAX-E_OMEGA)/3

    for r in range(NUM_ROBOTS):
        SIGMA_V2[r] = (SIGMA_V_INPUT[r])**2 + Q[2*r+0, 2*r+0]**2 * ((SIGMA_V_INPUT[r])**2 + E_V[r]**2) + \
            2*Q[2*r+0, 2*r+0]*Q_B[2*r+0]*E_V[r] + Q_B[2*r+0]**2
        SIGMA_OMEGA2[r] = (SIGMA_OMEGA_INPUT[r])**2 + Q[2*r+1, 2*r+1]**2 * ((SIGMA_OMEGA_INPUT[r])**2 + E_OMEGA[r]**2) + \
            2*Q[2*r+1, 2*r+1]*Q_B[2*r+1]*E_OMEGA[r] + Q_B[2*r+1]**2

    SIGMA0_V2 = SIGMA_V2 + E_V**2  # uniform distribution's variance and pick 0
    SIGMA0_OMEGA2 = SIGMA_OMEGA2 + E_OMEGA**2


update_para()


# used in [1] X. Wang, S. Sun, T. Li, and Y. Liu, “Fault Tolerant Multi-Robot Cooperative Localization Based on Covariance Union,” IEEE Robotics and Automation Letters, vol. 6, no. 4, pp. 7799–7806, Oct. 2021.
Rv = .5
MU_RANGE_BEARING = np.array([2.4, 2*pi/180])
MU_POSE = np.array([1.5, 1.5, 30*pi/180])
BIAS_SIGMA_RANGE_BEARING = np.array([0.08, 2*pi/180])
BIAS_SIGMA_POSE = np.array([0.05, 0.05, 1*pi/180])

meas_bia_prob = 0  # Ra
# comm_fail_prob = 0.1

COMM_RATE = 10

LANDMARK_POS = []  # [[11.5, 10.5, 0], [11.5, 4.5, 0], [11.5, -1.5, 0]]
LANDMARK_NUM = len(LANDMARK_POS)

R_ALL_0_LANDMARK = np.diag(np.tile([0.08, 2*(pi/180)], LANDMARK_NUM))
R_ALL_1_LANDMARK = np.diag(np.tile([0.05, 0.05, (pi/180)], LANDMARK_NUM))
R_ALL_1_NT_LANDMARK = np.diag(np.tile([0.05, 0.05], LANDMARK_NUM))


def rot_mat_2d(angle):
    '''
    return a 2D rotation matrix:
        [ cos(angle), sin(angle), 0]
        [-sin(angle), cos(angle), 0]
        [          0,          0, 1]

    :param angle: angle in rad
    :return: 2D rotation matrix
    '''
    return Rot.from_euler('z', angle).as_matrix().T


def measurement(X2, X1):
    '''
    Calculate observation variables

    :param X2: measured robot
    :param X1: measuring robot
    :returns _range, bearing, relative pose
    '''
    R1 = rot_mat_2d(X1[2])
    X2_ = np.dot(R1, X2-X1)

    return np.linalg.norm(X1[0:2]-X2[0:2]), atan2(X2[1]-X1[1], X2[0]-X1[0]) - X1[2], X2_.tolist()


def is_pos_def(P):
    '''
    Judge whether the input matrix P is positive definite

    :param P: matrix to be judges
    '''
    return np.all(np.linalg.eigvals(P) > 0)
