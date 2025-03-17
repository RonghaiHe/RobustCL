import numpy as np
import parameters
from math import atan2, pi, sin, cos

DELTA_T = parameters.DELTA_T
Q = parameters.Q
Q_B = parameters.Q_B
P_INIT = parameters.P_INIT


def update_para(Q_new):
    global Q
    Q = Q_new.copy()


def reset_para():
    global Q
    Q = parameters.Q


class Robot:
    def __init__(self, X, _id, NUM_ROBOTS) -> None:
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        :param: NUM_ROBOTS: int, the number of robots
        '''

        self.X = X
        self._id = _id
        self.NUM_ROBOTS = NUM_ROBOTS
        self.P = P_INIT.copy()

        self.X_list = [self.X.tolist()]

        self.MAEP_list = []

        self.MAEO_list = []

        self.ANEES_list = []

        self.RMSE_list = []

    def motion(self, v, omega):
        '''
        time propagation update with unicycle model

        :param: v: float, linear velocity
        :param: omega: float, angular velocity

        :return: F: np.array, Jacobian matrix of state covariance
        '''

        c1 = cos(self.X[2])
        s1 = sin(self.X[2])
        input = np.array([v, v, omega])
        incre = np.array([DELTA_T * c1, DELTA_T * s1, DELTA_T])
        # state update
        self.X = self.X + input * incre

        Q_now = Q[2*self._id:2*self._id+2, 2*self._id:2*self._id+2].copy()
        Q_now[0, 0] = Q_now[0, 0]*v + Q_B[2*self._id+0]
        Q_now[1, 1] = Q_now[1, 1]*omega + Q_B[2*self._id+1]

        # Jacobian
        F = np.array([[1, 0, -v * DELTA_T * s1],
                     [0, 1, v * DELTA_T * c1], [0, 0, 1]])
        G = np.array([[DELTA_T * c1, 0], [DELTA_T * s1, 0], [0, DELTA_T]])

        # state covariance update
        self.P = F @ self.P @ F.T + G @ Q_now**2 @ G.T
        self.P_prediction = (self.P).copy()
        self.X_prediction = (self.X).copy()
        return F

    def comparison(self, cla_true):
        '''
        Metric calculation

        :param: cla_true: class of robot_true, for comparison
        '''
        self.calc_RMSE(cla_true)
        self.calc_MAEP(cla_true)
        self.calc_MAEO(cla_true)
        self.calc_ANEES(cla_true)

    def calc_RMSE(self, cla_true):
        self.RMSE = (sum((self.X[0:2] - cla_true.X_true[0:2])**2)/2)**(0.5)

    def calc_MAEP(self, cla_true):
        self.MAEP = np.linalg.norm(self.X[0:2] - cla_true.X_true[0:2], ord=1)

    def calc_MAEO(self, cla_true):
        self.MAEO = abs(self.X[2] - cla_true.X_true[2])

    def calc_ANEES(self, cla_true):
        temp0 = (cla_true.X_true-self.X)[0:2]
        self.ANEES = (temp0.T @ np.linalg.inv(self.P[0:2, 0:2]) @ temp0)

    def storage(self):
        '''
        Storage the metric results in lists
        '''
        self.X_list.append(self.X.tolist())

        self.RMSE_list.append(self.RMSE)

        self.MAEP_list.append(self.MAEP)

        self.MAEO_list.append(self.MAEO)

        self.ANEES_list.append(self.ANEES)

    def draw(self, ax, str_color, str_label):
        '''
        Draw the robot in the figure

        :param: ax: the figure
        :param: str_color: the color of the robot
        :param: str_label: the label of the robot
        '''
        if (self._id == 0):
            ax.plot(np.array(self.X_list).T[0], np.array(
                self.X_list).T[1], 'o-', markersize=1, c=str_color, label=str_label)
        else:
            ax.plot(np.array(self.X_list).T[0], np.array(
                self.X_list).T[1], 'o-', markersize=1, c=str_color)


class Robot_true:
    def __init__(self, X, _id):
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        '''
        self.X_true = X
        self._id = _id
        self.X_true_list = [self.X_true.tolist()]

    def update(self, v, omega, noise):
        '''
        robot pose update with unicycle model
        :param: v: float
            linear velocity
        :param: omega: float
            angular velocity
        :param: noise: list 1*2
                v & omega's Gaussian noise(Unknown in fact, 
                but keep the same values in different algorithms)
        '''

        c1 = cos(self.X_true[2])
        s1 = sin(self.X_true[2])
        input = np.array([v, v, omega])
        incre = np.array([DELTA_T * c1, DELTA_T * s1, DELTA_T])

        Q_now = Q.copy()
        self.v_noise = noise[0]*(Q_now[0, 0] * v + Q_B[0])
        self.omega_noise = noise[1]*(Q_now[1, 1] * omega + Q_B[1])
        input_true = input + \
            np.array([self.v_noise, self.v_noise, self.omega_noise])
        self.X_true = self.X_true + input_true * incre

    def storage(self):
        self.X_true_list.append(self.X_true.tolist())

    def plot_shape(self, ax):
        '''
        Draw the shape of the robot in the figure

        :param: ax: the figure
        '''
        x = np.array([.2, .4, .2, -.2, -.2, .2])*2
        y = np.array([.1, 0, -.1, -.1, .1, .1])*2

        # Create points as a 2xN matrix
        points = np.vstack((x, y))
        # Apply rotation directly
        rotated_points = parameters.rot_mat_2d(
            self.X_true[2])[0:2][0:2].T @ points

        # Translate to robot position
        px = rotated_points[0, :] + self.X_true[0]
        py = rotated_points[1, :] + self.X_true[1]

        ax.plot(px, py, '-', c='k')

    def draw(self, ax, str_color=None, str_label=None):
        '''
        Draw the robot in the figure

        :param: ax: the figure
        '''

        if str_color is None and str_label is None:
            if (self._id == 0):
                ax.plot(np.array(self.X_true_list).T[0], np.array(
                    self.X_true_list).T[1], 'o-', markersize=2, c='k', label='GroundTruth')
            else:
                ax.plot(np.array(self.X_true_list).T[0], np.array(
                    self.X_true_list).T[1], 'o-', markersize=2, c='k')
        else:
            ax.plot(np.array(self.X_true_list).T[0], np.array(
                self.X_true_list).T[1], 'o-', markersize=1, c=str_color, label=str_label)
