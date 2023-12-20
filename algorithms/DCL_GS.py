import parameters
import numpy as np
from math import atan2, pi, sin, cos, sqrt, exp
from scipy.optimize import minimize_scalar, root_scalar
from scipy.linalg import sqrtm
from scipy.special import erf

import time

from algorithms.func_detect_fault import Maha

Q = parameters.Q
Q_B = parameters.Q_B
DELTA_T = parameters.DELTA_T
R_0 = parameters.R_0
R_1 = parameters.R_1
R_ALL_1 = parameters.R_ALL_1
R_ALL_1_NT = parameters.R_ALL_1_NT

R_ALL_0_LANDMARK = parameters.R_ALL_0_LANDMARK
R_ALL_1_LANDMARK = parameters.R_ALL_1_LANDMARK
R_ALL_1_NT_LANDMARK = parameters.R_ALL_1_NT_LANDMARK

V_MAX = parameters.V_MAX
E_V = parameters.E_V
E_OMEGA = parameters.E_OMEGA

SIGMA0_V2 = parameters.SIGMA0_V2
SIGMA_V2 = parameters.SIGMA_V2

SIGMA0_OMEGA2 = parameters.SIGMA0_OMEGA2
SIGMA_OMEGA2 = parameters.SIGMA_OMEGA2

RB = parameters.MAESUREMENT_RANGE_BOUND
BB = parameters.MAESUREMENT_BEARING_BOUND


def update_para(Q_new, sigma0_omega2_new, sigma_omega2_new):
    global Q, SIGMA0_OMEGA2, SIGMA_OMEGA2
    Q = Q_new.copy()
    SIGMA0_OMEGA2 = sigma0_omega2_new.copy()
    SIGMA_OMEGA2 = sigma_omega2_new.copy()


def reset_para():
    global Q, SIGMA0_OMEGA2, SIGMA_OMEGA2
    Q = parameters.Q
    SIGMA0_OMEGA2 = parameters.SIGMA0_OMEGA2
    SIGMA_OMEGA2 = parameters.SIGMA_OMEGA2

# My innovation!


class Robot_GS_EKF:
    def __init__(self, initial_s, _id, NUM_ROBOTS, flag, LANDMARK_POS):
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        :param: NUM_ROBOTS: int, the number of robots
        :param: flag: int
        :param: LANDMARK_POS: position of landmarks
        '''
        self.NUM_ROBOTS = NUM_ROBOTS
        self._id = _id
        self.flag = flag
        self.X_GS = np.array(initial_s, dtype='float64').reshape(
            (3*self.NUM_ROBOTS, 1))
        self.P_GS = parameters.P_ALL_INIT.copy()

        self.measuring = np.zeros(NUM_ROBOTS, dtype=bool)
        self.LANDMARK_NUM = len(LANDMARK_POS)
        self.LANDMARK_POS = np.array(
            LANDMARK_POS).reshape((3*self.LANDMARK_NUM, 1))
        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)

        if (self.flag >= 0):
            self.Z = np.zeros((2*self.NUM_ROBOTS, 1))
            self.Z_true = np.zeros((2*self.NUM_ROBOTS, 1))
            self.Z_landmark = np.zeros((2*self.LANDMARK_NUM, 1))
        elif (self.flag == -1):
            self.Z = np.zeros((3*self.NUM_ROBOTS, 1))
            self.Z_true = np.zeros((3*self.NUM_ROBOTS, 1))
            self.Z_landmark = np.zeros((3*self.LANDMARK_NUM, 1))

        self.X_list = [self.X_GS[3*self._id:3*self._id+3].reshape(3)]
        self.MAEP_list = []
        self.MAEO_list = []
        self.ANEES_list = []
        self.RMSE_list = []

        self.contain_bias_abso = np.zeros(self.LANDMARK_NUM, dtype=bool)
        self.contain_bias_rela = np.zeros(NUM_ROBOTS, dtype=bool)

    def reset_abso(self):
        '''
        Reset variables about the relative measurement(other robots)
        '''
        if (self.flag >= 0):
            self.Z_landmark = np.zeros((2*self.LANDMARK_NUM, 1))
        elif (self.flag == -1):
            self.Z_landmark = np.zeros((3*self.LANDMARK_NUM, 1))
        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)

        self.contain_bias_abso = np.zeros(self.LANDMARK_NUM, dtype=bool)

    def reset_rela(self):
        '''
        Reset variables about the absolute measurement(landmark)
        '''
        if (self.flag >= 0):
            self.Z = np.zeros((2*self.NUM_ROBOTS, 1))
            self.Z_true = np.zeros((2*self.NUM_ROBOTS, 1))
        elif (self.flag == -1):
            self.Z = np.zeros((3*self.NUM_ROBOTS, 1))
            self.Z_true = np.zeros((3*self.NUM_ROBOTS, 1))
        self.measuring = np.zeros(self.NUM_ROBOTS, dtype=bool)

        self.contain_bias_rela = np.zeros(self.NUM_ROBOTS, dtype=bool)

    def motion(self, v, omega):
        '''
        time propagation update with unicycle model

        :param: v: float, linear velocity
        :param: omega: float, angular velocity
        '''
        Q_ = np.zeros((2*self.NUM_ROBOTS, 2*self.NUM_ROBOTS))
        F_ = np.zeros((3*self.NUM_ROBOTS, 3*self.NUM_ROBOTS))
        G_ = np.zeros((3*self.NUM_ROBOTS, 2*self.NUM_ROBOTS))
        for r in range(self.NUM_ROBOTS):
            c1 = cos(self.X_GS[3*r+2])
            s1 = sin(self.X_GS[3*r+2])
            incre = np.array([DELTA_T * c1, DELTA_T * s1, DELTA_T])

            if r == self._id:
                input = np.array([v, v, omega])
            else:
                input = np.array([E_V[r], E_V[r], E_OMEGA[r]])
            self.X_GS[3*r:3*r+3] += (input * incre).reshape(-1, 1)

            Q_now = Q[2*self._id:2*self._id+2, 2*self._id:2*self._id+2].copy()
            if r == self._id:
                Q_now[0, 0] = (Q_now[0, 0]*v + Q_B[2*self._id+0])**2
                Q_now[1, 1] = (Q_now[1, 1]*omega + Q_B[2*self._id+1])**2
                F = np.array([[1, 0, -v * DELTA_T * s1],
                             [0, 1, v * DELTA_T * c1], [0, 0, 1]])
            else:
                Q_now[0, 0] = SIGMA_V2[r]
                Q_now[1, 1] = SIGMA_OMEGA2[r]
                F = np.array([[1, 0, -E_V[r] * DELTA_T * s1],
                             [0, 1, E_V[r] * DELTA_T * c1], [0, 0, 1]])
            G = np.array([[DELTA_T * c1, 0], [DELTA_T * s1, 0], [0, DELTA_T]])

            Q_[2*r:2*r+2, 2*r:2*r+2] = Q_now.copy()
            F_[3*r:3*r+3, 3*r:3*r+3] = F.copy()
            G_[3*r:3*r+3, 2*r:2*r+2] = G.copy()

        self.P_GS = F_ @ self.P_GS @ F_.T + G_ @ Q_ @ G_.T
        self.X_GS_prediction = self.X_GS.copy()
        self.P_GS_prediction = self.P_GS.copy()

    def measurement_abso(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all the absolute measurement(landmark)

        :param: cla_true: list of class "Robot_true"
            All GroundTruth about robots
        :param: measure_noises: list of measure_noises
            Let each algorithm obtain the same observation values
        :param: measure_bias_whether: list of whether the measurement is biased
        '''

        That_true = cla_trues[self._id].X_true.copy()

        # landmark
        for LM_id in range(self.LANDMARK_NUM):
            _range, bearing, X2_ = parameters.measurement(
                self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0], That_true)
            if _range <= RB and abs(bearing) <= BB:  # self->LM_id
                self.measuring_landmark[LM_id] = True

                if measure_bias_whether[LM_id]:
                    self.contain_bias_abso[LM_id] = True

                if (self.flag >= 0):
                    self.measure_noise = measure_noises.copy()
                    self.Z_landmark[2*LM_id, 0] = _range + \
                        measure_noises[LM_id, 0]
                    self.Z_landmark[2*LM_id+1, 0] = bearing + \
                        measure_noises[LM_id, 1]
                elif (self.flag == -1):
                    self.Z_landmark[3*LM_id:3*LM_id +
                                    3] = (X2_ + measure_noises[LM_id, :]).reshape(-1, 1)

    def measurement_rela(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all the relative measurements(other robots)

        :param: cla_true: list of class "Robot_true"
            All GroundTruth about robots
        :param: measure_noises: list of measure_noises
            Let each algorithm obtain the same observation values
        :param: measure_bias_whether: list of whether the measurement is biased
        '''
        That_true = cla_trues[self._id].X_true.copy()

        # other robots
        for r in range(self.NUM_ROBOTS):
            if (r == self._id):
                continue
            _range, bearing, X2_ = parameters.measurement(
                cla_trues[r].X_true, That_true)
            if _range <= RB and abs(bearing) <= BB:  # self->r
                self.measuring[r] = True
                if measure_bias_whether[r]:
                    self.contain_bias_rela[r] = True
                self.measure_noise = measure_noises.copy()
                if (self.flag >= 0):
                    self.Z[2*r, 0] = _range + measure_noises[r, 0]
                    self.Z[2*r+1, 0] = bearing + measure_noises[r, 1]
                elif (self.flag == -1):
                    self.Z[3*r:3*r+3] = (X2_ +
                                         measure_noises[r, :]).reshape(-1, 1)

    def abso_meas_correct(self, count=500):
        '''
        Absolute Measurement update
        '''
        for LM_id in range(self.LANDMARK_NUM):
            if not self.measuring_landmark[LM_id]:
                continue

            if (self.flag >= 0):
                H = np.zeros((2, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((2, 1))
                Z_now = self.Z_landmark[2*LM_id:2*LM_id+2]
            elif (self.flag == -1):
                H = np.zeros((3, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((3, 1))
                Z_now = self.Z_landmark[3*LM_id:3*LM_id+3]

            # rho2 = dp[0]**2 + dp[1]**2

            if (self.flag == 0):

                dp = self.LANDMARK_POS[3*LM_id:3*LM_id +
                                       2] - self.X_GS[3*self._id:3*self._id+2]
                rho = np.linalg.norm(dp, ord=2)
                rho2 = rho**2

                H[0, 3*self._id] = -dp[0]/rho
                H[0, 3*self._id+1] = -dp[1]/rho

                H[1, 3*self._id] = dp[1]/rho2
                H[1, 3*self._id+1] = dp[0]/rho2
                H[1, 3*self._id+2] = -1

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]

                R2 = (R_ALL_0_LANDMARK[2*LM_id:2 *
                      LM_id+2, 2*LM_id:2*LM_id+2])**2

            elif (self.flag == 1):
                R = R_ALL_0_LANDMARK[2*LM_id:2 *
                                     LM_id+2, 2*LM_id:2*LM_id+2].copy()
                gamma = parameters.rot_mat_2d(
                    self.X_GS[3*self._id+2, 0])[0:2, 0:2]
                J = np.array([[0, -1], [1, 0]])
                dp = self.LANDMARK_POS[3*LM_id:3*LM_id +
                                       2] - self.X_GS[3*self._id:3*self._id+2]

                rho, alpha = Z_now[0, 0], Z_now[1, 0]

                alpha_cal = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]

                R[1, 1] = R[1, 1] * rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                Z_now[0, 0] = rho * cos(alpha)
                Z_now[1, 0] = rho * sin(alpha)

                H_tilde = np.hstack((np.eye(2), J @ dp))

                H[:, 3*self._id:3*self._id+3] = gamma @ -H_tilde

                Z_cal = gamma @ dp

            elif (self.flag == -1):
                gamma = parameters.rot_mat_2d(self.X_GS[3*self._id+2, 0])
                J = np.array([[0, -1], [1, 0]])
                dp = self.LANDMARK_POS[3*LM_id:3*LM_id +
                                       3] - self.X_GS[3*self._id:3*self._id+3]
                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[0:2]).reshape(2,)

                H[:, 3*self._id:3*self._id+3] = gamma @ -H_tilde

                Z_cal = gamma @ dp

                R2 = (R_ALL_1_LANDMARK[3*LM_id:3 *
                      LM_id+3, 3*LM_id:3*LM_id+3])**2

            v = Z_now - Z_cal
            if (self.flag == 0 and abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi

            sigma_invention = H @ self.P_GS @ H.T + R2

            sigma = None
            try:
                sigma = np.linalg.inv(np.array(sigma_invention))
            except np.linalg.LinAlgError:
                continue

            K = self.P_GS @ H.T @ sigma

            self.X_GS = self.X_GS + K @ v
            self.P_GS = (np.eye(3*self.NUM_ROBOTS) - K @ H) @ self.P_GS

        self.X_GS_prediction = self.X_GS.copy()
        self.P_GS_prediction = self.P_GS.copy()

    def rela_meas_correct(self, count, cla_true=[]):
        '''
        Relative Measurement update
        '''
        for r in range(self.NUM_ROBOTS):
            if not self.measuring[r]:
                continue

            before_X = self.X_GS[3*self._id:3*self._id+3, 0].copy()
            before_r = self.X_GS[3*r:3*r+3, 0].copy()

            if (self.flag >= 0):
                H = np.zeros((2, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((2, 1))
                Z_now = self.Z[2*r:2*r+2]
            elif (self.flag == -1):
                H = np.zeros((3, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((3, 1))
                Z_now = self.Z[3*r:3*r+3]

            if (self.flag == 0):
                dp = self.X_GS[3*r:3*r+2] - self.X_GS[3*self._id:3*self._id+2]
                rho = np.linalg.norm(dp, ord=2)
                rho2 = rho**2

                H[0, 3*self._id] = -dp[0]/rho
                H[0, 3*self._id+1] = -dp[1]/rho
                H[0, 3*r] = -H[0, 3*self._id]
                H[0, 3*r+1] = -H[0, 3*self._id+1]

                H[1, 3*self._id] = dp[1]/rho2
                H[1, 3*self._id+1] = -dp[0]/rho2
                H[1, 3*self._id+2] = -1
                H[1, 3*r] = -H[1, 3*self._id]
                H[1, 3*r+1] = -H[1, 3*self._id+1]

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]
                R2 = (R_0)**2

            elif (self.flag == 1):
                R = R_0.copy()
                gamma = parameters.rot_mat_2d(
                    self.X_GS[3*self._id+2, 0])[0:2, 0:2]
                J = np.array([[0, -1], [1, 0]])
                dp = self.X_GS[3*r:3*r+2] - self.X_GS[3*self._id:3*self._id+2]

                rho, alpha = Z_now[0, 0], Z_now[1, 0]

                alpha_cal = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]
                if (abs(alpha - alpha_cal) > 4):
                    if alpha - alpha_cal > 0:
                        alpha = alpha - 2*pi
                    else:
                        alpha = alpha + 2*pi

                R[1, 1] = R[1, 1] * rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                Z_now[0, 0] = rho * cos(alpha)
                Z_now[1, 0] = rho * sin(alpha)

                H_tilde = np.hstack((np.eye(2), J @ dp))

                H[:, 3*self._id:3*self._id+3] = gamma @ -H_tilde
                H[:, 3*r:3*r+2] = gamma

                Z_cal = gamma @ dp

            elif (self.flag == -1):
                gamma = parameters.rot_mat_2d(self.X_GS[3*self._id+2, 0])
                J = np.array([[0, -1], [1, 0]])

                dp = self.X_GS[3*r:3*r+3] - self.X_GS[3*self._id:3*self._id+3]
                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[0:2]).reshape(2,)

                H[:, 3*self._id:3*self._id+3] = gamma @ -H_tilde
                H[:, 3*r:3*r+3] = gamma

                Z_cal = gamma @ dp
                R2 = (R_ALL_1[:3, :3])**2

            v = Z_now - Z_cal
            if (self.flag == 0 and abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi

            sigma_invention = H @ self.P_GS @ H.T + R2

            sigma = None
            try:
                sigma = np.linalg.inv(np.array(sigma_invention))
            except np.linalg.LinAlgError:
                continue

            K = self.P_GS @ H.T @ sigma
            X = self.X_GS.copy()
            self.X_GS = self.X_GS + K @ v
            self.P_GS = (np.eye(3*self.NUM_ROBOTS) - K @ H) @ self.P_GS

        self.X_GS_prediction = self.X_GS.copy()
        self.P_GS_prediction = self.P_GS.copy()

    def communicate1_CI(self, Xi_GS, Pi_GS, _id):
        '''
        Communication update

        :param: Xi_GS, np.array, [3, 1], robot_i's position
        :param: Pi_GS, np.array, [3, 3], robot_i's covariance
        :param: _id, int, robot_i's id
        '''
        eps = 1e-5

        inv_P, inv_P_i = None, None
        try:
            inv_P_i = np.linalg.inv(Pi_GS)
            inv_P = np.linalg.inv(self.P_GS)
        except np.linalg.LinAlgError:
            return 0

        # condition_number1 = np.linalg.cond(Pi_GS)
        # condition_number2 = np.linalg.cond(self.P_GS)

        # product1 = np.matmul(inv_P, self.P_GS)
        # is_identity1 = np.allclose(product1, np.eye(18))

        # product2 = np.matmul(inv_P_i, Pi_GS)
        # is_identity2 = np.allclose(product2, np.eye(18))

        # X_j, P_j should be sent to robot_j from robot_i
        def fitness(w):
            try:
                f = np.linalg.inv(w * inv_P + (1-w) * inv_P_i)
                return np.trace(f)
            except np.linalg.LinAlgError:
                return 100

        if (np.trace(abs(Pi_GS)) < eps):
            w = 1-eps
        elif (np.trace(abs(self.P_GS)) < eps):
            w = eps
        else:
            res = minimize_scalar(fitness, bounds=(0, 1), method='bounded')
            w = res.x

        temp = w * inv_P + (1-w) * inv_P_i
        if np.isinf(temp).any() or np.isnan(temp).any():
            return
        condition_number = np.linalg.cond(temp)
        if condition_number > 1e4:
            return 0

        self.P_GS = np.linalg.inv(w * inv_P + (1-w) * inv_P_i)
        self.X_GS = self.P_GS @ (w*inv_P @ self.X_GS + (1-w)*inv_P_i @ Xi_GS)

        return 1

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
        self.RMSE = (
            sum((self.X_GS[3*self._id:3*self._id+2, 0] - cla_true.X_true[0:2])**2)/2)**(0.5)

    def calc_MAEP(self, cla_true):
        self.MAEP = np.linalg.norm(np.squeeze(
            np.array(self.X_GS[3*self._id:3*self._id+2, 0])) - cla_true.X_true[0:2], ord=1)

    def calc_MAEO(self, cla_true):
        self.MAEO = abs(self.X_GS[3*self._id+2, 0] - cla_true.X_true[2])

    def calc_ANEES(self, cla_true):
        temp0 = (cla_true.X_true[0:2]-self.X_GS[3*self._id:3*self._id+2, 0])
        inv_P = None
        try:
            inv_P = np.linalg.inv(
                self.P_GS[3*self._id:3*self._id+2, 3*self._id:3*self._id+2])
            self.ANEES = (temp0.T @ inv_P @ temp0)
        except np.linalg.LinAlgError:
            self.ANEES = 100

    def storage(self):
        '''
        Storage the metric results in lists
        '''
        self.X_list.append(self.X_GS[3*self._id:3*self._id+3].reshape(3))

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


class Robot_GS_LRHKF(Robot_GS_EKF):

    def __init__(self, initial_s, _id, NUM_ROBOTS, flag, LANDMARK_POS):
        super().__init__(initial_s, _id, NUM_ROBOTS, flag, LANDMARK_POS)

        self.psi_y_time = []
        self.psi_y_all = []

        # [1] C. D. Karlgaard, “Nonlinear Regression Huber–Kalman Filtering and Fixed-Interval Smoothing,” Journal of Guidance, Control, and Dynamics, vol.38, no. 2, pp. 322–330, 2015
        self.gamma_thre = 1.345

    def psi(self, zeta_i):
        '''
        psi(zeta_i): Huber's derived function / zeta_i

        :param: zeta_i: The components of the regression equation
        :return: The value of psi(zeta_i)
        '''

        if abs(zeta_i) <= self.gamma_thre:
            return 1
        return self.gamma_thre / abs(zeta_i)

    def abso_meas_correct(self, count=500):
        '''
        Absolute Measurement update

        :param: count: simulation round, for showing the weight of the M-estimation
        '''

        # For finding real biased meas, EKF in order without waste
        for LM_id in range(self.LANDMARK_NUM):
            if not self.measuring_landmark[LM_id]:
                continue

            X_pre = self.X_GS.copy()
            P_pre = self.P_GS.copy()
            if not parameters.is_pos_def(P_pre):
                return

            if (self.flag >= 0):
                H = np.zeros((2, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((2, 1))
                Z_now = self.Z_landmark[2*LM_id:2*LM_id+2]
            elif (self.flag == -1):
                H = np.zeros((3, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((3, 1))
                Z_now = self.Z_landmark[3*LM_id:3*LM_id+3]

            gamma = parameters.rot_mat_2d(self.X_GS[3*self._id+2, 0])
            dp = self.LANDMARK_POS[3*LM_id:3*LM_id+3] - \
                self.X_GS[3*self._id:3*self._id+3]
            if (self.flag == 0):
                rho = np.linalg.norm(dp[:2,], ord=2)
                rho2 = rho**2

                H[0, 3*self._id] = -dp[0]/rho2
                H[0, 3*self._id+1] = -dp[1]/rho2

                H[1, 3*self._id] = dp[1]/rho
                H[1, 3*self._id+1] = dp[0]/rho
                H[1, 3*self._id+2] = -1

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]

                R2 = (R_ALL_0_LANDMARK[2*LM_id:2 *
                      LM_id+2, 2*LM_id:2*LM_id+2])**2
            elif (self.flag == 1):
                R = R_ALL_0_LANDMARK[2*LM_id:2 *
                                     LM_id+2, 2*LM_id:2*LM_id+2].copy()
                gamma_ = gamma[0:2, 0:2]
                J = np.array([[0, -1], [1, 0]])

                rho, alpha = Z_now[0, 0], Z_now[1, 0]

                alpha_cal = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]
                if (abs(alpha - alpha_cal) > 4):
                    if alpha - alpha_cal > 0:
                        alpha = alpha - 2*pi
                    else:
                        alpha = alpha + 2*pi

                R[1, 1] = R[1, 1] * rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                Z_now[0, 0] = rho * cos(alpha)
                Z_now[1, 0] = rho * sin(alpha)

                H_tilde = np.hstack((np.eye(2), J @ dp[:2]))

                H[:, 3*self._id:3*self._id+3] = gamma_ @ -H_tilde

                Z_cal[0:2, :] = gamma_ @ dp[:2]

            elif (self.flag == -1):
                J = np.array([[0, -1], [1, 0]])
                # dp = self.LANDMARK_POS[3*LM_id:3*LM_id+3] - self.X_GS[3*self._id:3*self._id+3]
                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[0:2]).reshape(2,)

                H[:3, 3*self._id:3*self._id+3] = gamma @ -H_tilde
                # H[3*_id:3*_id+3, 3*_id:3*_id+3] = gamma

                Z_cal[:3, :] = gamma @ dp

                R2 = (R_ALL_1_LANDMARK[3*LM_id:3 *
                      LM_id+3, 3*LM_id:3*LM_id+3])**2

            v = Z_now - Z_cal
            if (self.flag == 0) and (abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi

            # A matrix S contains 2 matrixs: P_prediction and R at the diagonal of that matrix
            if (self.flag >= 0):
                S = np.zeros((3*self.NUM_ROBOTS + 2, 3*self.NUM_ROBOTS + 2))
                S[:2, :2] = R2.copy()
                Psi = np.eye(3*self.NUM_ROBOTS + 2)
            elif (self.flag == -1):
                S = np.zeros((3*self.NUM_ROBOTS + 3, 3*self.NUM_ROBOTS + 3))
                S[:3, :3] = R2.copy()
                Psi = np.eye(3*self.NUM_ROBOTS + 3)

            S[-3*self.NUM_ROBOTS:, -3*self.NUM_ROBOTS:] = P_pre.copy()
            S_ = sqrtm(np.linalg.inv(S)).real

            P_pre_ = sqrtm(P_pre).real
            R_ = sqrtm(R2).real

            z = S_ @ np.vstack((v + H @ X_pre, X_pre))
            M = S_ @ np.vstack((H, np.eye(3*self.NUM_ROBOTS)))
            iter_M = 100

            X_all = self.X_GS.copy()
            X_past = np.zeros((3*self.NUM_ROBOTS, 1))
            Psi_x_I = None
            Psi_y_I = None
            temp_I = None
            total_no = False
            for iter_M_count in range(iter_M):
                if (np.linalg.norm(X_all - X_past) < 1e-5):
                    break
                X_past = X_all.copy()

                if iter_M_count:
                    Zeta = z - M @ X_all
                    Zeta = [self.psi(eles[0]) for eles in Zeta]
                    Psi = np.diag(Zeta)
                if (self.flag >= 0):
                    Psi_y = Psi[:2, :2]
                elif (self.flag == -1):
                    Psi_y = Psi[:3, :3]

                Psi_x = Psi[-3*self.NUM_ROBOTS:, -3*self.NUM_ROBOTS:]

                try:
                    Psi_x_I = np.linalg.inv(Psi_x)
                    Psi_y_I = np.linalg.inv(Psi_y)
                    temp_I = np.linalg.inv(
                        H @ P_pre_ @ Psi_x_I @ P_pre_ @ H.T + R_ @ Psi_y_I @ R_)
                except np.linalg.LinAlgError:
                    if not iter_M_count:
                        total_no = True
                    break

                K = P_pre_ @ Psi_x_I @ P_pre_ @ H.T @ temp_I

                X_all = X_pre + K @ v

            if total_no:
                continue
            if count <= 500 or self.contain_bias_abso[LM_id]:
                self.psi_y_all.append(np.diag(Psi_y))

            self.X_GS = np.array(X_all)
            self.P_GS = (np.eye(3*self.NUM_ROBOTS) - K @
                         H) @ P_pre_ @ Psi_x_I @ P_pre_

    def rela_meas_correct(self, count=500):
        '''
        Absolute Measurement update

        :param: count: simulation round, for showing the weight of the M-estimation
        '''

        for r in range(self.NUM_ROBOTS):
            if not self.measuring[r]:
                continue

            X_pre = self.X_GS.copy()
            P_pre = self.P_GS.copy()
            if not parameters.is_pos_def(P_pre):
                return

            if (self.flag >= 0):
                H = np.zeros((2, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((2, 1))
                Z_now = np.zeros((2, 1))
            elif (self.flag == -1):
                H = np.zeros((3, 3*self.NUM_ROBOTS))
                Z_cal = np.zeros((3, 1))
                Z_now = np.zeros((3, 1))

            gamma = parameters.rot_mat_2d(self.X_GS[3*self._id+2, 0])
            dp = self.X_GS[3*r:3*r+3] - self.X_GS[3*self._id:3*self._id+3]
            if (self.flag == 0):
                rho = np.linalg.norm(dp[:2, :], ord=2)
                rho2 = rho**2

                H[0, 3*self._id] = -dp[0]/rho
                H[0, 3*self._id+1] = -dp[1]/rho
                H[0, 3*r] = -H[0, 3*self._id]
                H[0, 3*r+1] = -H[0, 3*self._id+1]

                H[1, 3*self._id] = dp[1]/rho2
                H[1, 3*self._id+1] = -dp[0]/rho2
                H[1, 3*self._id+2] = -1
                H[1, 3*r] = -H[1, 3*self._id]
                H[1, 3*r+1] = -H[1, 3*self._id+1]

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]
                Z_now = self.Z[2*r:2*r+2, :]

                R2 = R_0**2

            elif (self.flag == 1):
                R = R_0.copy()
                gamma_ = gamma[0:2, 0:2]
                J = np.array([[0, -1], [1, 0]])
                rho, alpha = self.Z[2*r, 0], self.Z[2*r+1, 0]

                alpha_cal = atan2(dp[1], dp[0]) - self.X_GS[3*self._id+2, 0]
                if (abs(alpha - alpha_cal) > 4):
                    if alpha - alpha_cal > 0:
                        alpha = alpha - 2*pi
                    else:
                        alpha = alpha + 2*pi

                R[1, 1] = R[1, 1] * rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                Z_now[0, 0] = rho * cos(alpha)
                Z_now[1, 0] = rho * sin(alpha)

                # dp = self.X_GS[3*r:3*r+2] - self.X_GS[3*self._id:3*self._id+2]

                H_tilde = np.hstack((np.eye(2), J @ dp[:2]))

                H[:, 3*self._id:3*self._id+3] = gamma_ @ -H_tilde
                H[:, 3*r:3*r+2] = gamma_

                Z_cal[0:2, :] = gamma_ @ dp[:2]

            elif (self.flag == -1):
                J = np.array([[0, -1], [1, 0]])

                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[0:2]).reshape(2,)

                H[0:3, 3*self._id:3*self._id+3] = gamma @ -H_tilde
                H[0:3, 3*r:3*r+3] = gamma

                Z_cal[0:3, :] = gamma @ dp

                Z_now[0:3, :] = self.Z[3*r:3*r+3, :]

                R2 = R_1**2

            v = Z_now - Z_cal
            if (self.flag == 0) and (abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi

            # A matrix S contains 2 matrixs: P_prediction and R at the diagonal of that matrix
            if (self.flag >= 0):
                S = np.zeros((3*self.NUM_ROBOTS + 2, 3*self.NUM_ROBOTS + 2))
                S[:2, :2] = R2.copy()
                Psi = np.eye(3*self.NUM_ROBOTS + 2)
            elif (self.flag == -1):
                S = np.zeros((3*self.NUM_ROBOTS + 3, 3*self.NUM_ROBOTS + 3))
                S[:3, :3] = R2.copy()
                Psi = np.eye(3*self.NUM_ROBOTS + 3)

            S[-3*self.NUM_ROBOTS:, -3*self.NUM_ROBOTS:] = P_pre.copy()
            S_ = sqrtm(np.linalg.inv(S)).real

            P_pre_ = sqrtm(P_pre).real
            R_ = sqrtm(R2).real

            z = S_ @ np.vstack((v + H @ X_pre, X_pre))
            M = S_ @ np.vstack((H, np.eye(3*self.NUM_ROBOTS)))
            iter_M = 100

            X_all = self.X_GS.copy()
            X_past = np.zeros((3*self.NUM_ROBOTS, 1))
            Psi_x_I = None
            Psi_y_I = None
            temp_I = None
            total_no = False

            # iteration until convergence
            for iter_M_count in range(iter_M):
                if (np.linalg.norm(X_all - X_past) < 1e-5):
                    break
                X_past = X_all.copy()

                if iter_M_count:
                    Zeta = z - M @ X_all
                    Zeta = [self.psi(eles[0]) for eles in Zeta]
                    Psi = np.diag(Zeta)
                if (self.flag >= 0):
                    Psi_y = Psi[:2, :2]
                elif (self.flag == -1):
                    Psi_y = Psi[:3, :3]

                Psi_x = Psi[-3*self.NUM_ROBOTS:, -3*self.NUM_ROBOTS:]

                try:
                    Psi_x_I = np.linalg.inv(Psi_x)
                    Psi_y_I = np.linalg.inv(Psi_y)
                    temp_I = np.linalg.inv(
                        H @ P_pre_ @ Psi_x_I @ P_pre_ @ H.T + R_ @ Psi_y_I @ R_)
                except np.linalg.LinAlgError:
                    if not iter_M_count:
                        total_no = True
                    break

                K = P_pre_ @ Psi_x_I @ P_pre_ @ H.T @ temp_I

                X_all = X_pre + K @ v

            if total_no:
                continue

            # mission 2 only, v1.0.0 original division
            # if count <= 500 or self.contain_bias_rela[r]:

            # mission 2 only, v1.1.0 original division
            if 3*count < 1000 or self.contain_bias_rela[r] or 3*count >= 2000:
                self.psi_y_all.append(np.diag(Psi_y))  # For output

            self.X_GS = np.array(X_all)
            self.P_GS = (np.eye(3*self.NUM_ROBOTS) - K @
                         H) @ P_pre_ @ Psi_x_I @ P_pre_

        if len(self.psi_y_all):
            self.psi_y_time.append(np.mean(self.psi_y_all, axis=0))
            self.psi_y_all = []
        else:
            self.psi_y_time.append(np.full((3,), np.nan))
