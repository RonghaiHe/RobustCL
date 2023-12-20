import numpy as np
from math import atan2, pi, sin, cos
import parameters
from algorithms.DR import Robot

from scipy.optimize import minimize_scalar
from scipy.linalg import sqrtm
from scipy.linalg import block_diag

import cvxpy as cp

R_0 = parameters.R_0
R_1 = parameters.R_1
R_ALL_0_LANDMARK = parameters.R_ALL_0_LANDMARK
R_ALL_1_LANDMARK = parameters.R_ALL_1_LANDMARK

RB = parameters.MAESUREMENT_RANGE_BOUND
BB = parameters.MAESUREMENT_BEARING_BOUND


class Robot_DMV(Robot):
    def __init__(self, X, _id, NUM_ROBOTS, flag, LANDMARK_POS) -> None:
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        :param: NUM_ROBOTS: int, the number of robots
        :param: flag: int
        :param: LANDMARK_POS: position of landmarks
        '''

        Robot.__init__(self, X, _id, NUM_ROBOTS)
        self.flag = flag

        self.LANDMARK_NUM = len(LANDMARK_POS)
        self.LANDMARK_POS = np.array(
            LANDMARK_POS).reshape((3*self.LANDMARK_NUM, 1))
        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)

        if (flag >= 0):
            self.Z = np.zeros((NUM_ROBOTS, 2))
            self.Z_true = np.zeros((NUM_ROBOTS, 2))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 2))
        elif (flag == -1):
            self.Z = np.zeros((NUM_ROBOTS, 3))
            self.Z_true = np.zeros((NUM_ROBOTS, 3))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 3))

        self.Cov = np.zeros((3, 3))
        self.Cov_id = -1  # index for last communication, -1 means init or landmark
        self.bEKF = False  # bool, shall EKF?
        self.move_update = np.eye(3)
        self.meas_info_kind = -1
        self.sort_pair_request = []
        self.pair_history = np.zeros(NUM_ROBOTS, dtype=np.uint16)

        self.measuring = np.zeros(NUM_ROBOTS, dtype=bool)
        self.measured = np.zeros(NUM_ROBOTS)
        self.communicate1_send = np.zeros(NUM_ROBOTS)
        self.communicate1_recv = np.zeros(NUM_ROBOTS)

    def reset_rela(self):
        '''
        Reset variables about the relative measurement(other robots)
        '''

        self.bEKF = False
        self.meas_info_kind = -1
        self.sort_pair_request = []
        self.measuring = np.zeros(self.NUM_ROBOTS, dtype=bool)
        self.measured = np.zeros(self.NUM_ROBOTS)

        self.X_j, self.P_j = None, None

        self.communicate1_send = np.zeros(self.NUM_ROBOTS)
        self.communicate1_recv = np.zeros(self.NUM_ROBOTS)

        if (self.flag >= 0):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 2))
            self.Z = np.zeros((self.NUM_ROBOTS, 2))
            self.Z_true = np.zeros((self.NUM_ROBOTS, 2))
        elif (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z = np.zeros((self.NUM_ROBOTS, 3))
            self.Z_true = np.zeros((self.NUM_ROBOTS, 3))

    def reset_abso(self):
        '''
        Reset variables about the absolute measurement(landmark)
        '''
        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)

        if (self.flag >= 0):
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 2))
        elif (self.flag == -1):
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 3))

    def motion(self, v, omega):
        '''
        time propagation update with unicycle model

        :param: v: float, linear velocity
        :param: omega: float, angular velocity
        '''
        F = Robot.motion(self, v, omega)
        self.move_update = F @ self.move_update

    def sort_by_his(self, elem):
        '''
        sort the pair history according to the history of the robot

        :param: elem: int, the index of the robot

        :return: int, the index of the robot
        '''

        return self.pair_history[elem]

    def measurement_abso(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all the absolute measurement(landmark)

        :param: cla_true: list of class "Robot_true"
            All GroundTruth about robots
        :param: measure_noises: list of measure_noises
            Let each algorithm obtain the same observation values
        '''

        My_true = cla_trues[self._id].X_true.copy()
        # landmark
        for LM_id in range(self.LANDMARK_NUM):
            _range, bearing, X2_ = parameters.measurement(
                self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0], My_true)
            if _range <= RB and abs(bearing) <= BB:  # self->LM_id
                self.measuring_landmark[LM_id] = True

                if (self.flag >= 0):
                    # self.measure_noise[LM_id] = measure_noises[LM_id,0]* R_0[0,0]
                    self.measure_noise[LM_id, :] = measure_noises[LM_id, 0:2]
                    self.Z_landmark[LM_id, 0] = _range + \
                        self.measure_noise[LM_id, 0]
                    self.Z_landmark[LM_id, 1] = bearing + \
                        self.measure_noise[LM_id, 1]
                elif (self.flag == -1):
                    # self.measure_noise[LM_id] = measure_noises[LM_id,:]*np.array([R_1[0,0], R_1[1,1], R_1[2,2]])
                    self.measure_noise[LM_id] = measure_noises[LM_id, :]
                    self.Z_landmark[LM_id] = X2_ + measure_noises[LM_id, :]

    def measurement_rela(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all the relative measurements(other robots)

        :param: cla_true: list of class "Robot_true"
            All GroundTruth about robots
        :param: measure_noises: list of measure_noises
            Let each algorithm obtain the same observation values
        '''

        My_true = cla_trues[self._id].X_true.copy()
        # other robots
        for _id in range(self.NUM_ROBOTS):
            if (_id == self._id):
                continue
            _range, bearing, X2_ = parameters.measurement(
                cla_trues[_id].X_true, My_true)
            if _range <= RB and abs(bearing) <= BB:  # self->_id
                self.measuring[_id] = True
                self.sort_pair_request.append(_id)

                if (self.flag >= 0):
                    self.measure_noise[_id, :] = measure_noises[_id, :2]
                    self.Z[_id, 0] = _range + self.measure_noise[_id, 0]
                    self.Z[_id, 1] = bearing + self.measure_noise[_id, 1]
                elif (self.flag == -1):
                    self.measure_noise[_id] = measure_noises[_id, :]
                    self.Z[_id,] = (X2_ + self.measure_noise[_id])

        if self.Cov_id in self.sort_pair_request:
            self.sort_pair_request.remove(self.Cov_id)
            self.sort_pair_request.sort(key=self.sort_by_his)  # For Match
            self.sort_pair_request.insert(0, self.Cov_id)
        else:
            self.sort_pair_request.sort(key=self.sort_by_his)  # For Match

    def abso_meas_correct(self, count=500):
        '''
        Absolute Measurement update
        '''
        for LM_id in range(self.LANDMARK_NUM):
            if not self.measuring_landmark[LM_id]:
                continue

            J = np.array([[0, -1], [1, 0]])
            dp = self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0] - self.X  # (3,)
            gamma = parameters.rot_mat_2d(self.X[2])
            if (self.flag == 0):
                H = np.zeros((2, 3))
                Z_cal = np.zeros((2, 1))

                rho = np.linalg.norm(dp[:2], ord=2)
                rho2 = rho**2

                H[0, 0] = -dp[0]/rho
                H[0, 1] = -dp[1]/rho

                H[1, 0] = dp[1]/rho2
                H[1, 1] = -dp[0]/rho2
                H[1, 2] = -1

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X[2]
                R2 = (R_ALL_0_LANDMARK[2*LM_id:2 *
                      LM_id+2, 2*LM_id:2*LM_id+2])**2

            elif self.flag == 1:
                H = np.zeros((2, 3))
                Z_cal = np.zeros((2, 1))

                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[:2]).reshape(2)
                H = gamma[:2, :2] @ -H_tilde[:2, :]

                Z_cal = (gamma[:2, :2] @ dp[:2]).reshape(2, 1)

                rho, alpha = self.Z_landmark[LM_id,
                                             0], self.Z_landmark[LM_id, 1]

                self.Z_landmark[LM_id, 0] = rho * cos(alpha)
                self.Z_landmark[LM_id, 1] = rho * sin(alpha)

                R = R_ALL_0_LANDMARK[2*LM_id:2 *
                                     LM_id+2, 2*LM_id:2*LM_id+2].copy()
                R[1, 1] = R[1, 1]*rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

            elif (self.flag == -1):
                H = np.zeros((3, 3))
                Z_cal = np.zeros((3, 1))

                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = (J @ dp[0:2]).reshape(2,)

                H = gamma @ -H_tilde

                Z_cal = (gamma @ dp).reshape(3, 1)

                R2 = (R_ALL_1_LANDMARK[3*LM_id:3 *
                      LM_id+3, 3*LM_id:3*LM_id+3])**2

            v = self.Z_landmark[LM_id].reshape(-1, 1) - Z_cal
            if (self.flag == 0 and abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi

            sigma_invention = H @ self.P @ H.T + R2
            sigma = None
            try:
                sigma = np.linalg.inv(np.array(sigma_invention))
            except np.linalg.LinAlgError:
                continue
            K_i = self.P @ H.T @ sigma

            self.X = self.X + np.array(K_i @ v).reshape(3,)
            Coeff = np.eye(3) - K_i @ H
            self.P = Coeff @ self.P

            self.Cov_id = -1

        self.X_prediction = self.X.copy()
        self.P_prediction = self.P.copy()

    def communicate1_1(self, robot_j):
        '''
        Measurement update

        meas_info_kind=
        0: I->robot_j
        1: robot_j->I
        2: I->robot_j, robot_j->I

        :param: robot_j, class of Robot_DMV, other robot

        :return 1 or 0, for successfully update or not
        '''

        self.X_j = robot_j.X_prediction.copy()
        self.P_j = robot_j.P_prediction.copy()

        if self.Cov_id == robot_j._id and robot_j.Cov_id == self._id:
            # Both robots fuses each other last time, so EKF
            self.bEKF = True
            self.P_ij = self.move_update @ self.Cov @ (robot_j.move_update).T

        if self.measuring[robot_j._id]:
            if robot_j.measuring[self._id]:
                # I->robot_j, robot_j->I
                self.meas_info_kind = 2
            else:
                # I->robot_j
                self.meas_info_kind = 0
        else:
            self.meas_info_kind = 1  # robot_j->I

        can = self.EKF_DMV(robot_j)
        self.X_j, self.P_j = None, None
        return can

    def EKF_DMV(self, robot_j):
        '''
        DMV in the Measurement update

        :param: robot_j, class of Robot_DMV, other robot

        :return 1 or 0, for successfully update or not
        '''

        delta_x = self.X_j[0]-self.X[0]
        delta_y = self.X_j[1]-self.X[1]

        if (self.flag == 0):  # range-bearing
            rho = np.linalg.norm([delta_x, delta_y], ord=2)
            rho2 = rho**2

            temp1_x12 = delta_x/rho
            temp1_y12 = delta_y/rho
            temp2_x12 = delta_x/rho2
            temp2_y12 = delta_y/rho2

            # self -> X_j
            # delta_x: measured-measurer
            if self.meas_info_kind == 0:
                Z_cal = np.array([rho, atan2(delta_y, delta_x) - self.X[2]])

                H = np.array([
                    [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                    [temp2_y12, -temp2_x12, -1, -temp2_y12, temp2_x12, 0]
                ])
                R2 = R_0**2
                v = self.Z[robot_j._id] - Z_cal

            # X_j -> self
            # delta_x: measurer-measured
            elif self.meas_info_kind == 1:
                Z_cal = np.array(
                    [rho, atan2(-delta_y, -delta_x) - self.X_j[2]])

                H = np.array([
                    [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                    [temp2_y12, -temp2_x12, 0, -temp2_y12, temp2_x12, -1]
                ])
                R2 = R_0**2
                v = robot_j.Z[self._id] - Z_cal

            # Add robot_j's observation
            elif self.meas_info_kind == 2:
                Z_cal = np.zeros((4))

                Z_cal[:2] = np.array(
                    [rho, atan2(delta_y, delta_x) - self.X[2]])
                Z_cal[2:] = np.array(
                    [rho, atan2(-delta_y, -delta_x) - self.X_j[2]])

                H = np.array([
                    [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                    [temp2_y12, -temp2_x12, -1, -temp2_y12, temp2_x12, 0],
                    [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                    [temp2_y12, -temp2_x12, 0, -temp2_y12, temp2_x12, -1]
                ])
                v = np.concatenate(
                    (self.Z[robot_j._id], robot_j.Z[self._id])) - Z_cal
                R2 = (block_diag(R_0, R_0))**2

        elif (self.flag == 1):  # relative position
            if self.meas_info_kind == 0:

                rot = parameters.rot_mat_2d(self.X[2])[0:2, 0:2]
                Z_cal = rot @ (self.X_j[:2] - self.X[:2])
                c1 = rot[0, 0]
                s1 = rot[0, 1]

                H = np.array([
                    [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                    [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
                ])
                R = R_0.copy()
                rho, alpha = self.Z[robot_j._id, 0], self.Z[robot_j._id, 1]

                Z_now = np.array([rho * cos(alpha), rho * sin(alpha)])

                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R[1, 1] = R[1, 1] * rho
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                v = Z_now - Z_cal

            elif self.meas_info_kind == 1:
                rot = parameters.rot_mat_2d(self.X_j[2])[0:2, 0:2]
                Z_cal = rot @ (self.X[:2] - self.X_j[:2])
                c2 = rot[0, 0]
                s2 = rot[0, 1]

                H = np.array([
                    [c2, s2, 0, -c2, -s2, delta_x*s2 - delta_y*c2],
                    [-s2, c2, 0, s2, -c2, delta_x*c2 + delta_y*s2],
                ])
                R = R_0.copy()
                rho, alpha = robot_j.Z[self._id, 0], robot_j.Z[self._id, 1]

                Z_now = np.array([rho * cos(alpha), rho * sin(alpha)])

                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R[1, 1] = R[1, 1] * rho
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                v = Z_now - Z_cal

            # Add robot_j's observation
            elif self.meas_info_kind == 2:
                Z_cal = np.zeros((4))

                rot1 = parameters.rot_mat_2d(self.X[2])[0:2, 0:2]
                Z_cal[:2] = rot1 @ (self.X_j[:2] - self.X[:2])
                c1 = rot1[0, 0]
                s1 = rot1[0, 1]

                rot2 = parameters.rot_mat_2d(self.X_j[2])[0:2, 0:2]
                Z_cal[2:] = rot2 @ (self.X[:2] - self.X_j[:2])
                c2 = rot2[0, 0]
                s2 = rot2[0, 1]

                H = np.array([
                    [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                    [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
                    [c2, s2, 0, -c2, -s2, delta_x*s2 - delta_y*c2],
                    [-s2, c2, 0, s2, -c2, delta_x*c2 + delta_y*s2],
                ])
                R2 = block_diag(R_0.copy(), R_0.copy())
                rho1, alpha1 = self.Z[robot_j._id, 0], self.Z[robot_j._id, 1]

                gamma_bearing1 = parameters.rot_mat_2d(
                    alpha1)[0:2, 0:2].T  # anticlockwise
                R2[1, 1] = R2[1, 1] * rho1
                R2[:2, :2] = gamma_bearing1 @ R2[:2, :2]**2 @ gamma_bearing1.T

                rho2, alpha2 = robot_j.Z[self._id, 0], robot_j.Z[self._id, 1]

                alpha_cal2 = atan2(
                    self.X[1] - self.X_j[1], self.X[0] - self.X_j[0]) - self.X_j[2]
                if (abs(alpha2 - alpha_cal2) > 4):
                    if alpha2 - alpha_cal2 > 0:
                        alpha2 = alpha2 - 2*pi
                    else:
                        alpha2 = alpha2 + 2*pi

                Z_now = np.array(
                    [rho1 * cos(alpha1), rho1 * sin(alpha1), rho2 * cos(alpha2), rho2 * sin(alpha2)])

                gamma_bearing2 = parameters.rot_mat_2d(
                    alpha2)[0:2, 0:2].T  # anticlockwise
                R2[3, 3] = R2[3, 3] * rho2
                R2[2:, 2:] = gamma_bearing2 @ R2[2:, 2:]**2 @ gamma_bearing2.T

                v = Z_now - Z_cal

        elif (self.flag == -1):  # relative pose
            if self.meas_info_kind == 0:

                rot = parameters.rot_mat_2d(self.X[2])
                Z_cal = rot @ (self.X_j - self.X)
                c1 = rot[0, 0]
                s1 = rot[0, 1]

                H = np.array([
                    [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                    [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
                    [0, 0, -1, 0, 0, 1]
                ])
                v = self.Z[robot_j._id] - Z_cal
                R2 = R_1**2
            elif self.meas_info_kind == 1:
                rot = parameters.rot_mat_2d(self.X_j[2])
                Z_cal = rot @ (self.X - self.X_j)
                c2 = rot[0, 0]
                s2 = rot[0, 1]

                H = np.array([
                    [c2, s2, 0, -c2, -s2, delta_x*s2 - delta_y*c2],
                    [-s2, c2, 0, s2, -c2, delta_x*c2 + delta_y*s2],
                    [0, 0, 1, 0, 0, -1]
                ])
                v = robot_j.Z[self._id] - Z_cal
                R2 = R_1**2

            # Add robot_j's observation
            elif self.meas_info_kind == 2:
                Z_cal = np.zeros((6))

                rot1 = parameters.rot_mat_2d(self.X[2])
                Z_cal[:3] = rot1 @ (self.X_j - self.X)
                c1 = rot1[0, 0]
                s1 = rot1[0, 1]

                rot2 = parameters.rot_mat_2d(self.X_j[2])
                Z_cal[3:] = rot2 @ (self.X - self.X_j)
                c2 = rot2[0, 0]
                s2 = rot2[0, 1]

                H = np.array([
                    [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                    [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
                    [0, 0, -1, 0, 0, 1],
                    [c2, s2, 0, -c2, -s2, delta_x*s2 - delta_y*c2],
                    [-s2, c2, 0, s2, -c2, delta_x*c2 + delta_y*s2],
                    [0, 0, 1, 0, 0, -1]
                ])
                v = np.concatenate(
                    (self.Z[robot_j._id], robot_j.Z[self._id])) - Z_cal
                R2 = (block_diag(R_1, R_1))**2

        # v = self.Z_ - Z_cal

        if (self.flag == 0):
            if (abs(v[1]) > 4):
                if (v[1] > 0):
                    v[1] = v[1] - (2*pi)
                else:
                    v[1] = v[1] + (2*pi)
            if self.meas_info_kind == 2 and abs(v[3]) > 4:
                if (v[3] > 0):
                    v[3] = v[3] - (2*pi)
                else:
                    v[3] = v[3] + (2*pi)

        if self.bEKF:

            temp1 = np.concatenate((self.P, (self.P_ij)), axis=1)
            temp2 = np.concatenate(((self.P_ij).T, self.P_j), axis=1)
            Chi = np.concatenate((temp1, temp2), axis=0)
            S = H @ Chi @ H.T + R2

            if (abs(np.linalg.det(S)) < 1e-7):
                return 0

            S_I = None
            try:
                S_I = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return 0

            K = Chi @ H.T @ S_I

            temp_i = self.X_j.copy()
            temp = self.X.copy()
            temp_Chi = Chi.copy()

            self.X = self.X + K[0:3] @ v

            Chi = Chi - K @ S @ K.T

            self.P = Chi[0:3, 0:3]

            self.Cov = Chi[:3, 3:]

        else:
            # DMV
            H_i = H[:, :3]
            H_j = H[:, 3:]

            inv_P_i, inv_Pj_i = None, None
            try:
                inv_P_i = np.linalg.inv(np.array(self.P))
                inv_Pj_i = np.linalg.inv(H_j @ self.P_j @ H_j.T + R2)
            except np.linalg.LinAlgError:
                return 0

            def fitness(w):
                try:
                    f = np.linalg.inv(w * inv_P_i + (1-w) *
                                      H_i.T @ inv_Pj_i @ H_i)
                    return np.linalg.slogdet(f)[1]
                except np.linalg.LinAlgError:
                    return 100

            res = minimize_scalar(fitness, bounds=(0, 1), method='bounded')
            w = res.x
            gamma = 1-w

            def fitness_output(w):
                return np.linalg.inv(w * inv_P_i + (1-w) * H_i.T @ inv_Pj_i @ H_i)

            temp_I = None
            try:
                temp_I = np.linalg.inv(
                    H_i @ self.P @ H_i.T / w + H_j @ self.P_j @ H_j.T / (1-w) + R2/gamma)
            except np.linalg.LinAlgError:
                return 0

            K_i = self.P / w @ H_i.T @ temp_I

            self.X = self.X + np.array(K_i @ v).reshape(3,)
            self.P = fitness_output(w)

            self.Cov = np.zeros((3, 3))
            self.Cov_id = robot_j._id

        self.move_update = np.eye(3)

        return 1
