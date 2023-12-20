import numpy as np
from math import atan2, pi, sin, cos
import parameters

from algorithms.DR import Robot


R_0 = parameters.R_0
R_1 = parameters.R_1
R_ALL_0_LANDMARK = parameters.R_ALL_0_LANDMARK
R_ALL_1_LANDMARK = parameters.R_ALL_1_LANDMARK

RB = parameters.MAESUREMENT_RANGE_BOUND
BB = parameters.MAESUREMENT_BEARING_BOUND


class Robot_BDA_EKF(Robot):
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
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 2))
            self.note = np.zeros((NUM_ROBOTS, 2))
        elif (flag == -1):
            self.Z = np.zeros((NUM_ROBOTS, 3))
            self.Z_landmark = np.zeros((3*self.LANDMARK_NUM, 1))

        self.Cov = np.zeros((NUM_ROBOTS, 3, 3))
        self.pair = -1
        self.sort_pair_request = []
        self.pair_history = np.zeros(NUM_ROBOTS, dtype=np.uint16)

        self.measuring = np.zeros(NUM_ROBOTS, dtype=bool)

        self.contain_bias_abso = np.zeros(self.LANDMARK_NUM, dtype=bool)
        self.contain_bias_rela = np.zeros(NUM_ROBOTS, dtype=bool)

    def reset_abso(self):
        '''
        Reset variables about the absolute measurement(landmark)
        '''
        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)
        self.contain_bias_abso = np.zeros(self.NUM_ROBOTS, dtype=bool)

        if (self.flag >= 0):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 2))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 2))
        elif (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 3))
        # self.un = np.linalg.eig(self.P)[0][0]
        # self.un = np.trace(self.P)

    def reset_rela(self):
        '''
        Reset variables about the relative measurement(other robots)
        '''

        self.pair = -1
        self.sort_pair_request = []
        self.measuring = np.zeros(self.NUM_ROBOTS, dtype=bool)
        self.contain_bias_rela = np.zeros(self.NUM_ROBOTS, dtype=bool)

        if (self.flag >= 0):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 2))
            self.Z = np.zeros((self.NUM_ROBOTS, 2))
        elif (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z = np.zeros((self.NUM_ROBOTS, 3))

    def motion(self, v, omega):
        '''
        time propagation update with unicycle model

        :param: v: float, linear velocity
        :param: omega: float, angular velocity
        '''
        F = Robot.motion(self, v, omega)
        for _ in range(self.NUM_ROBOTS):
            if _ == self._id:
                continue
            self.Cov[_] = F @ self.Cov[_]

    def sort_by_his(self, elem):
        '''
        sort the pair history according to the history of the robot

        :param: elem: int, the index of the robot

        :return: int, the index of the robot
        '''
        return self.pair_history[elem]

    def measurement_abso(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all the observation ans sends to corresponding robot

        :param: cla_true: list of class "Robot_true"
            All GroundTruth about robots
        :param: measure_noises: list of measure_noises
            Let each algorithm obtain the same observation values
        :param: measure_bias_whether: list of whether the measurement is biased
        '''

        My_true = cla_trues[self._id].X_true.copy()
        # landmark
        for LM_id in range(self.LANDMARK_NUM):
            _range, bearing, X2_ = parameters.measurement(
                self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0], My_true)
            if _range <= RB and abs(bearing) <= BB:  # self->LM_id
                self.measuring_landmark[LM_id] = True

                if measure_bias_whether[LM_id]:
                    self.contain_bias_abso[LM_id] = True

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
        :param: measure_bias_whether: list of whether the measurement is biased
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

                if measure_bias_whether[_id]:
                    self.contain_bias_rela[_id] = True

                if (self.flag >= 0):
                    self.measure_noise[_id, :] = measure_noises[_id, :2].copy()
                    self.Z[_id, 0] = _range + self.measure_noise[_id, 0]
                    self.Z[_id, 1] = bearing + self.measure_noise[_id, 1]
                elif (self.flag == -1):
                    self.measure_noise[_id] = measure_noises[_id, :]
                    self.Z[_id] = (X2_ + self.measure_noise[_id]).tolist()

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

            for r in range(self.NUM_ROBOTS):
                if not r == self._id:
                    self.Cov[r] = Coeff @ self.Cov[r]

        self.P_prediction = self.P.copy()
        self.X_prediction = self.X.copy()

    def communicate2_1(self, robot_j, get_other=True):
        '''
        Measurement update

        :param: robot_j, class of Robot_DMV, other robot
        :param: cla_trues, class of Robot_true
        :param: get_other: bool
            whether need other robots' observation. In BDA-CU, no need.
        '''
        self.X_j = robot_j.X_prediction.copy()
        self.P_j = robot_j.P_prediction.copy()
        self.P_ij = self.Cov[robot_j._id] @ (robot_j.Cov[self._id]).T

        return self.EKF2(robot_j, get_other)

    def EKF2(self, robot_j, get_other):
        '''
        EKF, part of Measurement update

        :param: robot_j, class of Robot_DMV, other robot
        :param: get_other: bool
            whether need other robots' observation. In BDA-CU, no need.
        '''
        two_meas = False

        if (get_other and robot_j.measuring[self._id]):
            two_meas = True

        delta_x = self.X_j[0]-self.X[0]
        delta_y = self.X_j[1]-self.X[1]

        if (self.flag == 0):  # bearing
            rho = np.linalg.norm([delta_x, delta_y], ord=2)
            rho2 = rho**2
            Z_cal = np.array([rho, atan2(delta_y, delta_x) - self.X[2]])
            Z_now = self.Z[robot_j._id].copy()

            temp1_x12 = delta_x/rho
            temp1_y12 = delta_y/rho
            temp2_x12 = delta_x/rho2
            temp2_y12 = delta_y/rho2
            H = np.array([
                [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                [temp2_y12, -temp2_x12, -1, -temp2_y12, temp2_x12, 0]
            ])
            R = R_0
            R2 = R**2

            # Add robot_j's observation
            if two_meas:
                Z_cal = np.concatenate(
                    (Z_cal, [rho, atan2(-delta_y, -delta_x) - self.X_j[2]]))
                Z_now = np.concatenate((Z_now,  robot_j.Z[self._id]))
                H = np.vstack((H, np.array([
                    [-temp1_x12, -temp1_y12, 0, temp1_x12, temp1_y12, 0],
                    [temp2_y12, -temp2_x12, 0, -temp2_y12, temp2_x12, -1]
                ])))
                R = np.zeros((4, 4))
                R[:2, :2] = R_0
                R[2:, 2:] = R_0
                R2 = R**2

        # range-bearing, but transforms into relative position
        elif (self.flag == 1):
            rot = parameters.rot_mat_2d(self.X[2])[0:2, 0:2]
            Z_cal = rot @ (self.X_j[0:2] - self.X[0:2])
            Z_now = np.zeros(2)
            c1 = rot[0, 0]
            s1 = rot[0, 1]

            H = np.array([
                [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
            ])
            R = R_0.copy()
            rho, alpha = self.Z[robot_j._id, 0], self.Z[robot_j._id, 1]

            Z_now[0] = rho * cos(alpha)
            Z_now[1] = rho * sin(alpha)

            gamma_bearing = parameters.rot_mat_2d(
                alpha)[0:2, 0:2].T  # anticlockwise
            R[1, 1] = R[1, 1] * rho
            R2 = gamma_bearing @ R**2 @ gamma_bearing.T

            # Add robot_j's observation
            if two_meas:
                rot = parameters.rot_mat_2d(self.X_j[2])[0:2, 0:2]
                Z_cal = np.concatenate(
                    (Z_cal, rot @ (self.X[:2] - self.X_j[:2])))
                Z_now = np.concatenate((Z_now, np.zeros(2)))
                c2 = rot[0, 0]
                s2 = rot[0, 1]

                H = np.vstack((H, np.array([
                    [c2, s2, 0, -c2, -s2, -delta_x*s2 + delta_y*c2],
                    [-s2, c2, 0, s2, -c2, -delta_x*c2 - delta_y*s2],
                ])))
                R_ = R2.copy()
                R2 = np.zeros((4, 4))
                R2[:2, :2] = R_.copy()
                R2[2:, 2:] = R_0.copy()

                rho, alpha = robot_j.Z[self._id, 0], robot_j.Z[self._id, 1]

                alpha_cal = atan2(
                    self.X[1] - self.X_j[1], self.X[0] - self.X_j[0]) - self.X_j[2]
                if (abs(alpha - alpha_cal) > 4):
                    if alpha - alpha_cal > 0:
                        alpha = alpha - 2*pi
                    else:
                        alpha = alpha + 2*pi

                Z_now[2] = rho * cos(alpha)
                Z_now[3] = rho * sin(alpha)

                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2[3, 3] = R2[3, 3] * rho
                R2[2:, 2:] = gamma_bearing @ R2[2:, 2:]**2 @ gamma_bearing.T

        elif (self.flag == -1):  # relative pose
            rot = parameters.rot_mat_2d(self.X[2])
            Z_cal = rot @ (self.X_j - self.X)
            Z_now = self.Z[robot_j._id].copy()

            c1 = rot[0, 0]
            s1 = rot[0, 1]

            H = np.array([
                [-c1, -s1, -delta_x*s1 + delta_y*c1, c1, s1, 0],
                [s1, -c1, -delta_x*c1 - delta_y*s1, -s1, c1, 0],
                [0, 0, -1, 0, 0, 1]
            ])
            R = R_1
            R2 = R**2

            # Add robot_j's observation
            if two_meas:
                rot = parameters.rot_mat_2d(self.X_j[2])
                Z_cal = np.concatenate((Z_cal, rot @ (self.X - self.X_j)))
                Z_now = np.concatenate((Z_now, robot_j.Z[self._id]))
                c2 = rot[0, 0]
                s1 = rot[0, 1]

                H = np.vstack((H, np.array([
                    [c2, s1, 0, -c2, -s1, -delta_x*s1 + delta_y*c2],
                    [-s1, c2, 0, s1, -c2, -delta_x*c2 - delta_y*s1],
                    [0, 0, 1, 0, 0, -1]
                ])))
                R = np.zeros((6, 6))
                R[:3, :3] = R_1
                R[3:, 3:] = R_1
                R2 = R**2

        temp1 = np.concatenate((self.P, (self.P_ij)), axis=1)
        temp2 = np.concatenate(((self.P_ij).T, self.P_j), axis=1)
        Chi = np.concatenate((temp1, temp2), axis=0)

        v = Z_now - Z_cal

        if (self.flag == 0):
            if (abs(v[1]) > 4):
                if (v[1] > 0):
                    v[1] = v[1] - (2*pi)
                else:
                    v[1] = v[1] + (2*pi)
            if two_meas and abs(v[3]) > 4:
                if (v[3] > 0):
                    v[3] = v[3] - (2*pi)
                else:
                    v[3] = v[3] + (2*pi)

        S = H @ Chi @ H.T + R2

        S_I = None
        try:
            S_I = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return 0

        K = Chi @ H.T @ S_I

        X_pre = self.X.copy()
        P_pre = Chi.copy()

        self.X = self.X + K[0:3] @ v
        self.X_j = self.X_j + K[3:] @ v

        Chi = Chi - K @ S @ K.T

        self.P_j = Chi[3:, 3:]

        self.P = Chi[0:3, 0:3]
        self.P_ji = np.eye(3)

        # Update others' cross-covariance
        P_prediction_I = None
        try:
            P_prediction_I = np.linalg.inv(self.P_prediction)
        except np.linalg.LinAlgError:
            self.X = X_pre.copy()
            self.P = P_pre[:3, :3].copy()
            return 0

        for _ in range(self.NUM_ROBOTS):
            if _ == self._id:
                continue
            elif _ == robot_j._id:
                # self.Cov[_] = np.eye(3)
                self.Cov[_] = Chi[0:3, 3:]
            else:
                self.Cov[_] = self.P @ P_prediction_I @ self.Cov[_]

        return 1

    def communicate2_2(self, robot_i):
        '''
        Send the filter result back, part of Measurement update

        :param: robot_i, class of Robot_DMV, other robot
        '''
        self.X = (robot_i.X_j).copy()
        self.P = (robot_i.P_j).copy()

        P_prediction_I = None
        try:
            P_prediction_I = np.linalg.inv(self.P_prediction)
        except np.linalg.LinAlgError:
            print("Here comes a problem that need to be solved!!!")
            return

        for _ in range(self.NUM_ROBOTS):
            if _ == self._id:
                continue
            elif _ == robot_i._id:
                self.Cov[_] = (robot_i.P_ji).copy()
            else:
                self.Cov[_] = self.P @ P_prediction_I @ self.Cov[_]
