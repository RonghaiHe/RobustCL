import numpy as np
from math import atan2, cos, sin, pi
import parameters

from algorithms.DR import Robot
from scipy.optimize import minimize_scalar

R_1 = parameters.R_1
R_ALL_1_LANDMARK = parameters.R_ALL_1_LANDMARK
R_ALL_0_LANDMARK = parameters.R_ALL_0_LANDMARK


RB = parameters.MAESUREMENT_RANGE_BOUND
BB = parameters.MAESUREMENT_BEARING_BOUND


class Robot_CI(Robot):

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
        assert (
            flag < 2, "only the observation model is at least relative position with flag<2, CI could be used")
        Robot.__init__(self, X, _id, NUM_ROBOTS)
        self.flag = flag
        self.measuring = np.zeros(NUM_ROBOTS, dtype=bool)
        self.LANDMARK_NUM = len(LANDMARK_POS)
        self.LANDMARK_POS = np.array(
            LANDMARK_POS).reshape((3*self.LANDMARK_NUM, 1))
        self.contain_bias = np.zeros(NUM_ROBOTS, dtype=bool)

    def reset_rela(self):
        '''
        Reset the variables about the relative measurement
        '''

        self.measuring = np.zeros(self.NUM_ROBOTS, dtype=bool)
        self.contain_bias_rela = np.zeros(self.NUM_ROBOTS, dtype=bool)

        if (self.flag >= 0):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 2))
            self.Z = np.zeros((self.NUM_ROBOTS, 2))
        elif (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z = np.zeros((self.NUM_ROBOTS, 3))

    def reset_abso(self):
        '''
        Reset the variables about the absolute measurement
        '''

        self.measuring_landmark = np.zeros(self.LANDMARK_NUM, dtype=bool)
        self.contain_bias_abso = np.zeros(self.NUM_ROBOTS, dtype=bool)

        if (self.flag >= 0):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 2))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 2))
        elif (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z_landmark = np.zeros((self.LANDMARK_NUM, 3))

    def measurement_abso(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all measurements and later EKF

        :param: cla_true: list of class "Robot_true", All GroundTruth about robots
        :param: measure_noises: list of measure_noises
        :param: measure_bias_whether: list of whether the measurement is biased
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
                    self.measure_noise[LM_id] = measure_noises[LM_id, 0]
                    self.Z_landmark[LM_id, 0] = _range + \
                        self.measure_noise[LM_id, 0]
                    self.Z_landmark[LM_id, 1] = bearing + \
                        self.measure_noise[LM_id, 1]
                elif (self.flag == -1):
                    # self.measure_noise[LM_id] = measure_noises[LM_id,:]*np.array([R_1[0,0], R_1[1,1], R_1[2,2]])
                    self.measure_noise[LM_id] = measure_noises[LM_id, :]
                    self.Z_landmark[LM_id] = (
                        X2_ + measure_noises[LM_id, :]).reshape(1, 3)

    def measurement_rela(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all measurements and later sends to the corresponding robot

        :param: cla_true: list of class "Robot_true", All GroundTruth about robots
        :param: measure_noises: list of measure_noises
        :param: measure_bias_whether: list of whether the measurement is biased
        '''

        My_true = cla_trues[self._id].X_true.copy()
        # Loop other robots
        for r in range(self.NUM_ROBOTS):
            if (r == self._id):
                continue
            _range, bearing, X2_ = parameters.measurement(
                cla_trues[r].X_true, My_true)
            if _range <= RB and abs(bearing) <= BB:  # self->r
                self.measuring[r] = True

                if (self.flag >= 0):
                    self.measure_noise[r] = measure_noises[r, 0]
                    self.Z[r, 0] = _range + self.measure_noise[r, 0]
                    self.Z[r, 1] = bearing + self.measure_noise[r, 1]
                elif (self.flag == -1):
                    self.measure_noise[r] = measure_noises[r, :]
                    self.Z[r] = (X2_ + measure_noises[r, :]).reshape(1, 3)

    def abso_meas_correct(self, count=500):
        '''
        Absolute Measurement update
        :param: count: simulation round, for showing the weight of the M-estimation
        '''
        J = np.matrix([[0, -1], [1, 0]])
        for LM_id in range(self.LANDMARK_NUM):
            if not self.measuring_landmark[LM_id]:
                continue
            if (self.flag >= 0):
                H = np.zeros((2, 3))
                Z_cal = np.zeros((2, 1))
            elif (self.flag == -1):
                H = np.zeros((3, 3))
                Z_cal = np.zeros((3, 1))

            Z_now = (self.Z_landmark[LM_id]).T
            gamma = parameters.rot_mat_2d(self.X[2])
            dp = self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0] - self.X  # (3,)

            if (self.flag == 0):
                rho = np.linalg.norm(dp[:2], ord=2)
                rho2 = rho**2

                H[0, 3*self._id] = -dp[0]/rho
                H[0, 3*self._id+1] = -dp[1]/rho

                H[1, 3*self._id] = dp[1]/rho2
                H[1, 3*self._id+1] = dp[0]/rho2
                H[1, 3*self._id+2] = -1

                Z_cal[0, 0] = rho
                Z_cal[1, 0] = atan2(dp[1], dp[0]) - self.X[2]

                R2 = (R_ALL_0_LANDMARK[2*LM_id:2 *
                      LM_id+2, 2*LM_id:2*LM_id+2])**2

            elif (self.flag == 1):

                R = R_ALL_0_LANDMARK[2*LM_id:2 *
                                     LM_id+2, 2*LM_id:2*LM_id+2].copy()
                gamma = parameters.rot_mat_2d(self.X[2])[0:2, 0:2]

                rho, alpha = Z_now[0, 0], Z_now[1, 0]

                # alpha_cal = atan2(dp[1], dp[0]) - self.X[2]
                # if(abs(alpha - alpha_cal) > 4):
                #     if alpha - alpha_cal > 0:
                #         alpha = alpha - 2*pi
                #     else:
                #         alpha = alpha + 2*pi

                R[1, 1] = R[1, 1] * rho
                gamma_bearing = parameters.rot_mat_2d(
                    alpha)[0:2, 0:2].T  # anticlockwise
                R2 = gamma_bearing @ R**2 @ gamma_bearing.T

                Z_now[0, 0] = rho * cos(alpha)
                Z_now[1, 0] = rho * sin(alpha)

                H_tilde = np.hstack((np.eye(2), J @ dp))

                H = gamma @ -H_tilde
                Z_cal = (gamma @ dp[:2]).reshape(2, 1)
            elif (self.flag == -1):

                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = np.array(J @ np.array([dp[0:2]]).T)[:, 0]

                H[3*LM_id:3*LM_id+3, :] = gamma @ -H_tilde
                # H[3*LM_id:3*LM_id+3, 3*LM_id:3*LM_id+3] = gamma

                # print(gamma @ -H_tilde)
                # print(-dp[0]*math.sin(self.X_GS[3*self._id+2, 0]) + dp[1]*math.cos(self.X_GS[3*self._id+2, 0]))
                # print(-dp[0]*math.cos(self.X_GS[3*self._id+2, 0]) - dp[1]*math.sin(self.X_GS[3*self._id+2, 0]))

                # H[3*LM_id:3*LM_id+3, 3*self._id:3*self._id+3] = -gamma
                # H[3*LM_id, 3*self._id+2] = -dp[0]*math.sin(self.X_GS[3*self._id+2, 0]) + dp[1]*math.cos(self.X_GS[3*self._id+2, 0])
                # H[3*LM_id+1, 3*self._id+2] = -dp[0]*math.cos(self.X_GS[3*self._id+2, 0]) - dp[1]*math.sin(self.X_GS[3*self._id+2, 0])
                # H[3*LM_id:3*LM_id+3, 3*LM_id:3*LM_id+3] = gamma

                Z_cal = (gamma @ dp).reshape(3, 1)
                R2 = (R_ALL_1_LANDMARK[3*LM_id:3 *
                      LM_id+3, 3*LM_id:3*LM_id+3])**2

            v = Z_now - Z_cal
            if (self.flag == 0 and abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi
            # if(np.linalg.norm(v)>1):
            #     print(f"CI: My={self._id}, {v.T}")
            sigma_invention = H @ self.P @ H.T + R2
            # print(abs(np.linalg.det(sigma_invention)))
            # print(sigma_invention)
            # print(np.matrix(sigma_invention))
            # K = self.P_GS @ H.T @ np.linalg.pinv(np.mat(sigma_invention), rcond=1e-8)
            sigma = None
            try:
                sigma = np.linalg.inv(np.array(sigma_invention))
            except np.linalg.LinAlgError:
                continue
            K_i = self.P @ H.T @ sigma

            self.X = self.X + np.array(K_i @ v).reshape(3,)
            Coeff = np.eye(3) - K_i @ H
            self.P = Coeff @ self.P

            self.X_prediction = self.X.copy()
            self.P_prediction = self.P.copy()
