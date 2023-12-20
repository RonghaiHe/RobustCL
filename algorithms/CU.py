from scipy.linalg import sqrtm
from algorithms.func_detect_fault import Maha
from algorithms.BDA import Robot_BDA_EKF
from algorithms.CI import Robot_CI
import parameters
import numpy as np
from scipy.optimize import minimize_scalar

R_1 = parameters.R_1
R_0 = parameters.R_0


class Robot_BDA_EKF_CU(Robot_BDA_EKF):

    def CU_fuse(self):
        '''
        CU fusion
        '''
        def fitness(w):
            '''
            fitness function in CU

            :param: w: float, weight, [0,1], optimization variable
            '''
            X_i = (1-w)*self.X + w*self.X_i_j
            Ua = np.array([X_i - self.X_i_j]).T @ np.array([X_i - self.X_i_j])
            Ub = np.array([X_i - self.X]).T @ np.array([X_i - self.X])

            B = sqrtm(self.P_i_j + Ua).real

            temp = np.linalg.inv(B)
            G = temp.T @ (self.P + Ub) @ temp
            D, V = np.linalg.eig(G)
            temp1 = B.T @ V
            P_i = temp1 @ np.maximum(np.diag(D), np.eye(3)) @ temp1.T
            return np.linalg.slogdet(P_i)[1]

        def fitness_output(w):
            '''
            fitness function output in CU, the difference between the return

            :param: w: float, weight, [0,1], optimization variable
            '''
            X_i = (1-w)*self.X + w*self.X_i_j
            Ua = (X_i - self.X_i_j) @ (X_i - self.X_i_j).T
            Ub = (X_i - self.X) @ (X_i - self.X).T

            B = sqrtm(self.P_i_j + Ua).real
            temp = np.linalg.inv(B)

            G = temp.T @ (self.P + Ub) @ temp
            D, V = np.linalg.eig(G)
            temp1 = B.T @ V
            P_i = temp1 @ np.maximum(np.diag(D), np.eye(3)) @ temp1.T
            return X_i, P_i

        res = minimize_scalar(fitness, bounds=(0, 1), method='bounded')
        w = res.x

        self.X, self.P = fitness_output(w)

    def communicate2_2CU(self, robot_i):
        '''
        Measurement update

        :param: robot_i: class of Robot_BDA_EKF_CU, other robots
        '''
        if not parameters.is_pos_def(robot_i.P_j):
            print("BDA-CU: not positive definition!!!")
            return

        self.X_i_j = robot_i.X_j.copy()
        self.P_i_j = robot_i.P_j.copy()

        self.CU_fuse()

# [1] J. Klingner, N. Ahmed, and N. Correll, “Fault-tolerant Covariance Intersection for localizing robot swarms,” Robotics and Autonomous Systems, vol. 122, p. 103306, Dec. 2019.


class Robot_CI_CU(Robot_CI):

    # Coefficient usedin the FDE
    d_TOSS = 0.1
    d_CU = 0.025

    def __init__(self, X, _id, NUM_ROBOTS, flag, LANDMARK_POS):
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        :param: NUM_ROBOTS: int, the number of robots
        :param: flag: int
        :param: LANDMARK_POS: position of landmarks
        '''
        Robot_CI.__init__(self, X, _id, NUM_ROBOTS, flag, LANDMARK_POS)
        self.TP, self.FP, self.FN, self.TN = 0, 0, 0, 0

    def communicate1(self, robot_i):
        '''
        Communication update

        :param: robot_i: class of Robot_CI_CU, other robots
        '''
        eps = 1e-5
        gamma_i = parameters.rot_mat_2d(
            robot_i.X_prediction[2]).T  # anticlockwise
        J = np.array([[0, -1], [1, 0]])
        H_tilde = np.eye(3)

        inv_P = None
        if self.flag == -1:
            X_j = robot_i.X_prediction + gamma_i @ robot_i.Z[self._id]

            dp = X_j[0:2] - robot_i.X_prediction[0:2]
            H_tilde[0:2, 2] = (J @ dp).reshape(2,)

            P_j = H_tilde @ robot_i.P_prediction @ H_tilde.T + gamma_i @ R_1**2 @ gamma_i.T
            try:
                inv_P = np.linalg.inv(self.P)
            except np.linalg.LinAlgError:
                return 0

        inv_P_j = None
        try:
            inv_P_j = np.linalg.inv(P_j)
        except np.linalg.LinAlgError:
            return 0

        # X_j, P_j should be sent to robot_j from robot_i
        # Approximation formula shown in the paper
        w = 0.5 + (np.linalg.det(inv_P) - np.linalg.det(inv_P_j)) / \
            np.linalg.det(inv_P + inv_P_j)/2
        P_CI = None
        try:
            P_CI = np.linalg.inv(w * inv_P + (1-w) * inv_P_j)
        except np.linalg.LinAlgError:
            return 0

        if self.flag == -1:
            X_CI = P_CI @ (w*inv_P @ (self.X).reshape(3, 1) +
                           (1-w)*inv_P_j @ X_j.reshape(3, 1)).reshape(3,)
            delta_x = (self.X - X_j).reshape(3, 1)

        d = Maha(self.X, X_CI, self.P, P_CI)
        if d > self.d_TOSS:
            # Drop out
            if (robot_i.contain_bias[self._id]):
                self.TP += 1
            else:
                self.FP += 1
            return 0
        elif d > self.d_CU:
            # CU step
            U1 = P_j + np.array([X_CI - X_j]).T @ np.array([X_CI - X_j])
            U2 = self.P + np.array([X_CI - self.X]
                                   ).T @ np.array([X_CI - self.X])
            try:
                S = np.linalg.cholesky(U2)
                S_I = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return 0

            R = S_I.T @ U1 @ S_I
            eigvalues, V = np.linalg.eig(R)
            max_lambdas = [max(eigvalue, 1) for eigvalue in eigvalues]
            Max_lambdas = np.diag(max_lambdas)

            StV = S.T @ V
            self.P = StV @ Max_lambdas @ StV.T
            self.X = X_CI.copy()

            if (robot_i.contain_bias[self._id]):
                self.TP += 1
            else:
                self.FP += 1

        else:
            if self.flag == -1:
                self.X = X_CI.copy()
                self.P = P_CI.copy()

            if (robot_i.contain_bias[self._id]):
                self.FN += 1
            else:
                self.TN += 1
        return 1
