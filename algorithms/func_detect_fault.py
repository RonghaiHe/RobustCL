import numpy as np
import math

def Maha(X_pre, X, P_pre, P):
    '''
    Mahalanobis distance
    
    :param: X_pre: np.array(3,), state before fusion
    :param: X: np.array(3,), state after fusion
    :param: P_pre: np.array(3,3), covariance matrix before fusion
    :param: P: np.array(3,3), covariance matrix after fusion
    :return: Mahalanobis distance
    '''
    
    Inv = None
    try:
        Inv = np.linalg.inv(P + P_pre)
    except np.linalg.LinAlgError:
        return 1e3
    return ((X_pre-X).T @ Inv @ (X_pre-X))
