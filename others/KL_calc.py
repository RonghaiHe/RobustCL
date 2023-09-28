import numpy as np
from scipy.integrate import quad
import scipy.stats

def integrand(x, a):
    '''
    the joint distribution to be integrated(marginalized)

    :param x: random variable
    :param a: parameter
    :return: pdf of marginal distribution
    '''
    global E_V, Sigma_v0, Sigma_v
    y = 1/(2*np.pi*Sigma_v0*Sigma_v)/x*np.exp(-1/(2*Sigma_v**2)*(1 - a/x)**2 - 1/(2*Sigma_v0**2)*(x - E_V)**2)
    return y

def Simpson_integral(xs, n, step_size):
    '''
    Simpson integral

    :param: xs: list of x(random variable)
    :param: n: length of xs
    :param: step_size: step size of x
    :return: integral value of xs
    '''
    sum = 0
    for i in range(n):
        if(i==0 or i==n-1):
            sum += xs[i]
        elif i % 2: 
            sum += 4*xs[i]
        else:
            sum += 2*xs[i]
    return sum * step_size / 3

# Parameters of grid about the numerical integration 
a_start = -1
a_end = 1
step_size = 0.001

a_s = np.arange(a_start, a_end, step_size)

global E_V, Sigma_v0, Sigma_v

vs = ['v', 'omega']
KL_normal = []
for v in vs:

    if v == 'v':
        # Parameters: foward velocity
        E_V = 0.1
        Sigma_v0 = 1/30
        Sigma_v = 0.7
    else:
        # Parameters: angular velocity
        E_V = 0
        Sigma_v0 = 1/3
        Sigma_v = 0.3

    # init
    integral_results = []

    # circulate
    for a in a_s:
        
        # Using numerical integration to calculate the actual distribution
        integral_result = quad(lambda x: integrand(x, a), -np.inf, np.inf)
        
        integral_results.append(integral_result[0])

    integral_results = np.abs(np.array(integral_results))

    print(Simpson_integral(integral_results, len(a_s), step_size))

    # Normal distribution
    sigma = np.sqrt(Sigma_v0**2 + Sigma_v**2 * (E_V**2 + Sigma_v0**2))
    Normal = scipy.stats.norm.pdf(a_s, loc = E_V, scale = sigma)

    KL_normal.append(Simpson_integral(Normal * np.log2(Normal / integral_results), len(a_s), step_size))

# write out the data
np.savetxt('KL_calc_v.txt', KL_normal)