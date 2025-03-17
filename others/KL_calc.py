import numpy as np
from scipy.integrate import quad
import scipy.stats
import matplotlib.pyplot as plt  # Add matplotlib for plotting


def integrand(x, a):
    '''
    the joint distribution to be integrated(marginalized)

    :param x: random variable
    :param a: parameter
    :return: pdf of marginal distribution
    '''
    global E_V, Sigma_v0, Sigma_v
    y = 1/(2*np.pi*Sigma_v0*Sigma_v)/x*np.exp(-1/(2*Sigma_v**2)
                                              * (1 - a/x)**2 - 1/(2*Sigma_v0**2)*(x - E_V)**2)
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
        if (i == 0 or i == n-1):
            sum += xs[i]
        elif i % 2:
            sum += 4*xs[i]
        else:
            sum += 2*xs[i]
    return sum * step_size / 3


def plot_distributions(a_s, actual_dist, normal_dist, velocity_type):
    """
    Plot the actual and normal distributions for comparison

    :param a_s: x-axis values (parameter a)
    :param actual_dist: The actual distribution values
    :param normal_dist: The normal approximation distribution values
    :param velocity_type: Type of velocity ('v' or 'omega')
    """
    plt.figure(figsize=(10, 6))

    # Set style for better aesthetics
    plt.style.use('seaborn-whitegrid')

    downsampled_a_s = a_s[::10]
    downsampled_actual_dist = actual_dist[::10]
    downsampled_normal_dist = normal_dist[::10]

    # Plot with enhanced styling
    plt.plot(downsampled_a_s, downsampled_actual_dist, 'o', color='#1f77b4', label="Actual Distribution",
             markersize=3, alpha=0.5)
    plt.plot(downsampled_a_s, downsampled_normal_dist, color='#ff7f0e', label="Normal Approximation",
             linestyle='-', linewidth=2)

    # Add labels and title with better styling
    plt.xlabel("Parameter (a)", fontsize=12, fontweight='bold')
    plt.ylabel("Probability Density", fontsize=12, fontweight='bold')
    plt.title(f"Distribution Comparison for {velocity_type}",
              fontsize=14, fontweight='bold', pad=20)

    # Improve legend
    plt.legend(loc='best', frameon=True, fontsize=10)

    # Enhance grid
    plt.grid(True, alpha=0.3, linestyle='--')

    # Tight layout for better spacing
    plt.tight_layout()
    plt.savefig(
        f"distribution_comparison_{velocity_type}.pdf", dpi=300)  # , bbox_inches='tight')
    plt.close()


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
    Normal = scipy.stats.norm.pdf(a_s, loc=E_V, scale=sigma)

    # Plot the distributions for visual comparison
    plot_distributions(a_s, integral_results, Normal,
                       "Forward Velocity" if v == "v" else "Angular Velocity")

    KL_normal.append(Simpson_integral(
        Normal * np.log2(Normal / integral_results), len(a_s), step_size))

# write out the data
np.savetxt('KL_calc_v.txt', KL_normal)
