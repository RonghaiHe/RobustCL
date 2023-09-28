import matplotlib.pyplot as plt
import os
import scienceplots
import numpy as np
import parameters

DELTA_T = parameters.DELTA_T
NUM_ROBOTS = parameters.NUM_ROBOTS


#    blue,       orange,    green,     red,      purple,    brown,    pink,      gray,      yellow,     cyan
# ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
EACH_COLOR = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#e377c2',
               '#7f7f7f', '#bcbd22', '#17becf', '#1f77b4', 'k']

MARKER_STYLE = ['o', '^', 'o', '*', 'o', 'o', '*', '^', 's', '*', '*']
LABELS = [
          'BDA[8]', '1', 'BDA-CU[2]', '1', '1', '1', 'DMV[1]', '1', '1', '1',
          '1', '1', 'CI+CU[13]', '1', '1', '1', '1', '1', '1', '1',
          '1', '1', '1', '1', '1', '1', '1', '1', 'Ours', '1',
          'CCL', 'DR' ]

MEAN_LABELS = [r'$\Delta x$', r'$\Delta y$']

TEXT_BBOX = dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor='white', alpha=0.7)

def each_mean_(numbers, _list, tag='R'):
    '''
    output RMSE of each robot over time

    :param: numbers: int, the number of simulation time
    :param: _list: dict, the dict of RMSE of each robot
    :param: tag: str, the tag of metric, 'R' for RMSE, 'A' for ANEES
     '''
    t=np.arange(numbers)
    fig, ax = plt.subplots(NUM_ROBOTS,1)
    path = './data/relative-pose/'
    if not os.path.exists(path):
        os.makedirs(path)
    file_path = path + tag + '_each_' + str(parameters.comm_fail_prob) + '.csv'

    with open(file_path, 'w') as f:
        for type in _list.keys():
            f.write(str(type) + ',')
        f.write('\n')
        for count in range(numbers):
            for type, List in _list.items():            
                np.savetxt(f, List[:,count], fmt = '%.4f', newline=',')
                f.write(',')    
            f.write('\n')
            
def total_mean_(numbers, Lists, tag):
    '''
    output RMSE of each algorithm(mission 1)

    :param: numbers: int, the number of simulation time
    :param: _list: dict, the dict of RMSE of each robot
    :param: tag: str, the tag of metric, 'R' for RMSE, 'A' for ANEES
     '''
    
    plt.style.use(['science', 'ieee', 'no-latex'])
       
    path = './data/relative-pose/'
    if not os.path.exists(path):
        os.makedirs(path)
    file_path = path + tag + str(parameters.comm_fail_prob) + '.csv'
    with open(file_path, 'w') as f:
        for type in Lists.keys():
            f.write(str(type) + ',')
        f.write('\n')
        
        for type, List in Lists.items():
            Mean = [np.mean(List[:int(numbers/2)]), np.mean(List[int(numbers/2):])]
            np.savetxt(f, Mean, fmt = '%.4f', newline=',')
            f.write('\n')

    t=np.arange(numbers)*DELTA_T
    fig3, ax3 = plt.subplots(1,1)
    count = 0
    for type, List in Lists.items():
        ax3.plot(t,List, '-', label=LABELS[type], c = EACH_COLOR[count], linewidth = 2) # , marker = MARKER_STYLE[count]
        count = count + 1

    ax3.set_xlabel('simulation time [s]')
    ax3.set_xlim(left=0, right=(t[-1]+1e-1))
        
    path_fig = './figures/relative-pose/'
    if not os.path.exists(path_fig):
        os.makedirs(path_fig)
    if tag == 'R':
        ax3.legend(loc="upper left")
        ax3.set_ylabel('ARMSE [m]')
        ax3.plot([t[int(numbers/2)], t[int(numbers/2)]], [0, 6], 'k', linewidth = 2) # tau=0 vs 0.5
        ax3.set_ylim(bottom=0, top=2)
        ax3.text(50, 1.9, r'$\rho = %.1f$' % parameters.comm_fail_prob, color='k', ha='center', va='center', bbox=TEXT_BBOX)
        ax3.text(43, 1.7, r'$\tau = 0$', color='k', ha='center', va='center', fontsize = 6, bbox=TEXT_BBOX)
        ax3.text(57, 1.7, r'$\tau = 0.5$', color='k', ha='center', va='center', fontsize = 6, bbox=TEXT_BBOX)
        
        plt.savefig(path_fig + 'RMSE_time_avg_' + str(parameters.comm_fail_prob) + '.pdf', dpi=600, bbox_inches='tight')
    elif tag == 'A':
        ax3.legend(loc="upper right")
        ax3.set_ylabel('ANEES')
        ax3.plot([t[int(numbers/2)], t[int(numbers/2)]], [0, 30], 'k', linewidth = 2)
        ax3.plot([0, t[-1]], [2, 2], 'k--')
        ax3.set_ylim(bottom=0, top=30)
        
        plt.savefig(path_fig + 'ANEES_time_avg_' + str(parameters.comm_fail_prob) + '.pdf', dpi=600, bbox_inches='tight')

def total_mean_M(numbers, Dicts):
    '''
    draw average weight of M-estimation(mission 2)

    :param: numbers: int, the number of simulation time
    :param: Dicts: dict, the dictionary of average weight of M-estimation
    '''

    plt.style.use(['science', 'ieee', 'no-latex'])

    t=np.arange(numbers)*DELTA_T
    
    for rho in (0.1, 0.5, 0.9):
        
        fig3, ax3 = plt.subplots(1,1)
        ax3.plot([t[int(numbers/2)], t[int(numbers/2)]], [0, 1.2], 'k', linewidth = 2)
        count = 0
        for type, List in Dicts.items():
            for dim in range(2):
                ax3.plot(t,List[rho][:,dim], '-', label=MEAN_LABELS[dim], c = EACH_COLOR[count], linewidth = 1.5) # , marker = MARKER_STYLE[count]
                count = count + 1

        ax3.set_xlabel('simulation time [s]')
        ax3.set_xlim(left=0, right=(t[-1]+1e-1))
        ax3.legend(loc="lower left")
        ax3.set_ylabel('average weight')
        ax3.set_ylim(bottom=0, top=1.2)
        ax3.text(50, 1.1, r'$\rho = %.1f$' % rho, color='k', ha='center', va='center', bbox=TEXT_BBOX)
        ax3.text(40, 0.1, r'$\tau = 0$', color='k', ha='center', va='center', bbox=TEXT_BBOX)
        ax3.text(60, 0.1, r'$\tau = 0.5$', color='k', ha='center', va='center', bbox=TEXT_BBOX)

        path_fig = './figures/relative-pose/'
        if not os.path.exists(path_fig):
            os.makedirs(path_fig)
        plt.savefig(path_fig + 'weight_time_avg_' + str(rho) + '.pdf', dpi=600, bbox_inches='tight')
        plt.clf()

def each_meas_mean_(_list, tag):
    '''
    output RMSE of each robot over tau

    :param: _list: dict, the dict of RMSE of each robot
    :param: tag: str, the tag of metric, 'R' for RMSE, 'A' for ANEES
    '''
    path_fig = './figures/relative-pose/'
    if not os.path.exists(path_fig):
        os.makedirs(path_fig)
    with open(path_fig + tag + '_each_' + str(parameters.comm_fail_prob) + '.csv', 'w') as f:
        for type, List in _list.items():
            f.write(str(type) + '\n')
            np.savetxt(f, List, fmt='%.4f', newline=',')
            
            f.write('\n')

def bias_mean_(numbers, Lists, std, tag):
    '''
    draw ARMSE of all algorithms over tau

    :param: numbers: int, the number of simulation time
    :param: Lists: dict, the dict of ARMSE-mean of each robot
    :param: std: dict, the dict of ARMSE-std of each robot
    :param: tag: str, the tag of metric, 'R' for RMSE, 'A' for ANEES
    '''

    plt.style.use(['science', 'ieee', 'muted', 'no-latex'])
    path = './data/relative-pose/'
    if not os.path.exists(path):
        os.makedirs(path)
    prob=np.arange(numbers)/20
    fig3, ax3 = plt.subplots(1,1)
    count = 0
    with open(path + tag + '__' + str(parameters.comm_fail_prob) + '.txt', 'w') as f:
        for type, List in Lists.items():
            line, = ax3.plot(prob,List, label=LABELS[type], marker = MARKER_STYLE[count], markersize = 3, c = EACH_COLOR[count], linewidth = 2) #
            
            f.write(str(type) + '\t')
            np.savetxt(f, List, fmt='%.4f', newline=' ')
            
            f.write('\n')
            
            count = count + 1

    ax3.text(0.25, 3.0, r'$\rho = %.1f$' % parameters.comm_fail_prob, color='k', ha='center', va='center')
        
    ax3.set_xlabel(r'occurrence probability of biased measurements $\tau$')
    ax3.legend(loc="upper left")
    
    path_fig = './figures/relative-pose/'
    if not os.path.exists(path_fig):
        os.makedirs(path_fig)
    if tag == 'R':
        ax3.set_ylabel('ARMSE [m]')
        ax3.set_ylim(bottom=0, top=3.2)
        plt.savefig(path_fig + 'RMSE__' + str(parameters.comm_fail_prob) + '.pdf', dpi=600, bbox_inches='tight')
    elif tag == 'A':
        ax3.set_ylabel('ANEES')
        ax3.plot([0, prob[-1]], [2, 2], 'k--')
        ax3.set_ylim(bottom=0, top=15)
        plt.savefig(path_fig + 'ANEES__' + str(parameters.comm_fail_prob) +  '.pdf', dpi=600, bbox_inches='tight')