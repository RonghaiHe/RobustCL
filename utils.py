import os
import numpy as np
import math
import parameters
import draw
from algorithms.DR import Robot, Robot_true
from algorithms.BDA import Robot_BDA_EKF
from algorithms.DMV import Robot_DMV
from algorithms.CU import Robot_BDA_EKF_CU, Robot_CI_CU
from algorithms.DCL_GS import Robot_GS_LRHKF

R_0 = parameters.R_0
R_1 = parameters.R_1

MU_RANGE_BEARING, MU_POSE = parameters.MU_RANGE_BEARING, parameters.MU_POSE
BIAS_SIGMA_RANGE_BEARING, BIAS_SIGMA_POSE = parameters.BIAS_SIGMA_RANGE_BEARING, parameters.BIAS_SIGMA_POSE

comm_fail_prob = parameters.comm_fail_prob
COMM_RATE = parameters.COMM_RATE

SIGMA_V_INPUT = parameters.SIGMA_V_INPUT
SIGMA_OMEGA_INPUT = parameters.SIGMA_OMEGA_INPUT
E_V = parameters.E_V
E_OMEGA = parameters.E_OMEGA

V_MAX = parameters.V_MAX
V_MIN = parameters.V_MIN
OMEGA_MAX = parameters.OMEGA_MAX
OMEGA_MIN = parameters.OMEGA_MIN

LANDMARK_POS = parameters.LANDMARK_POS
DELTA_T = parameters.DELTA_T

Q_BACKUP = parameters.Q
OMEGA_MAX_BACKUP, OMEGA_MIN_BACKUP = OMEGA_MAX, OMEGA_MIN

def update_para():
    global OMEGA_MAX, OMEGA_MIN, SIGMA_OMEGA_INPUT
    OMEGA_MAX = parameters.OMEGA_MAX
    OMEGA_MIN = parameters.OMEGA_MIN
    SIGMA_OMEGA_INPUT = parameters.SIGMA_OMEGA_INPUT

def draw_RMSE(ax, list_times, list_RMSE, str_color, str_label, bool_label):
    """
    draw RMSE(in mission 4/5)
    :param ax: axis
    :param list_times: time list
    :param list_RMSE: RMSE list
    :param str_color: color
    :param str_label: label
    :param bool_label: whether to show label
    """
    if bool_label:
        ax.plot(list_times, list_RMSE, '-', linewidth = 2, c=str_color, label=str_label)
    else:
        ax.plot(list_times, list_RMSE, '-', linewidth = 2, c=str_color)

def storage(algs_robots, dr_robots, cen_robots=[]):
    """
    store the data
    :param algs_robots: list of algorithms
    :param dr_robots: list of DR
    :param cen_robots: list of centralized CL
    """
    RMSE, ANEES, M = {}, {}, {}
    for type, alg_robots in algs_robots.items():
        RMSE[type] = []
        ANEES[type] = []
        for robot_i in alg_robots:
            RMSE[type].append(robot_i.RMSE_list)
            ANEES[type].append(robot_i.ANEES_list)
        
        if type in {28}:
            M[type] = []
            for robot_i in alg_robots:
                M[type].append(robot_i.psi_y_time)
    
    if dr_robots is not None:
        RMSE[-1] = []
        ANEES[-1] = []
        for robot_i in dr_robots:
            RMSE[-1].append(robot_i.RMSE_list)
            ANEES[-1].append(robot_i.ANEES_list)

    # Pay attention to the format (Transpose)
    # RMSE[-2] = cen_robots.RMSE_list
    # ANEES[-2] = cen_robots.ANEES_list
    
    return RMSE, ANEES, M

def run(init_X, numbers=10000, flag=0, types = [-1, 0], comm_fail_prob = 0, meas_bia_prob = 0, Seed = 0, mission = 0):
    
    '''
    Parameters
    ----
    init_X: numpy.array NUM_ROBOTS*3
        Init position of each robots
    numbers: int
        total simulation time
    flag: int, observation model
        1 range-bearing and transform to relative position
        0 range-bearing
        -1 relative pose
    types: int
        -1 Dead Reckon(DR)
        0 Decentralized EKF(BDA)
        2 Decentralized EKF with CU(DCL-CU)
        6 Decentralized EKF(DMV)        

        12 CI-CU

        28 multi-centralized + M-estimation
    Seed: int
        random seed
    '''
    print(f'{comm_fail_prob}: Seed={Seed}')
    np.random.seed(Seed)    

    print(f"meas_bia_prob = {meas_bia_prob}, comm_fail_prob = {comm_fail_prob}")

    NUM_ROBOTS = parameters.NUM_ROBOTS
    LANDMARK_NUM = parameters.LANDMARK_NUM
    n2 = len(types) # the number of algorithms

    # Initialize
    # GroundTruth of each robot
    true_robots = [Robot_true(X=init_X[r], _id = r) for r in range(NUM_ROBOTS)]

    algs_robots = {}
    if n2: 
        for type in types:
            if type == 0:
                algs_robots[0] = [Robot_BDA_EKF(X=init_X[r], _id=r, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS) for r in range(NUM_ROBOTS)]
            elif type == 2:    
                algs_robots[2] = [Robot_BDA_EKF_CU(X=init_X[r], _id=r, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS) for r in range(NUM_ROBOTS)]
            elif type == 6:    
                algs_robots[6] = [Robot_DMV(X=init_X[r], _id=r, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS) for r in range(NUM_ROBOTS)]
            elif type == 12:
                algs_robots[12] = [Robot_CI_CU(X=init_X[r], _id=r, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS) for r in range(NUM_ROBOTS)]
            elif type == 28:    
                algs_robots[28] = [Robot_GS_LRHKF(initial_s=init_X, _id=r, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS) for r in range(NUM_ROBOTS)]   
            
    dr_robots = None           
    if -1 in types:
        dr_robots = [Robot(X=init_X[r], _id=r, NUM_ROBOTS=NUM_ROBOTS) for r in range(NUM_ROBOTS)]
    
    # cen_robots = Robots_CCL(X=init_X[0:NUM_ROBOTS], flag = flag)
    
    if mission in {4, 5}:
        list_times = []
        list_RMSE = {}
        draw.plt.style.use(['science', 'ieee', 'no-latex'])
        fig2, ax2 = draw.plt.subplots(1,1)
        
        for robot_i in true_robots:
            robot_i.draw(ax2)
            robot_i.plot_shape(ax2)
        cou = 0
        for type in types:
            list_RMSE[type] = []
            if not type == -1:
                for robot_i in algs_robots[type]:
                    robot_i.draw(ax2, draw.EACH_COLOR[cou], draw.LABELS[type])
                cou += 1
        if -1 in types:
            list_RMSE[-1] = []
            for robot_i in dr_robots:
                robot_i.draw(ax2, draw.EACH_COLOR[cou], draw.LABELS[-1])
        ax2.legend(loc = 'upper left')

        ax2.axis('equal')
        if comm_fail_prob == 0.5:
            ax2.text(0.75, 0.9, 'Time: 0s', transform = ax2.transAxes)
            ax2.text(0.75, 0.85, r'$\tau={:.{}f}$'.format(meas_bia_prob, 2), transform = ax2.transAxes)

        ax2.text(0.75, 0.05, r'$\rho=$' + str(comm_fail_prob), transform = ax2.transAxes)
        ax2.set_xlabel(r'$x$ [m]')
        if comm_fail_prob == 0.1:
            ax2.set_ylabel(r'$y$ [m]')

        path1 = './video-' + str(mission) + '/Tra-' + str(comm_fail_prob)
        if not os.path.exists(path1):
            os.makedirs(path1)
        draw.plt.savefig(path1 + '/0000.png', dpi=600, bbox_inches='tight')
        draw.plt.close()
        
        draw.plt.style.use(['science', 'ieee', 'no-latex'])
        fig, ax = draw.plt.subplots(1,1)
        if comm_fail_prob == 0.1:
            ax.set_ylabel('ARMSE [m]')
        ax.set_xlabel('simulation time [s]')
        ax.text(0.45, 0.95, r'$\rho=$' + str(comm_fail_prob), transform = ax.transAxes)
        path2 = './video-' + str(mission) + '/' + str(comm_fail_prob)
        if not os.path.exists(path2):
            os.makedirs(path2)
        draw.plt.savefig(path2 + '/0000.png', dpi=600, bbox_inches='tight')
        draw.plt.close()


    broadcast_comm_his_GS = {}
    for type in types:
        if type >= 10:
            broadcast_comm_his_GS[type] = np.zeros((NUM_ROBOTS,NUM_ROBOTS), dtype = np.uint16)
    
    change_para = [False, False]
    if mission in [1,2,4]:
        change_para = [True, True]
    elif mission == 5:
        change_para = [False, True]

    for count in range(numbers):
        
        # # A part to change some parameters
        # if change_para[0]:
        #     if 4*count > numbers:

        #         parameters.Q = np.diag(np.tile([.7, .3], NUM_ROBOTS))
        #         parameters.OMEGA_MAX = 1
        #         parameters.OMEGA_MIN = -1
        #         parameters.update_para()
        #         update_para()
        #         need_change_type = [False, False]
        #         for type in types:
        #             if type >= 20 and not need_change_type[1]:
        #                 from algorithms.DCL_GS import update_para as DCL_GS_up
        #                 DCL_GS_up(parameters.Q, parameters.SIGMA0_OMEGA2, parameters.SIGMA_OMEGA2)
        #                 need_change_type[1] = True
        #             elif type < 20 and not need_change_type[0]:
        #                 from algorithms.DR import update_para as DR_up
        #                 DR_up(parameters.Q)
        #                 need_change_type[0] = True
        #         change_para[0] = False
        Rv = 0
        if change_para[1]:
            if change_para[0]:
                if 2*count >= numbers: 
                    meas_bia_prob = 0.5
                    
                    change_para[1] = False # change once
            else: # mission 5
                meas_bia_prob = 0.05 * int(count / 91)
        
        if change_para[0]:
            Rv = 1.8e-3*count - 0.8
        else: Rv = parameters.Rv

############## Motion step starts ########################
        
        #### The INPUT velocity and angular velocity follows the normal distribution
        _input = [[np.random.randn()*SIGMA_V_INPUT[r] + E_V[r], np.random.randn()*SIGMA_OMEGA_INPUT[r] + E_OMEGA[r]] for r in range(NUM_ROBOTS)]
        # _input = [[0.2, np.random.randn()*SIGMA_OMEGA_INPUT[r] + E_OMEGA[r]] for r in range(NUM_ROBOTS)]
        for r in range(NUM_ROBOTS):
            if abs(_input[r][0] - E_V[r]) > 3*SIGMA_V_INPUT[r]: _input[r][0] = E_V[r]
            if abs(_input[r][1] - E_OMEGA[r]) > 3*SIGMA_OMEGA_INPUT[r]: _input[r][1] = E_OMEGA[r]

        for type in algs_robots.keys():
            for robot_i in algs_robots[type]:
                robot_i.reset_rela()
                robot_i.motion( v=_input[robot_i._id][0],omega = _input[robot_i._id][1])
        if -1 in types:
            for robot_i in dr_robots:
                robot_i.motion( v=_input[robot_i._id][0],omega = _input[robot_i._id][1])
                
        # cen_robots.motion(odometry_input=input)

        noises_move = np.random.randn(NUM_ROBOTS, 2) # random for noise
        for robot_i in true_robots:
            robot_i.update( v=_input[robot_i._id][0],omega = _input[robot_i._id][1], 
                           noise=noises_move[robot_i._id]) # _id must from 0 to NUM_ROBOTS-1

############ observation step  starts ############

        # Landmark, Absolute observation
        # Generate the noise
        measure_noises = np.random.randn(NUM_ROBOTS*(LANDMARK_NUM), 3)
        measure_bias_happen = np.random.rand(NUM_ROBOTS*(LANDMARK_NUM))
        measure_bias_whether = np.zeros(NUM_ROBOTS*(LANDMARK_NUM), dtype=bool) # whether the observation is biased

        for ind in range(NUM_ROBOTS*(LANDMARK_NUM)):
            if(measure_bias_happen[ind] < meas_bia_prob):
                if(flag == -1): measure_noises[ind,:] = measure_noises[ind,:] * BIAS_SIGMA_POSE + MU_POSE * Rv
                elif(flag >= 0): measure_noises[ind, :2] = measure_noises[ind,:2] * BIAS_SIGMA_RANGE_BEARING + MU_RANGE_BEARING * Rv
                measure_bias_whether[ind] = True
            else:
                if(flag == -1): measure_noises[ind,:] = measure_noises[ind,:] * np.array([R_1[0,0], R_1[1,1], R_1[2,2]])
                elif(flag >= 0): measure_noises[ind,:2] = measure_noises[ind,:2] * np.array([R_0[0,0], R_0[1,1]])

        for type in algs_robots.keys():
            for robot_i in algs_robots[type]:
                robot_i.measurement_abso(cla_trues = true_robots, 
                                    measure_noises = measure_noises[robot_i._id*(LANDMARK_NUM):robot_i._id*(LANDMARK_NUM)+(LANDMARK_NUM),:], 
                                    measure_bias_whether = measure_bias_whether[robot_i._id*(LANDMARK_NUM):robot_i._id*(LANDMARK_NUM)+(LANDMARK_NUM)])

                robot_i.abso_meas_correct(count)
                robot_i.reset_abso()


        measure_noises = np.random.randn(NUM_ROBOTS*(NUM_ROBOTS), 3)
        measure_bias_happen = np.random.rand(NUM_ROBOTS*(NUM_ROBOTS))
        measure_bias_whether = np.zeros(NUM_ROBOTS*(NUM_ROBOTS), dtype=bool) # whether the observation is biased
        
        for ind in range(NUM_ROBOTS*(NUM_ROBOTS)):
            if(measure_bias_happen[ind] < meas_bia_prob):
                if(flag == -1): measure_noises[ind,:] = measure_noises[ind,:] * BIAS_SIGMA_POSE + MU_POSE * Rv
                elif(flag >= 0): measure_noises[ind, :2] = measure_noises[ind,:2] * BIAS_SIGMA_RANGE_BEARING + MU_RANGE_BEARING * Rv
                measure_bias_whether[ind] = True
            else:
                if(flag == -1): measure_noises[ind,:] = measure_noises[ind,:] * np.array([R_1[0,0], R_1[1,1], R_1[2,2]])
                elif(flag >= 0): measure_noises[ind,:2] = measure_noises[ind,:2] * np.array([R_0[0,0], R_0[1,1]])


        # Relative observaion
        # Only Resilient systems: when measure, precise later whether commuting or not
        for type in algs_robots.keys():
            if type < 20: continue
            for robot_i in algs_robots[type]:
                robot_i.measurement_rela(cla_trues = true_robots, 
                                    measure_noises = measure_noises[robot_i._id*(NUM_ROBOTS):robot_i._id*(NUM_ROBOTS)+(NUM_ROBOTS),:], 
                                    measure_bias_whether = measure_bias_whether[robot_i._id*(NUM_ROBOTS):robot_i._id*(NUM_ROBOTS)+(NUM_ROBOTS)])

                robot_i.rela_meas_correct(count)


        # Observation and communication which couples
        # Observation every 10 times
        # n2>0 -> have decentralized
        if((not (count+1) % COMM_RATE) and n2):
            
            for type in algs_robots.keys():
                if type >= 20: continue
                for robot_i in algs_robots[type]:
                    robot_i.measurement_rela(cla_trues = true_robots, 
                                        measure_noises = measure_noises[robot_i._id*(NUM_ROBOTS):robot_i._id*(NUM_ROBOTS)+(NUM_ROBOTS),:], 
                                        measure_bias_whether = measure_bias_whether[robot_i._id*(NUM_ROBOTS):robot_i._id*(NUM_ROBOTS)+(NUM_ROBOTS)])

            # To keep the noise in the same values
            communicate_noises = np.random.rand(NUM_ROBOTS**2, 60)
            
            # Centralized
            # for __ in range(NUM_ROBOTS):
            #     cen_robots.reset()
            #     cen_robots.observation(_id = __, cla_trues=true_robots, measure_noises = measure_noises[__*NUM_ROBOTS:__*NUM_ROBOTS+NUM_ROBOTS,:])

            #     if communicate_noises[__*NUM_ROBOTS+__, 0] > comm_fail_prob:
            #         cen_robots.rela_meas_ekf(_id = __)

############ communication step  starts ############
            
            # This's communicate time needed, assumed that they measured at the same time
            # NUM_ROBOTS**2 considers more communications
            # Neglect the computation times nd the check the broadcasts times and the times sending message to the broadcasts 
            communicate_time = np.random.rand(NUM_ROBOTS*60)*DELTA_T*2
            

            # circle all algorithms
            for type in algs_robots.keys():
                # No_more_robots = set()
                communicate_time_copy = communicate_time.copy()
                robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                comm_times = np.zeros(NUM_ROBOTS, dtype=np.uint8) # only for picking which index in communicate_time_copy
                send_times = np.zeros((NUM_ROBOTS, NUM_ROBOTS), dtype=int) # Times about send ing the message
                if type in {0, 1, 3, 4}: # DCL request, only EKF at one robot
                    
                    asked4belief_before = np.ones(NUM_ROBOTS, dtype=int)*-1 # index is asked for belief before filtering by value
                    # received_after = np.zeros(NUM_ROBOTS, dtype=bool) # Received after filtering, and work, so no repeated

                    broadcast_matched = np.zeros(NUM_ROBOTS, dtype=bool) # For match
                    send_back_index = np.ones(NUM_ROBOTS, dtype=int)*-1 # Index
                                        
                    
                    # X_current = np.zeros((NUM_ROBOTS, 3)) # Current pose
                    # P_current = np.zeros((NUM_ROBOTS, 3, 3)) # Current covariance
                    # for robot_i in algs_robots[type]:
                    #     X_current[robot_i._id] = robot_i.X.copy()
                    #     P_current[robot_i._id] = robot_i.P.copy()

                    '''
                    If robot i measures robot j, 3 steps happen:
                    1. robot i asks robot j for robot j's belief
                    2. robot j sends its belief to robot i
                    3. robot i sends robot j's belief after filter to robot j

                    Q: Why replacing the origin communicate_time_copy with new feedback time?
                    Answer: Because the origin sending-message time is later than now, 
                    and the communication must happen one2one.
                    '''

                    while communicate_time_copy[robot_comm_order[0]] < DELTA_T*COMM_RATE:
                        robot_i = algs_robots[type][robot_comm_order[0]]
                        if broadcast_matched[robot_i._id]:
                            # Send my own belief and ask your belief(if needed) (2rd)
                            if asked4belief_before[robot_i._id] >= 0:
                                r = asked4belief_before[robot_i._id]
                                if communicate_noises[robot_i._id*NUM_ROBOTS + r, send_times[robot_i._id, r]] > comm_fail_prob:
                                    asked4belief_before[robot_i._id] = -1
                                    can = algs_robots[type][r].communicate2_1(robot_j = robot_i, get_other=False)
                                    
                                    if(robot_i._id in algs_robots[type][r].sort_pair_request): 
                                        algs_robots[type][r].sort_pair_request.remove(robot_i._id)
                                    
                                    if can:
                                        algs_robots[type][r].pair_history[robot_i._id] = count
                                        
                                    communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                    comm_times[r] += 1
                                    communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                                    
                                    if robot_i.measuring[r] or can:
                                        # NEED...
                                        # Next, r sends its belief to robot_i, whatever
                                        
                                        send_back_index[r] = robot_i._id
                                        communicate_time_copy[robot_comm_order[0]] += 1
                                    else:
                                        broadcast_matched[r], broadcast_matched[robot_i._id] = False, False
                                        comm_times[robot_comm_order[0]] += 1
                                        communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                                else:
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                    comm_times[robot_comm_order[0]] += 1
                                    communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                
                                send_times[robot_i._id, r] += 1
                                # robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])

                            # r is major, robot_i, who? just sender(3rd)
                            elif send_back_index[robot_i._id] >= 0:
                                
                                r = send_back_index[robot_i._id]
                                # Receive the belief(fused or not), r maybe fuse or just replace(3rd)
                                if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                    # r receives robot_i's belief(3nd)
                                    
                                    can = 0
                                    if algs_robots[type][r].measuring[robot_i._id]:
                                        can = algs_robots[type][r].communicate2_1(robot_j = robot_i, get_other=False)
                                        if(robot_i._id in algs_robots[type][r].sort_pair_request): algs_robots[type][r].sort_pair_request.remove(robot_i._id)
                                    
                                    # NEED: ranges from 0 to NUM_ROBOTS-1
                                    communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                    send_back_index[robot_i._id] = -1

                                    # If succeed
                                    if can:
                                        algs_robots[type][r].pair_history[robot_i._id] = count
                                    
                                    # If just robot_i succeed
                                    # else:
                                    else:
                                        algs_robots[type][r].communicate2_2(robot_i = robot_i)

                                    broadcast_matched[r], broadcast_matched[robot_i._id] = False, False # For the next match

                                    algs_robots[type][r].X_prediction = algs_robots[type][r].X.copy()
                                    algs_robots[type][r].P_prediction = algs_robots[type][r].P.copy()
                                    robot_i.X_prediction, robot_i.P_prediction = robot_i.X.copy(), robot_i.P.copy()

                                    comm_times[r] += 1
                                    communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]                                        

                                    
                                    send_back_index[robot_i._id] = -1

                                    
                                # Fail to send, and resend at the next time
                                else:
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                                comm_times[robot_comm_order[0]] += 1
                                communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                
                                send_times[robot_i._id, r] += 1
                                
                            
                                
                            robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                            
                            continue

                        pick = 0
                        while(len(robot_i.sort_pair_request)):
                            r = robot_i.sort_pair_request[pick]
                            if broadcast_matched[r]:
                                pick += 1
                                if pick == len(robot_i.sort_pair_request): 
                                    pick = -1
                                    # robot_i.sort_pair_request = []
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                    comm_times[robot_comm_order[0]] += 1
                                    communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                                    robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                                    # No_more_robots.add(robot_i._id)
                                    break
                            else: break
                        
                        if pick >=0 and len(robot_i.sort_pair_request):
                            # Request (1st)
                            if robot_i.measuring[r] and communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                asked4belief_before[r] = robot_i._id
                                broadcast_matched[r], broadcast_matched[robot_i._id] = True, True # For match
                                
                                communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                # waiting ...
                                communicate_time_copy[robot_comm_order[0]] += 1
                                # NEED: Robot's index ranges from 0 to NUM_ROBOTS-1
                                comm_times[r] += 1
                                # communicate_time_copy add communicate times: r->robot_i
                                communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                            
                            # Cannot send the message to that robot, waiting time and send at the next time
                            else:
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                comm_times[robot_comm_order[0]] += 1
                                communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                            send_times[robot_i._id, r] += 1
                        
                        # Not any other robot needed to EKF
                        else: communicate_time_copy[robot_comm_order[0]] += 2
                        
                        robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                
                
                elif type in {2, 5}: # (DCL no request, origin) now: BDA-CU with request, match
                    asked4belief_before = np.ones(NUM_ROBOTS, dtype=int)*-1 # index is asked for belief before filtering by value
                    broadcast_matched = np.zeros(NUM_ROBOTS, dtype=bool) # For match
                    broadcast_filtered = np.zeros((NUM_ROBOTS, NUM_ROBOTS), dtype=bool) # For match
                    
                    # Value>=0 -> index-CONNECTION-build_connection[index]
                    build_connection = np.ones(NUM_ROBOTS, dtype=int)*-1

                    # Value>=0 means index should send info back to Value
                    send_back_index = np.ones(NUM_ROBOTS, dtype=int)*-1 # Index

                    # Value>=0: Just for CU
                    send4CU = np.ones(NUM_ROBOTS, dtype=int)*-1 # Index

                    while communicate_time_copy[robot_comm_order[0]] < DELTA_T*COMM_RATE:
                        robot_i = algs_robots[type][robot_comm_order[0]]
                        if broadcast_matched[robot_i._id]:
                            # Send my own belief and ask your belief(if needed) (2rd)
                            if asked4belief_before[robot_i._id] >= 0:
                                r = asked4belief_before[robot_i._id]
                                if communicate_noises[robot_i._id*NUM_ROBOTS + r, send_times[robot_i._id, r]] > comm_fail_prob:
                                    asked4belief_before[robot_i._id] = -1
                                    can = algs_robots[type][r].communicate2_1(robot_j = robot_i, get_other=False)
                                    
                                    if(robot_i._id in algs_robots[type][r].sort_pair_request): 
                                        algs_robots[type][r].sort_pair_request.remove(robot_i._id)
                                    
                                    if can:
                                        broadcast_filtered[r, robot_i._id] = True
                                        algs_robots[type][r].pair_history[robot_i._id] = count
                                        
                                    communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                    comm_times[r] += 1
                                    communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                                    
                                    if robot_i.measuring[r] or can:
                                        # NEED...
                                        # Next, r sends its belief to robot_i, whatever
                                        
                                        send_back_index[r] = robot_i._id
                                        communicate_time_copy[robot_comm_order[0]] += 1
                                    else:
                                        broadcast_matched[r], broadcast_matched[robot_i._id] = False, False
                                        comm_times[robot_comm_order[0]] += 1
                                        communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                else:
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                    comm_times[robot_comm_order[0]] += 1
                                    communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                
                                send_times[robot_i._id, r] += 1
                                # robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])

                            # r is major, robot_i, who? just sender(3rd)
                            elif send_back_index[robot_i._id] >= 0:
                                
                                r = send_back_index[robot_i._id]
                                # Receive the belief(fused or not), r maybe fuse or just replace(3rd)
                                if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                    # r receives robot_i's belief(3nd)
                                    
                                    can = 0
                                    if algs_robots[type][r].measuring[robot_i._id]:
                                        can = algs_robots[type][r].communicate2_1(robot_j = robot_i, get_other=False)
                                        if(robot_i._id in algs_robots[type][r].sort_pair_request): algs_robots[type][r].sort_pair_request.remove(robot_i._id)
                                    
                                    # NEED: ranges from 0 to NUM_ROBOTS-1
                                    communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                    send_back_index[robot_i._id] = -1

                                    # If succeed
                                    if can:
                                        broadcast_filtered[r, robot_i._id] = True
                                        algs_robots[type][r].pair_history[robot_i._id] = count
                                        
                                        # If robot_i also succeed
                                        if broadcast_filtered[robot_i._id, r]:
                                            algs_robots[type][r].communicate2_2CU(robot_i = robot_i)
                                        
                                        # NEED: ranges from 0 to NUM_ROBOTS-1
                                        # communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                        
                                        send4CU[r] = robot_i._id
                                        communicate_time_copy[robot_comm_order[0]] += 1
                                        # send_back_index[r] = robot_i._id
                                    
                                    # If just robot_i succeed
                                    # else:
                                    elif broadcast_filtered[robot_i._id, r]:
                                        algs_robots[type][r].communicate2_2(robot_i = robot_i)
                                        # All fails
                                        broadcast_matched[r], broadcast_matched[robot_i._id] = False, False # For the next match

                                        algs_robots[type][r].X_prediction = algs_robots[type][r].X.copy()
                                        algs_robots[type][r].P_prediction = algs_robots[type][r].P.copy()
                                        robot_i.X_prediction, robot_i.P_prediction = robot_i.X.copy(), robot_i.P.copy()

                                        comm_times[robot_comm_order[0]] += 1
                                        communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                    
                                    comm_times[r] += 1
                                    communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]                                        

                                    
                                    send_back_index[robot_i._id] = -1

                                    
                                # Fail to send, and resend at the next time
                                else:
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                                    comm_times[robot_comm_order[0]] += 1
                                    communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                    
                                
                                send_times[robot_i._id, r] += 1
                                
                                # Receive the fused estimate or not(3rd). r needs to CU or =
                                # this means that r has received robot_i's info before because 2nd(4th)
                            elif send4CU[robot_i._id] >= 0:
                                r = send4CU[robot_i._id]
                                if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                    # both can EKF, CU
                                    if broadcast_filtered[r, robot_i._id] and broadcast_filtered[robot_i._id, r]:
                                        algs_robots[type][r].communicate2_2CU(robot_i = robot_i)
                                    
                                    broadcast_matched[r], broadcast_matched[robot_i._id] = False, False # For the next match
                                    algs_robots[type][r].X_prediction = algs_robots[type][r].X.copy()
                                    algs_robots[type][r].P_prediction = algs_robots[type][r].P.copy()
                                    robot_i.X_prediction, robot_i.P_prediction = robot_i.X.copy(), robot_i.P.copy()

                                    
                                    send4CU[robot_i._id] = -1
                                    # NEED: ranges from 0 to NUM_ROBOTS-1
                                    communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                    
                                    comm_times[r] += 1
                                    communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                                    
                                # Fail to send and resend at the next time
                                else:
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                                comm_times[robot_comm_order[0]] += 1
                                communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                                send_times[robot_i._id, r] +=  1
                                              
                            robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])                            
                            continue

                        pick = 0
                        while(len(robot_i.sort_pair_request)):
                            r = robot_i.sort_pair_request[pick]
                            if broadcast_matched[r]:
                                pick += 1
                                if pick == len(robot_i.sort_pair_request): 
                                    pick = -1
                                    communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                    comm_times[robot_comm_order[0]] += 1
                                    communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                                    robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                                    break
                            else: break
                        
                        if pick >=0 and len(robot_i.sort_pair_request):
                            # Request (1st)
                            if robot_i.measuring[r] and communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                asked4belief_before[r] = robot_i._id
                                broadcast_matched[r], broadcast_matched[robot_i._id] = True, True # For match
                                
                                communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                # waiting ...
                                communicate_time_copy[robot_comm_order[0]] += 1
                                # NEED: Robot's index ranges from 0 to NUM_ROBOTS-1
                                comm_times[r] += 1
                                # communicate_time_copy add communicate times: r->robot_i
                                communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                            
                            # Cannot send the message to that robot, waiting time and send at the next time
                            else:
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                comm_times[robot_comm_order[0]] += 1
                                communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                            send_times[robot_i._id, r] += 1
                                                    
                        # Not any other robot needed to EKF
                        else: communicate_time_copy[robot_comm_order[0]] += 2
                        
                        robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                    
                    # print('a')

                elif type in {6, 7}: # Zhu_Kia from paper, both robot EKF both measurement(If it is, not others needed)
                    broadcast_matched = np.zeros(NUM_ROBOTS, dtype=bool) # For match
                    
                    # send_back_index[i] = j, means robot_i sends its belief to robot_j
                    send_back_index = np.ones(NUM_ROBOTS, dtype=int)*-1
                    
                    # Still can communicate
                    while communicate_time_copy[robot_comm_order[0]] < DELTA_T*COMM_RATE:
                        # which robot's sending message could be received first
                        robot_i = algs_robots[type][robot_comm_order[0]]
                        
                        
                        if send_back_index[robot_i._id] >= 0:
                            # robot_i has received the belief and measurement before, 
                            # then this time should send back(2nd)
                            
                            # r = That robot's _id. The one robot_i sends to
                            r = send_back_index[robot_i._id]
                            if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                # Successfully send back
                                
                                can_2 = algs_robots[type][r].communicate1_1(robot_j = robot_i)
                                if can_2: algs_robots[type][r].pair_history[robot_i._id] = count
                                algs_robots[type][r].sort_pair_request.remove(robot_i._id)

                                # Anyway, cancel it, meaning having received it, both robots can update prediction
                                broadcast_matched[r], broadcast_matched[robot_i._id] = False, False
                                send_back_index[robot_i._id] = -1

                                # Update both prediction believes
                                robot_i.X_prediction = robot_i.X.copy()
                                robot_i.P_prediction = robot_i.P.copy()
                                algs_robots[type][r].X_prediction = algs_robots[type][r].X.copy()
                                algs_robots[type][r].P_prediction = algs_robots[type][r].P.copy()

                                # That robot no need to wait, continue to send
                                communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]]
                                comm_times[r] += 1
                                communicate_time_copy[r] += communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]                                
                            
                            else:
                                # waiting the change of broadcast_matched
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                            
                            # Anyway, robot_i's communication arrival times need to update
                            comm_times[robot_comm_order[0]] += 1
                            communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                            
                            send_times[robot_i._id, r] += 1
                        
                        # for r in robot_i.sort_pair_request:
                        elif(len(robot_i.sort_pair_request)):
                            # robot_i measures other robots and looks for the 1st one to communicate

                            r = robot_i.sort_pair_request[0]

                            can_2 = 0
                            # Request, and send own pose(1st)
                            if robot_i.measuring[r] and communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                
                                # Anyway, broadcast them
                                broadcast_matched[r], broadcast_matched[robot_i._id] = True, True # For match
                                can_2 = algs_robots[type][r].communicate1_1(robot_j = robot_i)
                                # Can filter
                                if can_2: algs_robots[type][r].pair_history[robot_i._id] = count
                                
                                # r should send back its prediction belief
                                send_back_index[r] = robot_i._id
                                
                                                                
                                if robot_i._id not in algs_robots[type][r].sort_pair_request or \
                                    (robot_i._id in algs_robots[type][r].sort_pair_request and
                                     robot_i._id != algs_robots[type][r].sort_pair_request[0]):
                                        
                                        # Originally r does not send to robot_i
                                        # Now, feedback first, give up the original sending
                                        comm_times[r] += 1
                                        communicate_time_copy[r] = communicate_time_copy[robot_comm_order[0]] + communicate_time_copy[NUM_ROBOTS*comm_times[r] + r]
                                

                                if robot_i._id in algs_robots[type][r].sort_pair_request:
                                    algs_robots[type][r].sort_pair_request.remove(robot_i._id)
                                
                                # robot_i: waiting
                                communicate_time_copy[robot_comm_order[0]] += 1
                            
                            # Cannot send the message to that robot, still waiting for broadcast_matched
                            else: 

                                # robot_i resends
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]]+robot_comm_order[0]] - DELTA_T*2)
                                comm_times[robot_comm_order[0]] += 1
                                comm_times[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]

                            send_times[robot_i._id, r] += 1
                            
                        # robot_i has no one to communicate
                        else: communicate_time_copy[robot_comm_order[0]] += 2
                        
                        robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                                


                # only communicate once: CI-CU
                elif type >=10 and type < 20:
                    
                    for r in range(NUM_ROBOTS):
                        broadcast_comm_his_GS[type][r][r] = count
                        for _id in range(NUM_ROBOTS):
                            if _id != r and not algs_robots[type][r].measuring[_id]: broadcast_comm_his_GS[type][r][_id] += numbers

                    while communicate_time_copy[robot_comm_order[0]] < DELTA_T*COMM_RATE:
                        robot_i = algs_robots[type][robot_comm_order[0]]

                        comm_1st_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: broadcast_comm_his_GS[type][robot_i._id][i])
                        if(broadcast_comm_his_GS[type][robot_i._id][comm_1st_order[0]] == count):
                            # No more communication
                            communicate_time_copy[robot_comm_order[0]] += 1

                        else:
                            r = comm_1st_order[0]

                            # robot_i -> r, no matter success or fail
                            if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:


                                can = algs_robots[type][r].communicate1(robot_i)
                                # Received and update with others
                                if can: broadcast_comm_his_GS[type][robot_i._id][r] = count
                                # Received but cannot update, temporally no use
                                else: broadcast_comm_his_GS[type][robot_i._id][r] += numbers
                            else:
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                            
                            # Whether being received, the robot_i's X is updated and sent at the next time
                            robot_i.X_prediction = robot_i.X.copy()
                            robot_i.P_prediction = robot_i.P.copy()
                            
                            send_times[robot_i._id, r] += 1
                            comm_times[robot_comm_order[0]] += 1
                            communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                        robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                    broadcast_comm_his_GS[type] = [[val - numbers if val >= numbers else val for val in row] for row in broadcast_comm_his_GS[type]]

                
                # resilient
                elif type >= 20:
                    for r in range(NUM_ROBOTS):
                        broadcast_comm_his_GS[type][r][r] = count

                    while communicate_time_copy[robot_comm_order[0]] < DELTA_T*COMM_RATE:
                        robot_i = algs_robots[type][robot_comm_order[0]]
                        # sort the broadcast_comm_his_GS[type][robot_i._id]

                        comm_1st_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: broadcast_comm_his_GS[type][robot_i._id][i])
                        if(broadcast_comm_his_GS[type][robot_i._id][comm_1st_order[0]] == count):
                            # No more communication
                            communicate_time_copy[robot_comm_order[0]] += 1

                        else:
                            r = comm_1st_order[0]
                            # robot_i -> r, no matter success or fail
                            if communicate_noises[robot_i._id*NUM_ROBOTS+r, send_times[robot_i._id, r]] > comm_fail_prob:
                                can = algs_robots[type][r].communicate1_CI(robot_i.X_GS_prediction, robot_i.P_GS_prediction, robot_i._id)
                                if can: broadcast_comm_his_GS[type][robot_i._id][r] = count
                                else: broadcast_comm_his_GS[type][robot_i._id][r] += numbers
                            else:
                                communicate_time_copy[robot_comm_order[0]] -= (communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]] - DELTA_T*2)
                            
                            # Whether being received, the robot_i's X_GS is updated and sent at the next time
                            robot_i.X_GS_prediction = robot_i.X_GS.copy()
                            robot_i.P_GS_prediction = robot_i.P_GS.copy()
                            
                            send_times[robot_i._id, r] += 1
                            comm_times[robot_comm_order[0]] += 1
                            communicate_time_copy[robot_comm_order[0]] += communicate_time_copy[NUM_ROBOTS*comm_times[robot_comm_order[0]] + robot_comm_order[0]]
                        robot_comm_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: communicate_time_copy[:NUM_ROBOTS][i])
                    broadcast_comm_his_GS[type] = [[val - numbers if val >= numbers else val for val in row] for row in broadcast_comm_his_GS[type]]


############ communication step  ends############

        for robot_i in true_robots:
            robot_i.storage()

        for type in algs_robots.keys():
            for robot_i in algs_robots[type]:
                robot_i.comparison(cla_true=true_robots[robot_i._id])
                robot_i.storage()

        if -1 in types:                    
            for robot_i in dr_robots:
                robot_i.comparison(cla_true=true_robots[robot_i._id])
                robot_i.storage()
        
        # cen_robots.comparison(cla_true=true_robots)
        # cen_robots.storage()

        # # 将滤波后的信息返回到发送信息的机器人中
        # for robot_i in robots:
        #     if robot_i.pair>=0 and robots[robot_i.pair].measured:
        #         if np.random.rand()>.5: continue
        #         robot_i.communicate2(robots[robot_i.pair], type=type)
        
        # animate part 
        if mission in [4, 5]:

            list_times.append((count+1)*DELTA_T)

            # with each_step_cond: 
            draw.plt.style.use(['science', 'ieee', 'no-latex'])
            fig2, ax2 = draw.plt.subplots(1,1)
            for robot_i in true_robots:
                robot_i.draw(ax2)
                robot_i.plot_shape(ax2)
            cou = 0
            for type in types:
                RMSEs = []
                if not type == -1:
                    for robot_i in algs_robots[type]:
                        robot_i.draw(ax2, draw.EACH_COLOR[cou], draw.LABELS[type])
                        RMSEs.append(robot_i.RMSE)
                    cou += 1
                    list_RMSE[type].append(np.mean(RMSEs))
            if -1 in types:
                RMSEs = []
                for robot_i in dr_robots:
                    robot_i.draw(ax2, draw.EACH_COLOR[cou], draw.LABELS[-1])
                    RMSEs.append(robot_i.RMSE)
                list_RMSE[-1].append(np.mean(RMSEs))
            ax2.legend(loc = 'upper left')
            ax2.axis('equal')
            if comm_fail_prob == 0.5:
                ax2.text(0.75, 0.9, 'Time: {:.{}f}s'.format((count + 1)*DELTA_T, 1), transform = ax2.transAxes)
                ax2.text(0.75, 0.85, r'$\tau={:.{}f}$'.format(meas_bia_prob, 2), transform = ax2.transAxes)
            
            ax2.set_xlabel(r'$x$ [m]')
            if comm_fail_prob == 0.1:
                ax2.set_ylabel(r'$y$ [m]')    
            ax2.text(0.75, 0.05, r'$\rho=$' + str(comm_fail_prob), transform = ax2.transAxes)
            draw.plt.savefig('./video-' + str(mission) + '/Tra-' + str(comm_fail_prob) + '/' + str(count+1).zfill(4) + '.png', dpi=600, bbox_inches='tight')
            draw.plt.close()
            
            draw.plt.style.use(['science', 'ieee', 'no-latex'])
            fig, ax = draw.plt.subplots(1,1)
            cou = 0
            for type in types:
                
                if not type == -1:
                    for r in range(NUM_ROBOTS):
                        bool_label = False
                        if r<1:
                            bool_label = True
                        draw_RMSE(ax, list_times, list_RMSE[type], draw.EACH_COLOR[cou], draw.LABELS[type], bool_label)
                    cou += 1
            if -1 in types:
                for r in range(NUM_ROBOTS):
                    bool_label = False
                    if r<1:
                        bool_label = True
                    draw_RMSE(ax, list_times, list_RMSE[-1], draw.EACH_COLOR[cou], draw.LABELS[-1], bool_label)
            
            ax.text(0.45, 0.95, r'$\rho=$' + str(comm_fail_prob), transform = ax.transAxes)
            if mission == 4 and 2*count >= numbers:
                ax.plot([50, 50], [0, 0.25], 'k', linewidth = 2) # tau=0 vs 0.5
            ax.legend(loc = 'upper left')
            if comm_fail_prob == 0.1:
                ax.set_ylabel('ARMSE [m]')
            ax.set_xlabel('simulation time [s]')
            draw.plt.savefig('./video-' + str(mission) + '/' + str(comm_fail_prob) + '/' + str(count+1).zfill(4) + '.png', dpi=600, bbox_inches='tight')
            draw.plt.close()
                
    if mission in [0]:
        parameters.Q = Q_BACKUP.copy()
        parameters.OMEGA_MAX = OMEGA_MAX_BACKUP
        parameters.OMEGA_MIN = OMEGA_MIN_BACKUP
        parameters.update_para()
        update_para()
        need_change_type = [False, False]
        for type in types:
            if type >= 20 and not need_change_type[1]:
                from algorithms.DCL_GS import reset_para as DCL_GS_rp
                DCL_GS_rp()
                need_change_type[1] = True
            elif type < 20 and not need_change_type[0]:
                from algorithms.DR import reset_para as DR_rp
                DR_rp()
                need_change_type[0] = True

    RMSE_list, ANEES_list, M = storage(algs_robots=algs_robots, dr_robots=dr_robots)
    
    return RMSE_list, ANEES_list, M

def experiment1(args):
    '''
    Draw ARMSE with given type of algorithms over time

    :param args: parameters set at the beginning
    '''


    numbers = args['numbers']
    init_X = args['init_X']
    types = args['types']
    flag = args['flag']
    

    R_list, A_list= {}, {} # 20*10000
    total_R_list, total_A_list = {}, {} # 20*6*10000
    aver_R_list, aver_A_list = {}, {} # 6*10000
    R_, A_ = {}, {} # 1*10000
    global meas_bia_prob, comm_fail_prob
    comm_fail_prob = parameters.comm_fail_prob
    
    # Loop random seed to simulate each run
    for _ in range(20):
        ANEES_, RMSE_ = {}, {}
        print(f"Random seed = {_}")
        meas_bia_prob = parameters.meas_bia_prob
        RMSE_list, ANEES_list, M= run(init_X = np.array(init_X), numbers=numbers, flag=flag, types = types, comm_fail_prob = comm_fail_prob, meas_bia_prob = meas_bia_prob, Seed=_+0, mission=1)
        for type in RMSE_list.keys():
            
            if _ == 0:
                R_list[type] = []
                A_list[type] = []

                total_R_list[type] = []
                total_A_list[type] = []

            if not type == -2: 
                RMSE_[type] = np.mean(np.array(RMSE_list[type]), axis=0)
                R_list[type].append(RMSE_[type])
            else: 
                RMSE_[type] = np.mean(np.array(RMSE_list[type]), axis=1) # 10000*9
                R_list[type].append(RMSE_[type])

            total_R_list[type].append(RMSE_list[type])
            
            if not type == -2:
                ANEES_[type] = np.mean(np.array(ANEES_list[type]), axis=0)
                A_list[type].append(ANEES_[type])
            else:
                ANEES_[type] = np.mean(np.array(ANEES_list[type]), axis=1)
                A_list[type].append(ANEES_[type])

            total_A_list[type].append(ANEES_list[type])             
    
    # MEAN for running times, but divide each robot
    for type in total_R_list.keys():
        if not type == -2:
            aver_R_list[type] = np.mean(np.array(total_R_list[type]), axis = 0)
            aver_A_list[type] = np.mean(np.array(total_A_list[type]), axis = 0)
        else:
            aver_R_list[type] = np.mean(np.array(total_R_list[type]), axis = 0).T
            aver_A_list[type] = np.mean(np.array(total_A_list[type]), axis = 0).T


    # draw.each_mean_(numbers=numbers, _list = aver_R_list, tag = 'R')
    # draw.each_mean_(numbers=numbers, _list = aver_A_list, tag = 'A')
    
    # MEAN for running times, but totally
    for type in R_list.keys():
        # 1*10000
        R_[type] = np.mean(np.array(R_list[type]), axis=0)
        A_[type] = np.mean(np.array(A_list[type]), axis=0)

    
    draw.total_mean_(numbers=numbers, Lists = R_, tag='R')
    draw.total_mean_(numbers=numbers, Lists = A_, tag='A')
    
def experiment2(args):
    '''
    Draw average weight from M-estimation over time

    :param args: parameters set at the beginning
    '''


    numbers = args['numbers']
    init_X = args['init_X']
    types = args['types']
    flag = args['flag']
    

    M_list = {} # 20*10000
    total_M_list = {} # 20*6*10000
    aver_M_list = {} # 6*10000
    M_ = {} # 1*10000
    global meas_bia_prob, comm_fail_prob
    comm_fail_prob = parameters.comm_fail_prob
    
    for rho in [0.9, 0.5, 0.1]:
        comm_fail_prob = rho
        for _ in range(20):
            print(f"Random seed = {_}")
            meas_bia_prob = parameters.meas_bia_prob # 16 4
            # M: 6*10000*3
            RMSE_list, ANEES_list, M = run(init_X = np.array(init_X), numbers=numbers, flag=flag, types = types, comm_fail_prob = comm_fail_prob, meas_bia_prob = meas_bia_prob, Seed=_+0, mission=2)
            for type in RMSE_list.keys():
                
                if _ == 0 and rho == 0.9:
                    M_list[type] = {0.1: [], 0.5: [], 0.9: []}
                    total_M_list[type] = {0.1: [], 0.5: [], 0.9: []}

                M_list[type][rho].append(np.nanmean(np.array(M[type]), axis=0)) # 20*10000*3
                total_M_list[type][rho].append(M[type])
                
    # MEAN for running times, but divide each robot
    for type in total_M_list.keys():
        aver_M_list[type] = {0.1: np.nanmean(np.array(total_M_list[type][0.1]), axis = 0), 
                             0.5: np.nanmean(np.array(total_M_list[type][0.5]), axis = 0), 
                             0.9: np.nanmean(np.array(total_M_list[type][0.9]), axis = 0)
                             }


    # draw.each_mean_(numbers=numbers, _list = aver_M_list)
    
    # MEAN for running times, but totally
    for type in M_list.keys():
        # 1*10000
        M_[type] = {0.1: np.nanmean(np.array(M_list[type][0.1]), axis=0), 
                    0.5: np.nanmean(np.array(M_list[type][0.5]), axis=0), 
                    0.9: np.nanmean(np.array(M_list[type][0.9]), axis=0)
                    } # 10000*3

    
    draw.total_mean_M(numbers=numbers, Dicts = M_)

def experiment3(args):
    '''
    Draw time-averaged ARMSE with given type of algorithms over tau

    :param args: parameters set at the beginning
    '''

    global meas_bia_prob, comm_fail_prob
    meas_bia_prob = parameters.meas_bia_prob
    comm_fail_prob = parameters.comm_fail_prob

    numbers = args['numbers']
    init_X = args['init_X']
    types = args['types']
    flag = args['flag']
    
    R_list, A_list= {}, {} # 20*11
    total_R_list, total_A_list = {}, {} # 20*6*11
    aver_R_list, aver_A_list = {}, {} # 6*11
    R_, A_ = {}, {} # 1*11
    R_std, A_std = {}, {} # 1*11
    ANEES_, RMSE_ = {}, {}
    
    # _: simulation random seed
    for _ in range(20): 
        
        print(f"Random seed = {_}")
        for network in range(11):
            meas_bia_prob = network/20
            
            # 9*10000
            RMSE_list, ANEES_list, M = run(init_X = np.array(init_X), numbers=numbers, flag=flag, types = types, comm_fail_prob = comm_fail_prob, meas_bia_prob = meas_bia_prob, Seed=_, mission=3)
            for type in RMSE_list.keys():
                
                if type not in R_list:
                    R_list[type] = []
                    A_list[type] = []

                    total_R_list[type] = []
                    total_A_list[type] = []
                
                if np.shape(R_list[type])[0] - 1 < _:
                    R_list[type].append([])
                    A_list[type].append([])
                    total_R_list[type].append([])
                    total_A_list[type].append([])

                if not type == -2: 
                    RMSE_[type] = np.mean(np.mean(np.array(RMSE_list[type])))
                    R_list[type][_].append(RMSE_[type]) # scalar
                    total_R_list[type][_].append(np.mean(np.array(RMSE_list[type]), axis=1)) # 20*11*(9,)

                else: 
                    RMSE_[type] = np.mean(np.mean(np.array(RMSE_list[type]))) # 10000*9
                    R_list[type][_].append(RMSE_[type])
                    total_R_list[type][_].append(np.mean(np.array(RMSE_list[type]), axis=0))
                
                if not type == -2:
                    ANEES_[type] = np.mean(np.mean(np.array(ANEES_list[type])))
                    A_list[type][_].append(ANEES_[type])
                    total_A_list[type][_].append(np.mean(np.array(ANEES_list[type]), axis = 1))
                else:
                    ANEES_[type] = np.mean(np.mean(np.array(ANEES_list[type])))
                    A_list[type][_].append(ANEES_[type])
                    total_A_list[type][_].append(np.mean(np.array(ANEES_list[type]), axis = 0))             

    
    for type in total_R_list.keys():
        aver_R_list[type] = (np.mean(np.array(total_R_list[type]), axis = 0)).T # 11*(9,)
        aver_A_list[type] = (np.mean(np.array(total_A_list[type]), axis = 0)).T


    draw.each_meas_mean_(_list = aver_R_list, tag = 'R') # 6*10
    draw.each_meas_mean_(_list = aver_A_list, tag = 'A')

    for type in R_list.keys():
        # 20*11
        R_[type] = np.mean(np.array(R_list[type]), axis=0) # 1*11
        A_[type] = np.mean(np.array(A_list[type]), axis=0) # 1*11

        R_std[type] = np.std(np.array(R_list[type]), axis=0) # 1*11
        A_std[type] = np.std(np.array(A_list[type]), axis=0) # 1*11

    
    draw.bias_mean_(numbers=11, Lists = R_, std = R_std, tag='R')
    draw.bias_mean_(numbers=11, Lists = A_, std = A_std, tag = 'A')
    # draw.show()

def experiment4(args):
    '''
    Show animation about experiment(one run)

    :param args: parameters set at the beginning
    '''
    global fig, ax, number_shall

    numbers = args['numbers']
    init_X = args['init_X']
    types = args['types']
    flag = args['flag']

    global meas_bia_prob, comm_fail_prob
    meas_bia_prob = parameters.meas_bia_prob # 16 4
    comm_fail_prob = parameters.comm_fail_prob
    
    RMSE_list, ANEES_list, M= run(np.array(init_X), numbers, flag, types, comm_fail_prob, meas_bia_prob, 1, 4)

def experiment5(args):
    '''
    Show animation about experiment3(one run and tau increased over time)

    :param args: parameters set at the beginning
    '''
    global fig, ax, number_shall

    numbers = args['numbers']
    init_X = args['init_X']
    types = args['types']
    flag = args['flag']

    global meas_bia_prob, comm_fail_prob
    meas_bia_prob = parameters.meas_bia_prob # 16 4
    comm_fail_prob = parameters.comm_fail_prob
    
    RMSE_list, ANEES_list, M= run(np.array(init_X), numbers, flag, types, comm_fail_prob, meas_bia_prob, 1, 5)

def exec_time(exec_start, exec_end):
    '''
    Converts total seconds into hours, minutes, and seconds. from tsangkai

    Parameters
    ----------
    exec_start : float
        Unix epoch execution start time.
    exec_end : float
        Unix epoch execution stop time.

    Returns
    -------
    hours : float
        Total hours.
    minutes : float
        Remaining minutes.
    seconds : float
        Remaining seconds.

    '''

    hours, rem = divmod(exec_end - exec_start, 3600)
    minutes, seconds = divmod(rem, 60)

    return hours, minutes, seconds
