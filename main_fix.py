import time
import argparse
import parameters
parser = argparse.ArgumentParser()

MEAS_MODEL = ['range-bearing', 'range-bearing->relative position',
              'relative pose']
MISSION_DETAIL = ['Draw the sample trajectory', 'ARMSE over time', 'weight of M-estimation over time',
                  'ARMSE over tau', 'animation about ARMSE over time',
                  'animation about ARMSE over tau']
LABELS = ['BDA[8]', '1', 'BDA-CU[2]', '1', '1', '1', 'DMV[1]', '1', '1', '1',
          '1', '1', 'CI+CU[13]', '1', '1', '1', '1', '1', '1', '1',
          '1', '1', '1', '1', '1', '1', '1', '1', 'Ours', '1',
          'CCL', 'DR']


def main(args):
    '''
    Main function

    :param args: parameters set at the beginning
    '''

    # extract args

    print('Loaded Settings:')
    print('Initial pose(s): ' + str(args['init_X']))
    print('Algorithm(s): ' + ', '.join(LABELS[i] for i in args['types']))
    print('Measurement model: ' + MEAS_MODEL[args['flag']])
    print('comm_fail_prob: ' + str(args['comm_fail_prob']))
    print('The number of Times: ' + str(args['numbers']))
    print('Missions: ' + str(args['mission']) + ':')
    print(MISSION_DETAIL[args['mission']])
    print('')

    parameters.comm_fail_prob = args['comm_fail_prob']
    print(parameters.comm_fail_prob)
    import utils as u
    # start
    exec_start = time.time()

    # v1.1.0, Add New Mission
    if args['mission'] == 0:
        u.experiment0(args)
    elif args['mission'] == 1:
        u.experiment1(args)
    elif args['mission'] == 2:
        u.experiment2(args)
    elif args['mission'] == 3:
        u.experiment3(args)
    elif args['mission'] == 4:
        u.experiment4(args)
    elif args['mission'] == 5:
        u.experiment5(args)

    # end
    exec_end = time.time()
    hours, minutes, seconds = u.exec_time(exec_start, exec_end)
    print("Total execution time: {:0>2}:{:0>2}:{:05.2f}".format(
        int(hours), int(minutes), seconds))


if __name__ == '__main__':
    # change for dict, just for debug
    parser.add_argument(
        '--comm_fail_prob', help='set communication failure probability', default=0.9)
    args_ = parser.parse_args()
    args = {
        # Initial poses of 6 robots
        'init_X': [[0, 12, 0], [1, 9, 0], [0, 6, 0], [1, 3, 0], [0, 0, 0], [1, -3, 0]],
        # Algorithm types, see variable **LABELS**
        # 'types': [28], # For experiment2 only, Fig.3

        # Final contrast, experiment1&3, Fig.1 & Fig.2 & Fig.4
        'types': [-1, 6, 0, 2, 12, 28],

        # Measurement Model, see variable **MEAS_MODEL**
        'flag': -1,
        # rho, failed communciation probability
        'comm_fail_prob': float(args_.comm_fail_prob),
        # simulation round
        'numbers': 1000,
        # program mission, see variable **MISSION_DETAIL**
        'mission': 1
    }

    main(args)
