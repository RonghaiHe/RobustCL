import time
import argparse
import parameters
import utils as u

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
    print('Initial pose(s): ' + str(args.init_X))
    print('Algorithm(s): ' + ', '.join(LABELS[i] for i in args.types))
    print('Measurement model: ' + MEAS_MODEL[args.flag])
    print('comm_fail_prob: ' + str(args.comm_fail_prob))
    print('The number of Times: ' + str(args.numbers))
    print('Missions: ' + str(args.mission) + ':')
    print(MISSION_DETAIL[int(args.mission)])
    print('')

    parameters.comm_fail_prob = args.comm_fail_prob

    # start
    exec_start = time.time()

    # v1.1.0, Add New Mission
    if args.mission == '0':
        u.experiment0(args)
    elif args.mission == '1':
        u.experiment1(args)
    elif args.mission == '2':
        u.experiment2(args)
    elif args.mission == '3':
        u.experiment3(args)
    elif args.mission == '4':
        u.experiment4(args)
    elif args.mission == '5':
        u.experiment5(args)

    # end
    exec_end = time.time()
    hours, minutes, seconds = u.exec_time(exec_start, exec_end)
    print("Total execution time: {:0>2}:{:0>2}:{:05.2f}".format(
        int(hours), int(minutes), seconds))


if __name__ == '__main__':
    # Define the command-line arguments
    parser.add_argument(
        '--comm_fail_prob', type=float, help='set communication failure probability', default=0.9)
    parser.add_argument(
        '--init_X', nargs='+', type=int, help='initial poses of 6 robots',
        default=[[0, 12, 0], [1, 9, 0], [0, 6, 0], [1, 3, 0], [0, 0, 0], [1, -3, 0]])
    parser.add_argument(
        '--types', nargs='+', type=int, help='algorithm types', default=[-1, 6, 0, 2, 12, 28])
    parser.add_argument(
        '--flag', type=int, help='measurement model', default=-1)
    parser.add_argument(
        '--numbers', type=int, help='simulation round', default=1000)
    parser.add_argument(
        '--mission', type=str, help='program mission', default='1')

    # Parse the command-line arguments
    args = parser.parse_args()

    main(args)
