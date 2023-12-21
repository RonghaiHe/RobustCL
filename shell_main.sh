#!/bin/bash

# Runs the program, which receives two parameters: the terminal title and the program command to execute
run_program_in_terminal() {
    xterm -T "$1" -e "$2" &
}

# Define the parameters
program_missions=$1

comm_fail_probs=(0.1 0.5 0.9)
# init_X=("0 12 0" "1 9 0" "0 6 0" "1 3 0" "0 0 0" "1 -3 0")


# Loop over the parameters
for comm_fail_prob in "${comm_fail_probs[@]}"; do
    # Write a if-else statement for each program_missions value
    if [ "$program_missions" == "0" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types 28 \
            --mission $program_missions"
    elif [ "$program_missions" == "1" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types -1 6 0 2 12 28 \
            --mission $program_missions"
    elif [ "$program_missions" == "2" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types 28 \
            --mission $program_missions"
    elif [ "$program_missions" == "3" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types -1 6 0 2 12 28 \
            --mission $program_missions"
    elif [ "$program_missions" == "4" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types -1 6 0 2 12 28 \
            --mission $program_missions"
    elif [ "$program_missions" == "5" ]; then
        run_program_in_terminal "terminal-$comm_fail_prob" "python main.py \
            --comm_fail_prob $comm_fail_prob \
            --types -1 6 0 2 12 28 \
            --mission $program_missions"
    else
        # Default case when program_missions doesn't match any specific mission
        echo "Invalid program_missions value"
    fi
done

exit 0