#!/bin/bash

# Installs xterm
apt-get install xterm

# Runs the program, which receives two parameters: the terminal title and the program command to execute
run_program_in_terminal() {
    xterm -T "$1" -e "$2" &
}

# Run the program in a new terminal
run_program_in_terminal "terminal-1" "sudo python main_fix.py --comm_fail_prob 0.1"
run_program_in_terminal "terminal-5" "sudo python main_fix.py --comm_fail_prob 0.5"
run_program_in_terminal "terminal-9" "sudo python main_fix.py --comm_fail_prob 0.9"

exit 0