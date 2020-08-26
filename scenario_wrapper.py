import os
import argparse
from argparse import RawTextHelpFormatter

# def main():
#     """
#     main function
#     """
description = ("Scenario Runner Wrapper for Testing")

parser = argparse.ArgumentParser(description=description,
                                    formatter_class=RawTextHelpFormatter)
parser.add_argument('--num_agents', default="1")
parser.add_argument('--num_reps', default="1")
parser.add_argument('--num_tests', default="1")

args = parser.parse_args()

repetitions = int(args.num_reps) * int(args.num_agents)# 4 * num_of_repetitions

# 5 test configurations
for test_num in range(int(args.num_tests)):
    counter = 0
    while counter < repetitions:
        os.system("python3 scenario_runner.py --scenario RandomTest_1 --reloadWorld --output -t {} --rep {}".format(test_num, counter))
        counter += 1
    print("\nTest {} completed\n".format(test_num))

