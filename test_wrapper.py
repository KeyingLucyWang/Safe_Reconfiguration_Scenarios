import os
import argparse
from argparse import RawTextHelpFormatter

description = ("Agent Wrapper for Testing")

parser = argparse.ArgumentParser(description=description,
                                    formatter_class=RawTextHelpFormatter)
parser.add_argument('--num_reps', default="1")
parser.add_argument('--num_tests', default="1")

args = parser.parse_args()

repetitions = int(args.num_reps)
tests = int(args.num_tests)
for test in range(tests):
    counter = 0
    print("\nRunning autonomous agent\n")
    # run autonomous agent
    while counter < repetitions:
        os.system('python3 autonomous_agent.py')
        counter += 1
        print("autonomous: scenario repetition " + str(counter))

    counter = 0

    print("\nRunning cautious human agent\n")
    # run cautious human agent
    while counter < repetitions:
        os.system('python3 cautious_agent.py')
        counter += 1
        print("cautious human: scenario repetition " + str(counter))


    counter = 0

    print("\nRunning aggressive human agent\n")
    # run aggressive human agent
    while counter < repetitions:
        os.system('python3 aggressive_agent.py')
        counter += 1
        print("aggressive human: scenario repetition " + str(counter))

    counter = 0

    print("\nRunning reconfiguration agent\n")
    # run reconfiguration agent
    while counter < repetitions:
        os.system('python3 reconfiguration_agent.py')
        counter += 1
        print("reconfiguration: scenario repetition " + str(counter))

