import os

repetitions = 1
counter = 0

print("\nRunning autonomous agent\n")
# run autonomous agent
while counter < repetitions:
    os.system('python3 autonomous_agent.py')
    counter += 1
    print("autonomous: scenario " + str(counter))

counter = 0

print("\nRunning cautious human agent\n")
# run cautious human agent
while counter < repetitions:
    os.system('python3 cautious_agent.py')
    counter += 1
    print("cautious human: scenario " + str(counter))


counter = 0

print("\nRunning aggressive human agent\n")
# run aggressive human agent
while counter < repetitions:
    os.system('python3 aggressive_agent.py')
    counter += 1
    print("aggressive human: scenario " + str(counter))

counter = 0

print("\nRunning reconfiguration agent\n")
# run reconfiguration agent
while counter < repetitions:
    os.system('python3 reconfiguration_agent.py')
    counter += 1
    print("reconfiguration: scenario " + str(counter))

