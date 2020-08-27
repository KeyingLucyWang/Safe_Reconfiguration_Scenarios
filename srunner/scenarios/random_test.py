import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider, CarlaActorPool
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      LaneChange,
                                                                      SetInitSpeed,
                                                                      AccelerateToVelocity,
                                                                      AccelerateToCatchUp,
                                                                      HandBrakeVehicle,
                                                                      ChangeAutoPilot,
                                                                      SyncArrival,
                                                                      AddNoiseToVehicle,
                                                                      ChangeNoiseParameters,
                                                                      BasicAgentBehavior,
                                                                      TrafficJamChecker,
                                                                      Idle,
                                                                      ActorSource,
                                                                      ActorSink)

from srunner.scenariomanager.scenarioatomics.atomic_criteria import OtherCollisionTest, CollisionTest, ReachedRegionTest, InRadiusRegionTest
import srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions as conditions
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerRegion,
                                                                               InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               InTimeToArrivalToLocation,
                                                                               InTimeToArrivalToVehicle,
                                                                               InTimeToArrivalToVehicleSideLane,
                                                                               WaitUntilInFront,
                                                                               AtRightmostLane,
                                                                               WaitEndIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               TriggerVelocity)


from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from srunner.scenarios.actor_dict import (ActorDict, 
                                        LeadVehicleDict, 
                                        DynamicObstaclesDict, 
                                        StationaryObstaclesDict,
                                        CutInDict)

from tabulate import tabulate

# return a list of potential scenarios
def list_potential_scenarios():
    # possible scenarios:                 0                     1           2
    potential_scenario_names = ["DynamicObstaclesAhead", "VehiclesAhead", "CutIn"]#, "StationaryObstaclesAhead" , "CutIn", "ChangeLane"]
    return potential_scenario_names

def list_potential_triggers(map):
    potential_triggers = []

    # # first trigger: 0
    # trigger_location = carla.Location(15, -251, 3)
    # trigger_waypoint = map.get_waypoint(trigger_location)
    # potential_triggers.append(trigger_waypoint)

    # second trigger: 1
    trigger_location = carla.Location(32, -289, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    
    # third trigger: 2
    trigger_location = carla.Location(48, -312, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)

    # fourth trigger: 3
    trigger_location = carla.Location(72, -336, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)

    # fifth trigger: 4
    trigger_location = carla.Location(117, -358, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)

    # sixth trigger: 5
    trigger_location = carla.Location(131, -362, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    
    # seventh trigger: 6
    trigger_location = carla.Location(203, -341, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    
    return potential_triggers

class RandomTest(BasicScenario):
    
    def __init__(self, world, ego_vehicles, config, test_num, rep, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        
        self._map = CarlaDataProvider.get_map()
        self._world = world

        # specify test run number
        self._test = int(test_num)
        self._rep = int(rep)

        # location of the ego vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 10
        self._other_actor_transform = []
        self.timeout = timeout
        # Timeout of scenario in seconds

        self._num_scenarios = 3 #random.randint(1, 5)

        self.scenario_list = []
        
        # return all possible scenarios and trigger points
        potential_scenarios = list_potential_scenarios()
        potential_triggers = list_potential_triggers(self._map)

        # if self._test >= 0 and self._test <= 4:
        # if self._rep != 0:
        #     print("\nrunning repeated scenarios\n")#{}\n".format(str(self._test)))
        #     self.selected_scenarios = self.select_scenarios(potential_scenarios, self._test)
        #     self.selected_triggers = self.select_triggers(potential_triggers, self._test)
        # else:
        # i = 0
        # while i < int(generate):
        if self._rep == 0:
            # i += 1
            # print("\nrandomly generating scenario {}\n".format(i))
            # randomly select specified number of scenarios and their trigger points
            self.selected_scenarios = self.select_scenarios_random(potential_scenarios, self._num_scenarios)
            self.selected_triggers = random.sample(potential_triggers, self._num_scenarios)            


            for i in range(len(self.selected_scenarios)):
                trigger = self.selected_triggers[i]
                scenario = self.selected_scenarios[i]
                print(scenario + " | ")
                print(str(trigger) + " | ")
                print("\n")
                if scenario == "VehiclesAhead":
                    actor_dict = LeadVehicleDict(trigger, self._map, None)
                # elif scenario == "StationaryObstaclesAhead":
                #     actor_dict = StationaryObstaclesDict(trigger, self._map)
                elif scenario == "CutIn":
                    actor_dict = CutInDict(trigger, self._map, None)
                elif scenario == "DynamicObstaclesAhead":
                    off_highway_1 = self._map.get_waypoint(carla.Location(203, -341, 3))
                    if (trigger != off_highway_1):
                        trigger = off_highway_1
                    actor_dict = DynamicObstaclesDict(trigger, self._map, None)
                else:
                    print("actor dict class not implemented")
                    exit(1)

                if not actor_dict.failed:
                    self.scenario_list.append((scenario, actor_dict))
                
        # for trigger in self.selected_triggers:
        #     print(str(trigger) + " | ")
        #     print("\n")
        
        # self.actor_list = dict()

        # if randomize:
        #     self._first_vehicle_location = random.randint(20, 150)
            data = "\n"
            for ind in range(len(self.scenario_list)):
                (scenario, actor_dict) = self.scenario_list[ind]
                # scenario = self.selected_scenarios[ind]
                # trigger = self.selected_triggers[ind]
                x = actor_dict.start_transform.location.x
                y = actor_dict.start_transform.location.y
                z = actor_dict.start_transform.location.z
                transform_str = "{}|{}|{}".format(x, y, z)

                x = actor_dict.trigger.transform.location.x
                y = actor_dict.trigger.transform.location.y
                z = actor_dict.trigger.transform.location.z
                trigger_str = "{}|{}|{}".format(x, y, z)

                        #test.rep.scenario
                test_info = "{}.{}.{}".format(self._test, self._rep, ind)
                if scenario == "VehiclesAhead":
                    data += "{},VehiclesAhead,{},{},{},{},{},{},{},".format(test_info, 
                                                                        #actor_dict.start_transform.location,
                                                                        #actor_dict.trigger.transform.location,
                                                                        transform_str,
                                                                        trigger_str,
                                                                        actor_dict.start_dist,
                                                                        actor_dict.speed,
                                                                        actor_dict.stop,
                                                                        "",
                                                                        "")
                elif scenario == "DynamicObstaclesAhead":
                    # self._initialize_dynamic_obstacles_ahead()
                    data += "{},DynamicObstaclesAhead,{},{},{},{},{},{},{},".format(test_info, 
                                                                        #actor_dict.start_transform.location,
                                                                        #actor_dict.trigger.transform.location,
                                                                        transform_str,
                                                                        trigger_str,
                                                                        actor_dict.start_dist,
                                                                        actor_dict.speed,
                                                                        actor_dict.time_to_reach,
                                                                        "",
                                                                        "")
                else: #CutIn
                    data += "{},CutIn,{},{},{},{},{},{},{},".format(test_info, 
                                                                    #actor_dict.start_transform.location,
                                                                    #actor_dict.trigger.transform.location,
                                                                    transform_str,
                                                                    trigger_str,
                                                                    actor_dict.start_dist,
                                                                    actor_dict.speed,
                                                                    actor_dict.delta_speed,
                                                                    actor_dict.trigger_distance,
                                                                    actor_dict.direction)
            
            # 20% chance that there is extreme weather
            # extreme_weather = (random.randint(0, 5) < 1)
            extreme_weather = False
            if extreme_weather:
                data += "True,"
            else:
                data += "False,"
            # data += "\n"
            # f = open('test.txt', 'a')
            # f.write(data)
            f = open('test_config.txt', 'w')
            f.write(data)
            f.close()

            # agent = ""
            # while not agent:
            #     f = open('test_agent.txt','r')
            #     agent = f.read()
            #     f.close()
            # f = open('test_agent.txt','w')
            # f.write("")
            # f.close()

            f = open('test_log.txt', 'a')
            f.write(data)
            f.close()

        # if int(generate) < 0: # retrieve test config from test_config.txt
        else:
            f = open('test_config.txt', 'r')
            config_info = f.read()
            f.close()

            # print("test {}".format(test_num))
            # config_lines = file_content.split("\n")
            # config_info = config_lines[test_num]
            config_list = config_info.split(",")

            # agent = ""
            # while not agent:
            #     f = open('test_agent.txt','r')
            #     agent = f.read()
            #     f.close()

            f = open('test_log.txt', 'a')

            for i in range(self._num_scenarios):
                test_info = config_list[0+i*9].split('.')
                test = test_info[0]
                rep = int(test_info[1])
                scenario = test_info[2]
                rep += 1 # rep += 1
                test_info[1] = str(rep)
                test_info_str = ".".join(test_info)
                config_list[0+i*9] = test_info_str
            config_info = ",".join(config_list)
            f.write(config_info)
            f.close()

            # update scenario config file
            f = open('test_config.txt', 'w')
            f.write(config_info)
            f.close()

            for i in range(self._num_scenarios):
                scenario_name = config_list[1+i*9]
                # start_transform = config_list[3]
                trigger = config_list[3+i*9].split("|") #NEEDS TO BE PARSED --> x.y.z
                x = float(trigger[0])
                y = float(trigger[1])
                z = float(trigger[2])
                trigger_waypoint = self._map.get_waypoint(carla.Location(x, y, z))
                
                print(scenario_name + " | ")
                print(str(trigger_waypoint) + " | ")
                print("\n")

                start_dist = int(config_list[4+i*9])
                speed = int(config_list[5+i*9])

                if scenario_name == "VehiclesAhead":
                    parsed_dict = dict()
                    parsed_dict["start_dist"] = start_dist
                    parsed_dict["trigger"] = trigger_waypoint
                    parsed_dict["speed"] = speed
                    if config_list[6+i*9] == "True":
                        parsed_dict["stop"] = True
                    else:
                        parsed_dict["stop"] = False

                    actor_dict = LeadVehicleDict(None, self._map, parsed_dict)
                    
                elif scenario_name == "DynamicObstaclesAhead":
                    time_to_reach = config_list[6+i*9]
                    parsed_dict = dict()
                    parsed_dict["start_dist"] = start_dist
                    parsed_dict["trigger"] = trigger_waypoint
                    parsed_dict["speed"] = speed
                    parsed_dict["time_to_reach"] = int(time_to_reach)

                    actor_dict = DynamicObstaclesDict(None, self._map, parsed_dict)

                elif scenario_name == "CutIn":
                    delta_speed = int(config_list[6+i*9])
                    trigger_dist = int(config_list[7+i*9])
                    direction = config_list[8+i*9]

                    parsed_dict = dict()
                    parsed_dict["start_dist"] = start_dist
                    parsed_dict["trigger"] = trigger_waypoint
                    parsed_dict["speed"] = speed
                    parsed_dict["delta_speed"] = delta_speed
                    parsed_dict["trigger_distance"] = trigger_dist
                    parsed_dict["direction"] = direction

                    actor_dict = CutInDict(None, self._map, parsed_dict)

                else:
                    print("ERROR: Invalid scenario name")
                    raise RuntimeError

                self.scenario_list.append((scenario_name, actor_dict))

            #PARSE CONFIG HERE

        f = open('test.log', 'a')
        # log test configurations
        # row = []
        # for ind in range(len(self.selected_scenarios)):
        #     scenario = self.selected_scenarios[ind]
        #     trigger = self.selected_triggers[ind]
        #     row.append(scenario + " at: \n" + str(trigger.transform.location))
        
        # scenario_headers = [
        #     "Scenario One", "Scenario Two", "Scenario Three"
        # ]
        f.write("\n\n\n\n##########################################################\n" 
                + "\nTest Number: {}\nRep: {}\n\n".format(test_num, rep)
                + "##########################################################\n")

        # f.write(tabulate([row], headers=scenario_headers))
        # f.write("\n-----------------------------------------------------------------------------------------------------------------------------------------")
        # f.write("\n<-----  Test Configurations  ----->")
        for (scenario, actor_dict) in self.scenario_list:
            log_header = [scenario]
            row = actor_dict._print_config()
            f.write(tabulate([row]))
        f.write("\n")
        f.close()

        super(RandomTest, self).__init__("RandomTest",
                                        ego_vehicles,
                                        config,
                                        world,
                                        debug_mode,
                                        criteria_enable=criteria_enable)

    def select_scenarios_random(self, potential_scenarios, num_scenarios):
        random_scenarios = []
        while num_scenarios > 0:
            scenario_ind = random.randint(0, len(potential_scenarios)-1)
            random_scenarios.append(potential_scenarios[scenario_ind])
            num_scenarios -= 1
        return random_scenarios

    def select_scenarios(self, potential_scenarios, test_num):
        scenario_set = [[1, 1, 0],
                        [1, 1, 0],
                        [2, 1, 1],
                        [2, 0, 0],
                        [1, 1, 1]]
        selected_ind = scenario_set[test_num]
        selected = []
        for i in selected_ind:
            selected.append(potential_scenarios[i])
        return selected

    def select_triggers(self, potential_triggers, test_num):
        trigger_set = [ [0, 1, 6],
                        [2, 3, 6],
                        [1, 4, 5],
                        [2, 3, 6],
                        [1, 3, 6] ]
        selected_ind = trigger_set[test_num]
        selected = []
        for i in selected_ind:
            selected.append(potential_triggers[i])
        return selected

    # def check_valid_location(self, actor_locations, new_location):
    #     dist_threshold = 50
    #     for location in actor_locations:
    #         if abs(location - new_location) < dist_threshold:
    #             return False
    #     return True

    # def generate_valid_location(self, actor_locations):
    #     new_location = random.randint(10, 100)
    #     while not self.check_valid_location(actor_locations, new_location):
    #         new_location = random.randint(10, 100)
    #     return new_location
    def _initialize_vehicles_ahead(self, actor_dict):
        vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.start_transform)#default_transform)
        vehicle.set_simulate_physics(enabled=False)
        index = len(self.other_actors)
        actor_dict.index = index
        # print("index of vehiclesAhead actor: " + str(index))
        self.other_actors.append(vehicle)
    
    def _initialize_stationary_obstacles_ahead(self, actor_dict):
        static = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.start_transform)
        static.set_simulate_physics(True)
        index = len(self.other_actors)
        actor_dict.index = index
        # print("index of Stationary obstacle actor: " + str(index))
        self.other_actors.append(static)

    def _initialize_cut_in(self, actor_dict):
        vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.start_transform)#default_transform)
        vehicle.set_simulate_physics(enabled=False)
        index = len(self.other_actors)
        actor_dict.index = index
        # print("index of Cut In actor: " + str(index))
        self.other_actors.append(vehicle)

    def _initialize_dynamic_obstacles_ahead(self, actor_dict):
        actor_dict.speed = 3 + (0.4 * actor_dict.num_lane_changes)
        while True:
            try:
                actor_dict.start_transform, orientation = actor_dict._calculate_transform()
                walker = CarlaActorPool.request_new_actor('walker.*', actor_dict.start_transform)
                walker.set_transform(actor_dict.start_transform)
                walker.set_simulate_physics(enabled=False)
                # print("walker spawned at" + str(actor_dict.start_transform))
                break

            except RuntimeError as r:
                # actor_dict._update_transform()
                actor_dict.start_dist += 0.4
                # print("self.start_dist after update: " + str(self.start_dist))
                actor_dict.spawn_attempted += 1
                if actor_dict.spawn_attempted >= actor_dict._number_of_attempts:
                    raise r

    
        default = actor_dict._calculate_default(actor_dict.start_transform)
        walker.set_transform(default)
        index = len(self.other_actors)
        actor_dict.index = index
        # print("index of dynamic crossing actor: " + str(index))
        self.other_actors.append(walker)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # num_actors = self._num_actors
        for (scenario, actor_dict) in self.scenario_list:
            if scenario == "VehiclesAhead":
                self._initialize_vehicles_ahead(actor_dict)
            elif scenario == "StationaryObstaclesAhead":
                self._initialize_stationary_obstacles_ahead(actor_dict)
            elif scenario == "CutIn":
                self._initialize_cut_in(actor_dict)
            elif scenario =="DynamicObstaclesAhead":
                self._initialize_dynamic_obstacles_ahead(actor_dict)
                # pass
            else:
                print("Not implemented error. ")
                exit(1)


    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        root = py_trees.composites.Parallel("Running random scenario",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        root.add_child(StandStill(self.ego_vehicles[0], "scenario ended", 3.0))
        root.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], carla.Location(200, -250, 0), 10))
        for (scenario, actor_dict) in self.scenario_list:
            if scenario == "VehiclesAhead":
                wait_for_trigger = InTriggerDistanceToLocation(self.ego_vehicles[0], actor_dict.trigger.transform.location, 10)
                start_transform = ActorTransformSetter(self.other_actors[actor_dict.index], actor_dict.start_transform)

                keep_driving = WaypointFollower(self.other_actors[actor_dict.index], actor_dict.speed, avoid_collision=True)
                
                drive = py_trees.composites.Parallel("Driving for a distance",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                drive_success = DriveDistance(self.other_actors[actor_dict.index], random.randint(50,100))
                drive.add_child(drive_success)
                drive.add_child(keep_driving)
                stand = StandStill(self.other_actors[actor_dict.index], "stand still", random.randint(1,5))
                stop = StopVehicle(self.other_actors[actor_dict.index], 1.0)
                accelerate = AccelerateToVelocity(self.other_actors[actor_dict.index], 1.0, actor_dict.speed)
                # Build behavior tree
                sequence = py_trees.composites.Sequence("VehiclesAhead behavior sequence")
                sequence.add_child(wait_for_trigger)
                sequence.add_child(start_transform)
                
                # stop_condition = py_trees.composites.Parallel("stop condition",
                #                             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                # stop_condition.add_child(keep_driving)
                # stop_condition.add_child(drive_success)
                if actor_dict.stop:
                    sequence.add_child(drive)
                    sequence.add_child(stop)
                    sequence.add_child(accelerate)
                    sequence.add_child(keep_driving) #stop_condition
                    print("lead vehicle stop behavior added")
                else:
                    sequence.add_child(keep_driving) #stop_condition
                
                sequence.add_child(ActorDestroy(self.other_actors[actor_dict.index]))
                root.add_child(sequence)

            elif scenario == "StationaryObstaclesAhead":
                lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
                self.ego_vehicles[0].get_location()).lane_width
                lane_width = lane_width + (1.25 * lane_width)
                sequence = py_trees.composites.Sequence("StationaryObstaclesAhead behavior sequence")
                # actor_stand = TimeOut(15)
                # actor_removed = ActorDestroy(self.other_actors[num_actors])
                # end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven)
                sequence.add_child(ActorTransformSetter(self.other_actors[actor_dict.index], actor_dict.start_transform))
                sequence.add_child(TimeOut(120))
                root.add_child(sequence)

            elif scenario == "CutIn":
                sequence = py_trees.composites.Sequence("CarOn_{}_Lane" .format(actor_dict.direction))
                car_visible = ActorTransformSetter(self.other_actors[actor_dict.index], actor_dict.start_transform)
                
                sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                        actor_dict.trigger.transform.location, 10))
                sequence.add_child(car_visible)

                # accelerate
                accelerate = AccelerateToCatchUp(self.other_actors[actor_dict.index], self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=actor_dict.delta_speed, trigger_distance=actor_dict.trigger_distance, max_distance=100)
                
                sequence.add_child(accelerate)

                # lane_change
                # lane_change = None
                if actor_dict.direction == 'left':
                    lane_change = LaneChange(
                        self.other_actors[actor_dict.index], speed=actor_dict.lane_change_speed, direction='right', distance_same_lane=50, distance_other_lane=10)
                    sequence.add_child(lane_change)    
                else:
                    lane_change = LaneChange(
                        self.other_actors[actor_dict.index], speed=actor_dict.lane_change_speed, direction='left', distance_same_lane=50, distance_other_lane=10)
                    sequence.add_child(lane_change)
                sequence.add_child(WaypointFollower(self.other_actors[actor_dict.index], target_speed=20, avoid_collision=True))
                endcondition = DriveDistance(self.other_actors[actor_dict.index], 150)
                parallel = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                parallel.add_child(sequence)
                parallel.add_child(endcondition)
                root.add_child(parallel)

            elif scenario == "DynamicObstaclesAhead":
                lane_width = actor_dict.trigger.lane_width
                lane_width = lane_width + (1.25 * lane_width * actor_dict.num_lane_changes)

                start_condition = InTimeToArrivalToVehicle(self.other_actors[actor_dict.index],
                                                        self.ego_vehicles[0],
                                                        actor_dict.time_to_reach)
                
                # actor_velocity = KeepVelocity(self.other_actors[actor_dict.index],
                #                         actor_dict.speed,
                #                         name="walker velocity")

                # actor_drive = DriveDistance(self.other_actors[actor_dict.index],
                #                         0.5 * lane_width,
                #                         name="walker drive distance")
                
                actor_start_cross_lane = AccelerateToVelocity(self.other_actors[actor_dict.index],
                                                        1.0,
                                                        actor_dict.speed,
                                                        name="walker crossing lane accelerate velocity")
                actor_cross_lane = DriveDistance(self.other_actors[actor_dict.index],
                                            lane_width,
                                            name="walker drive distance for lane crossing ")

                actor_stop_cross_lane = StopVehicle(self.other_actors[actor_dict.index],
                                                1.0,
                                                name="walker stop")

                ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                            5,
                                            name="ego vehicle passed walker")
                
                # actor_remove = ActorDestroy(self.other_actors[actor_dict.index],
                #                             name="Destroying walker")
                
                scenario_sequence = py_trees.composites.Sequence()
                keep_velocity_other = py_trees.composites.Parallel(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
                keep_velocity = py_trees.composites.Parallel(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")

                scenario_sequence.add_child(ActorTransformSetter(self.other_actors[actor_dict.index], actor_dict.start_transform,
                                                            name='TransformSetterTS3walker', physics=False))
                # scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[actor_dict.index], True))
                scenario_sequence.add_child(start_condition)
                # scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[actor_dict.index], False))
                # scenario_sequence.add_child(keep_velocity)
                scenario_sequence.add_child(keep_velocity_other)
                scenario_sequence.add_child(actor_stop_cross_lane)

                # keep_velocity.add_child(actor_velocity)
                # keep_velocity.add_child(actor_drive)
                keep_velocity_other.add_child(actor_start_cross_lane)
                # keep_velocity_other.add_child(actor_cross_lane)
                # keep_velocity_other.add_child(ego_pass_machine)
                
                root.add_child(scenario_sequence)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0], optional=False, name="CheckCollisions", terminate_on_failure=True)
        target_reached = InRadiusRegionTest(self.ego_vehicles[0], 200, -249, 30)

        other_collision_one = OtherCollisionTest(self.other_actors[0], optional=True, terminate_on_failure=True)
        other_collision_two = OtherCollisionTest(self.other_actors[1], optional=True, terminate_on_failure=True)
        other_collision_three = OtherCollisionTest(self.other_actors[2], optional=True, terminate_on_failure=True)
        # distance_driven = DrivenDistanceTest(self.ego_vehicles[0], 2000, )
        criteria.append(collision_criterion)
        criteria.append(target_reached)
        criteria.append(other_collision_one)
        criteria.append(other_collision_two)
        criteria.append(other_collision_three)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
