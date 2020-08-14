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

from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
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

# return a list of potential scenarios
def list_potential_scenarios():
    potential_scenario_names = ["DynamicObstaclesAhead", "CutIn", "VehiclesAhead"]#, "StationaryObstaclesAhead" , "CutIn", "ChangeLane"]
    # scenario_description = {
    #                         'name': scenario_name,
    #                         'other_actors': other_vehicles,
    #                         'trigger_position': waypoint,
    #                         }
    return potential_scenario_names

def list_potential_triggers(map):
    potential_triggers = []

    # first trigger
    trigger_location = carla.Location(7.1, -210, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)

    trigger_location = carla.Location(20, -260, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)

    # second trigger
    # trigger_location = carla.Location(28, -280, 3)
    # # trigger_location = carla.Location(20, -280, 3)
    # trigger_waypoint = map.get_waypoint(trigger_location)
    # potential_triggers.append(trigger_waypoint)

    trigger_location = carla.Location(49, -320, 3)

    # trigger_location = carla.Location(40, -315, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    # third trigger
    trigger_location = carla.Location(65, -339, 3)
    # trigger_location = carla.Location(62, -336, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    # 4th trigger
    trigger_location = carla.Location(109, -360, 3)
    # trigger_location = carla.Location(109, -360, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    
    # Turning Right location (exiting highway)
    # trigger_location = carla.Location(188, -365, 3)
    # # trigger_rotation = carla.Rotation(0, 240, 0)
    # # trigger_transform = carla.Transform(trigger_location, trigger_rotation)
    # trigger_waypoint = map.get_waypoint(trigger_location)
    # potential_triggers.append(trigger_waypoint)

    # dynamic obstacles crossing location
    # 5th trigger
    trigger_location = carla.Location(203, -341, 3)
    trigger_waypoint = map.get_waypoint(trigger_location)
    potential_triggers.append(trigger_waypoint)
    
    # trigger_location = carla.Location(200, -298, 3)
    # trigger_waypoint = map.get_waypoint(trigger_location)
    # potential_triggers.append(trigger_waypoint)

    return potential_triggers

class RandomTest(BasicScenario):
    
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        
        self._map = CarlaDataProvider.get_map()
        self._world = world

        # location of the ego vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 10
        self._other_actor_transform = []
        self.timeout = timeout
        # Timeout of scenario in seconds

        self._num_scenarios = 3 #random.randint(1, 5)
        self._num_actors = self._num_scenarios #self._num_scenarios

        self.scenario_list = []
        # have a list of scenarios descriptions (scenario name, actor_dict)
        # actor_dict: trigger_point, transform(offset?), other scenario-specific parameters
        potential_scenarios = list_potential_scenarios()
        potential_triggers = list_potential_triggers(self._map)

        self.selected_scenarios = self.select_scenarios(potential_scenarios, self._num_scenarios)
        self.selected_triggers = random.sample(potential_triggers, self._num_scenarios)
        
        for i in range(len(self.selected_scenarios)):
            trigger = self.selected_triggers[i]
            scenario = self.selected_scenarios[i]
            print(scenario + " | ")
            print(str(trigger) + " | ")
            print("\n")
            if scenario == "VehiclesAhead":
                actor_dict = LeadVehicleDict(trigger, self._map)
            elif scenario == "StationaryObstaclesAhead":
                actor_dict = StationaryObstaclesDict(trigger, self._map)
            elif scenario == "CutIn":
                actor_dict = CutInDict(trigger, self._map)
            elif scenario == "DynamicObstaclesAhead":
                # off_highway_1 = self._map.get_waypoint(carla.Location(200, -298, 3))
                off_highway_1 = self._map.get_waypoint(carla.Location(203, -341, 3))
                if (trigger != off_highway_1):
                    # and trigger != off_highway_2):
                    # trigger = random.choice([off_highway_1, off_highway_2])
                    trigger = off_highway_1
                    print("changed trigger")
                actor_dict = DynamicObstaclesDict(trigger, self._map)
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

        super(RandomTest, self).__init__("RandomTest",
                                        ego_vehicles,
                                        config,
                                        world,
                                        debug_mode,
                                        criteria_enable=criteria_enable)

    def select_scenarios(self, potential_scenarios, num_scenarios):
        random_scenarios = []
        while num_scenarios > 0:
            scenario_ind = random.randint(0, len(potential_scenarios)-1)
            random_scenarios.append(potential_scenarios[scenario_ind])
            num_scenarios -= 1
        return random_scenarios

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
        vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.default_transform)
        vehicle.set_simulate_physics(enabled=False)
        index = len(self.other_actors)
        actor_dict.index = index
        print("index of vehiclesAhead actor: " + str(index))
        self.other_actors.append(vehicle)
    
    def _initialize_stationary_obstacles_ahead(self, actor_dict):
        static = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.start_transform)
        static.set_simulate_physics(True)
        index = len(self.other_actors)
        actor_dict.index = index
        print("index of Stationary obstacle actor: " + str(index))
        self.other_actors.append(static)

    def _initialize_cut_in(self, actor_dict):
        vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', actor_dict.default_transform)
        vehicle.set_simulate_physics(enabled=False)
        index = len(self.other_actors)
        actor_dict.index = index
        print("index of Cut In actor: " + str(index))
        self.other_actors.append(vehicle)

    def _initialize_dynamic_obstacles_ahead(self, actor_dict):
        actor_dict.speed = 3 + (0.4 * actor_dict.num_lane_changes)
        while True:
            try:
                actor_dict.start_transform, orientation = actor_dict._calculate_transform()
                walker = CarlaActorPool.request_new_actor('walker.*', actor_dict.start_transform)
                walker.set_transform(actor_dict.start_transform)
                walker.set_simulate_physics(enabled=False)
                print("walker spawned at" + str(actor_dict.start_transform))
                break

            except RuntimeError as r:
                actor_dict._update_transform()
    
        default = actor_dict._calculate_default(actor_dict.start_transform)
        walker.set_transform(default)
        index = len(self.other_actors)
        actor_dict.index = index
        print("index of dynamic crossing actor: " + str(index))
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
        
        for (scenario, actor_dict) in self.scenario_list:
            if scenario == "VehiclesAhead":
                wait_for_trigger = InTriggerDistanceToLocation(self.ego_vehicles[0], actor_dict.trigger.transform.location, 10)
                start_transform = ActorTransformSetter(self.other_actors[actor_dict.index], actor_dict.start_transform)

                keep_driving = WaypointFollower(self.other_actors[actor_dict.index], actor_dict.speed)
                
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
                if random.randint(0,1) != -1:
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
                                         delta_velocity=actor_dict.delta_speed, trigger_distance=10, max_distance=500)
                
                sequence.add_child(accelerate)

                # lane_change
                # lane_change = None
                if actor_dict.direction == 'left':
                    lane_change = LaneChange(
                        self.other_actors[actor_dict.index], speed=10, direction='right', distance_same_lane=5, distance_other_lane=300)
                    sequence.add_child(lane_change)    
                else:
                    lane_change = LaneChange(
                        self.other_actors[actor_dict.index], speed=10, direction='left', distance_same_lane=5, distance_other_lane=300)
                    sequence.add_child(lane_change)

                endcondition = DriveDistance(self.other_actors[actor_dict.index], 200)
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

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
