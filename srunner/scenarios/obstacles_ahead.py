from __future__ import print_function

import math
import py_trees
import carla
import random

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider, CarlaActorPool
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               DriveDistance)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp

class StationaryObstaclesAhead(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                    timeout=40):
        """
        Initialize all parameters required for the scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)

        self._ego_vehicle_distance_driven = 10
        # self._other_actor_target_velocity = 10

        self.timeout = timeout

        self._num_actors = 2
        # Call constructor of BasicScenario
        super(StationaryObstaclesAhead, self).__init__(
            "StationaryObstaclesAhead",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)


    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        actor_list = []
        num_actors = 0
        # initialize parameters for each actor
        while num_actors < self._num_actors:
            actor_dict = {}
            actor_dict["_start_distance"] = random.randint(1, 3) * num_actors * 10 + random.randint(10, 30)#num_actors * 20 + 20
            actor_dict["offset"] = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
            actor_list.append(actor_dict)
            num_actors += 1

        print("num_actors: " + str(num_actors))
        num_actors -= 1
        while num_actors >= 0:
            actor_dict = actor_list[num_actors]
            _start_distance = actor_dict["_start_distance"]
            lane_width = self._reference_waypoint.lane_width
            location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _start_distance)
            waypoint = self._wmap.get_waypoint(location)
            offset = actor_dict["offset"]
            position_yaw = waypoint.transform.rotation.yaw + offset['position']
            orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
            offset_location = carla.Location(
                offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
                offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
            location += offset_location
            location.z += offset['z']
            self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
            static = CarlaActorPool.request_new_actor('static.prop.container', self.transform)
            static.set_simulate_physics(True)
            self.other_actors.append(static)
            print("initialized actor: " + str(num_actors))
            num_actors -= 1
       
    def _create_behavior(self):
        """
        Setup the behavior for NewScenario
        """
        lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
            self.ego_vehicles[0].get_location()).lane_width
        lane_width = lane_width + (1.25 * lane_width)
        
         # non leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # scenario_sequence = py_trees.composites.Sequence()
        num_actors = self._num_actors - 1
        while num_actors >= 0:
            scenario_sequence = py_trees.composites.Sequence()
            actor_stand = TimeOut(15)
            actor_removed = ActorDestroy(self.other_actors[num_actors])
            end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven)

            root.add_child(scenario_sequence)
            scenario_sequence.add_child(ActorTransformSetter(self.other_actors[num_actors], self.transform))
            scenario_sequence.add_child(actor_stand)
            scenario_sequence.add_child(actor_removed)
            scenario_sequence.add_child(end_condition)
            num_actors -= 1
        return root

    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
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
    

class DynamicObstaclesAhead(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40
        # other vehicle parameters
        self._other_actor_target_velocity = 5
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 10

        # if random.randint(1, 2) == 1:
        #     self._adversary_type = False  # flag to select either pedestrian (False) or cyclist (True)
        # else:
        #     
        self._adversary_type = adversary_type
        
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0

        self._num_actors = random.randint(1, 3)
        print("number of actors: {}".format(self._num_actors))

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()
        self.actor_list = {}

        super(DynamicObstaclesAhead, self).__init__("DynamicObstaclesAhead",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint, offset):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        # offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        if not self._adversary_type:
            location.z = self._trigger_location.z + offset['z']
        else:
            location.z = self._trigger_location.z - offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw, actor_dict):

        actor_dict["_time_to_reach"] *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw = orientation_yaw
            self._other_actor_target_velocity = 3 + (0.4 * self._num_lane_changes)
            walker = CarlaActorPool.request_new_actor('walker.*', transform)
            adversary = walker
        else:
            self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
            print("target velocity: {}".format(self._other_actor_target_velocity))
            first_vehicle = CarlaActorPool.request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary


    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        num_actors = 0
        
        while num_actors < self._num_actors:
            actor_dict = {}
            actor_dict["_start_distance"] = random.randint(1, 3) * num_actors * 10 + random.randint(10, 30)
            actor_dict["offset"] = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
            actor_dict["_time_to_reach"] = random.randint(1, 5)
            
        # cyclist transform
        # _start_distance = 12
        # We start by getting and waypoint in the closest sidewalk.
            waypoint = self._reference_waypoint
            while True:
                wp_next = waypoint.get_right_lane()
                self._num_lane_changes += 1
                if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                    break
                elif wp_next.lane_type == carla.LaneType.Shoulder:
                    # Filter Parkings considered as Shoulders
                    if wp_next.lane_width > 2:
                        actor_dict["_start_distance"] += 1.5
                        waypoint = wp_next
                    break
                else:
                    actor_dict["_start_distance"] += 1.5
                    waypoint = wp_next

            actor_dict["transform"], orientation_yaw = self._calculate_base_transform(actor_dict["_start_distance"], waypoint, actor_dict["offset"])
            first_vehicle = self._spawn_adversary(actor_dict["transform"], orientation_yaw, actor_dict)

            # Now that we found a possible position we just put the vehicle to the underground
            disp_transform = carla.Transform(
                carla.Location(actor_dict["transform"].location.x,
                            actor_dict["transform"].location.y,
                            actor_dict["transform"].location.z - 500),
                            actor_dict["transform"].rotation)

            first_vehicle.set_transform(disp_transform)
            first_vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(first_vehicle)
            self.actor_list[num_actors] = actor_dict
            num_actors += 1

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes
        # leaf nodes
        num_actors = 0
        while num_actors < self._num_actors:
            if self._ego_route is not None:
                start_condition = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                        self._ego_route,
                                                                        self.actor_list[num_actors]["transform"].location,
                                                                        dist_to_trigger)
            else:
                start_condition = InTimeToArrivalToVehicle(self.other_actors[num_actors],
                                                        self.ego_vehicles[0],
                                                        self.actor_list[num_actors]["_time_to_reach"])
                print("time to reach: {}".format(self.actor_list[num_actors]["_time_to_reach"]))
                print("other actor: {}".format(num_actors))

            actor_velocity = KeepVelocity(self.other_actors[num_actors],
                                        self._other_actor_target_velocity,
                                        name="walker velocity")
            actor_drive = DriveDistance(self.other_actors[num_actors],
                                        0.5 * lane_width,
                                        name="walker drive distance")
            actor_start_cross_lane = AccelerateToVelocity(self.other_actors[num_actors],
                                                        1.0,
                                                        self._other_actor_target_velocity,
                                                        name="walker crossing lane accelerate velocity")
            actor_cross_lane = DriveDistance(self.other_actors[num_actors],
                                            lane_width,
                                            name="walker drive distance for lane crossing ")
            actor_stop_crossed_lane = StopVehicle(self.other_actors[num_actors],
                                                self._other_actor_max_brake,
                                                name="walker stop")
            ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                            5,
                                            name="ego vehicle passed prop")
            actor_remove = ActorDestroy(self.other_actors[num_actors],
                                        name="Destroying walker")
            end_condition = DriveDistance(self.ego_vehicles[0],
                                        self._ego_vehicle_distance_driven,
                                        name="End condition ego drive distance")

            # non leaf nodes

            scenario_sequence = py_trees.composites.Sequence()
            keep_velocity_other = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
            keep_velocity = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")

            # building tree

            root.add_child(scenario_sequence)
            scenario_sequence.add_child(ActorTransformSetter(self.other_actors[num_actors], self.actor_list[num_actors]["transform"],
                                                            name='TransformSetterTS3walker', physics=False))
            scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[num_actors], True))
            scenario_sequence.add_child(start_condition)
            scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[num_actors], False))
            scenario_sequence.add_child(keep_velocity)
            scenario_sequence.add_child(keep_velocity_other)
            scenario_sequence.add_child(actor_stop_crossed_lane)
            scenario_sequence.add_child(actor_remove)
            scenario_sequence.add_child(end_condition)

            keep_velocity.add_child(actor_velocity)
            keep_velocity.add_child(actor_drive)
            keep_velocity_other.add_child(actor_start_cross_lane)
            keep_velocity_other.add_child(actor_cross_lane)
            keep_velocity_other.add_child(ego_pass_machine)
            num_actors += 1

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