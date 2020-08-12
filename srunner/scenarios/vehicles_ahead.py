import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider, CarlaActorPool
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
import srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions as conditions
# from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
#                                                                                InTriggerDistanceToNextIntersection,
#                                                                                DriveDistance,
#                                                                                StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class VehiclesAhead(BasicScenario):
    
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        
        self._map = CarlaDataProvider.get_map()
        # self._first_vehicle_location = 25
        # self._first_vehicle_speed = 10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 10
        self._other_actor_transform = []
        self.timeout = timeout
        # Timeout of scenario in seconds

        self._num_actors = 2
        self.actor_list = dict()

        if randomize:
            self._first_vehicle_location = random.randint(20, 150)
            # print("leading vehicle start location: " + str(self._first_vehicle_location))
            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)
        
        super(VehiclesAhead, self).__init__("VehiclesAhead",
                                            ego_vehicles,
                                            config,
                                            world,
                                            debug_mode,
                                            criteria_enable=criteria_enable)
    
    def check_valid_location(self, actor_locations, new_location):
        dist_threshold = 50
        for location in actor_locations:
            if abs(location - new_location) < dist_threshold:
                return False
        return True

    def generate_valid_location(self, actor_locations):
        new_location = random.randint(10, 100)
        while not self.check_valid_location(actor_locations, new_location):
            new_location = random.randint(10, 100)
        return new_location

    def _setup_scenario_trigger(self, config):
        start_location = None
        if config.trigger_points and config.trigger_points[0]:
            # start_waypoint, _ = get_waypoint_in_distance(self._map.get_waypoint(config.trigger_points[0].location), 10)
            start_location = config.trigger_points[0].location # start_waypoint.transform.location
            # print("start location set to: {}, {}, {}".format(start_location.x, start_location.y, start_location.z))

        ego_vehicle_route = CarlaDataProvider.get_ego_vehicle_route()

        if start_location:
            if ego_vehicle_route:
                if config.route_var_name is None:  # pylint: disable=no-else-return
                    return conditions.InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                            ego_vehicle_route,
                                                                            start_location,
                                                                            5)
                else:
                    check_name = "WaitForBlackboardVariable: {}".format(config.route_var_name)
                    return conditions.WaitForBlackboardVariable(name=check_name,
                                                                variable_name=config.route_var_name,
                                                                variable_value=True,
                                                                var_init_value=False)

            return conditions.InTimeToArrivalToLocation(self.ego_vehicles[0],
                                                        2.0,
                                                        start_location)

        return None

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        num_actors = 0
        actor_locations = []
        while num_actors < self._num_actors:
            actor_dict = dict()
            actor_dict["speed"] = random.randint(10, 20)
            actor_dict["location"] = self.generate_valid_location(actor_locations)
            actor_locations.append(actor_dict["location"])
            
            
            print("actor " + str(num_actors) + " location: " + str(actor_dict["location"]))
            self.actor_list[num_actors] = actor_dict
            num_actors += 1
        
        num_actors = self._num_actors - 1
        while num_actors >= 0:
            vehicle_location = self.actor_list[num_actors]["location"]
            # first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, vehicle_location)
            first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, vehicle_location)
            lane = random.randint(0, 2)
            if lane == 0:
                pass
            elif lane == 1:
                left_lane = first_vehicle_waypoint.get_left_lane()
                if left_lane:
                    first_vehicle_waypoint = left_lane
            elif lane == 2:
                right_lane = first_vehicle_waypoint.get_right_lane()
                if right_lane:
                    first_vehicle_waypoint = right_lane
            
            # self.actor_list[num_actors]["transform"] = carla.Transform(
            #     carla.Location(first_vehicle_waypoint.transform.location.x,
            #                 first_vehicle_waypoint.transform.location.y,
            #                 first_vehicle_waypoint.transform.location.z + 1),
            #                 first_vehicle_waypoint.transform.rotation)
            other_actor_transform = carla.Transform(
                carla.Location(first_vehicle_waypoint.transform.location.x,
                            first_vehicle_waypoint.transform.location.y,
                            first_vehicle_waypoint.transform.location.z + 1),
                            first_vehicle_waypoint.transform.rotation)
            self.actor_list[num_actors]["transform"] = other_actor_transform
            first_vehicle_transform = carla.Transform(
                carla.Location(other_actor_transform.location.x,
                            other_actor_transform.location.y,
                            other_actor_transform.location.z - 500),
                            other_actor_transform.rotation)
            first_vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
            first_vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(first_vehicle)
            num_actors -= 1

        # first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        # self._other_actor_transform = carla.Transform(
        #     carla.Location(first_vehicle_waypoint.transform.location.x,
        #                    first_vehicle_waypoint.transform.location.y,
        #                    first_vehicle_waypoint.transform.location.z + 1),
        #     first_vehicle_waypoint.transform.rotation)
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform.location.x,
        #                    self._other_actor_transform.location.y,
        #                    self._other_actor_transform.location.z - 500),
        #     self._other_actor_transform.rotation)
        # first_vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)


    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        num_actors = 0 # = 2
        root = py_trees.composites.Parallel("Running leading vehicles",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        while num_actors < self._num_actors:

            # to avoid the other actor blocking traffic, it was spawed elsewhere
            # reset its pose to the required one
            start_transform = ActorTransformSetter(self.other_actors[num_actors], self.actor_list[num_actors]["transform"])

            # let the other actor drive until next intersection
            # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
            driving_to_next_intersection = py_trees.composites.Parallel(
                "DrivingTowardsIntersection",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

            driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[num_actors], self.actor_list[num_actors]["speed"]))
            # driving_to_next_intersection.add_child(conditions.InTriggerDistanceToNextIntersection(
            #     self.other_actors[num_actors], self._other_actor_stop_in_front_intersection))

            # # stop vehicle
            # stop = StopVehicle(self.other_actors[num_actors], self._other_actor_max_brake)

            # end condition
            endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
            endcondition_part1 = conditions.InTriggerDistanceToVehicle(self.other_actors[num_actors],
                                                            self.ego_vehicles[0],
                                                            distance=20,
                                                            name="FinalDistance")
            endcondition_part2 = conditions.StandStill(self.ego_vehicles[0], name="StandStill", duration=5)
            endcondition.add_child(endcondition_part1)
            endcondition.add_child(endcondition_part2)

            # Build behavior tree
            sequence = py_trees.composites.Sequence("Sequence Behavior")
            sequence.add_child(start_transform)
            sequence.add_child(driving_to_next_intersection)
            # sequence.add_child(stop)
            sequence.add_child(endcondition)
            sequence.add_child(ActorDestroy(self.other_actors[num_actors]))
            root.add_child(sequence)

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
