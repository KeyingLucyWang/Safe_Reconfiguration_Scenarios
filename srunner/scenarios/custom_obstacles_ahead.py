from __future__ import print_function

import math
import py_trees
import carla

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

class _ObstaclesAhead(BasicScenario):
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
        super(testScenario, self).__init__(
            "testScenario",
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
            actor_dict["_start_distance"] = num_actors * 20 + 20
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
    