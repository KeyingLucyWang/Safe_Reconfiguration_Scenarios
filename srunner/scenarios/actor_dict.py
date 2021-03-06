from __future__ import print_function

import math
import carla
import random

from srunner.tools.scenario_helper import (get_location_in_distance_from_wp, get_waypoint_in_distance)

class ActorDict(object):
    def __init__(self, trigger, world_map):
        self.map = world_map
        self.name = 'Generic'
        self.index = -1 # needs to be updated when added to other_actors list.

        self.start_dist = random.randint(10, 50) #randomize later
        self.speed = random.randint(15, 50) #randomize later
        self.trigger = trigger #reference waypoint

        self.start_transform = None
        self.default_transform = None

        self.failed = False
        # self._calculate_transform() #needs to be calculated

    # needs to be overriden
    def _calculate_transform(self):
        print("_calculate_transform needs to be overriden")

    # # needs to be overriden
    # def _initialize_actor(self):
    #     print("_initialize_actor needs to be overriden")
    
    def _calculate_default(self, transform):
        default_transform = carla.Transform(
                carla.Location(transform.location.x,
                transform.location.y,
                transform.location.z - 500),
                transform.rotation)
        return default_transform

class LeadVehicleDict(ActorDict):
    def __init__(self, trigger, world_map, actor_dict):
        super().__init__(trigger, world_map)

        if actor_dict:
            self.start_dist = actor_dict["start_dist"]
            self.trigger = actor_dict["trigger"]
            self.speed = actor_dict["speed"]
            self.stop = actor_dict["stop"]
        else:
            self.stop = random.choice([False, True])


        self.start_transform = self._calculate_transform()
        print("vehicle start location: {}".format(self.start_transform.location))
        self.default_transform = self._calculate_default(self.start_transform)
        self.name = "VehiclesAhead"


    def _calculate_transform(self):
        # print("calculating transform for trigger {} at start distance {}".format(self.trigger.transform.location, self.start_dist))
        waypoint, _ = get_waypoint_in_distance(self.trigger, self.start_dist)
        # print("calculated waypoint location {}".format(waypoint.transform.location))
        # lane = random.randint(0, 2)
        # if lane == 0:
        #     pass
        # elif lane == 1:
        #     left_lane = waypoint.get_left_lane()
        #     if left_lane:
        #         waypoint = left_lane
        # elif lane == 2:
        #     right_lane = waypoint.get_right_lane()
        #     if right_lane:
        #         waypoint = right_lane

        start_transform = carla.Transform(
                carla.Location(waypoint.transform.location.x,
                waypoint.transform.location.y,
                waypoint.transform.location.z + 1),
                waypoint.transform.rotation)
        
        return start_transform

    def _print_config(self):
        string = [
            "VehiclesAhead",
            "Start_Transform: "+str(self.start_transform.location),
            "Trigger: "+str(self.trigger.transform.location),
            "Start_Dist: {}".format(self.start_dist),
            "Speed: {}".format(self.speed),
        ]
        return string

class StationaryObstaclesDict(ActorDict):
    def __init__(self, trigger, world_map):
        super().__init__(trigger, world_map)
        # self.default_transform = self._calculate_default(self.start_transform)
        self.name = "StationaryObstaclesAhead"
        self.offset = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
        self.start_transform = self._calculate_transform()


    def _calculate_transform(self):
        waypoint, _ = get_waypoint_in_distance(self.trigger, self.start_dist)
        lane = random.randint(0, 2)
        if lane == 0:
            pass
        elif lane == 1:
            left_lane = waypoint.get_left_lane()
            if left_lane:
                waypoint = left_lane
        elif lane == 2:
            right_lane = waypoint.get_right_lane()
            if right_lane:
                waypoint = right_lane
        
        lane_width = waypoint.lane_width
        position_yaw = waypoint.transform.rotation.yaw + self.offset['position']
        # orientation_yaw = waypoint.transform.rotation.yaw + self.offset['orientation']
        
        offset_location = carla.Location(
            self.offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            self.offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location = waypoint.transform.location
        location += offset_location
        location.z += self.offset['z']

        start_transform = carla.Transform(location, waypoint.transform.rotation)
        return start_transform

class DynamicObstaclesDict(ActorDict):
    def __init__(self, trigger, world_map, actor_dict):
        super().__init__(trigger, world_map)

        if actor_dict:
            self.start_dist = actor_dict["start_dist"]
            self.trigger = actor_dict["trigger"]
            self.speed = actor_dict["speed"]
            self.time_to_reach = actor_dict["time_to_reach"]
        else:
            self.time_to_reach = random.randint(2, 6)

        self.num_lane_changes = 1
        self.offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        self.start_transform, self.orientation_yaw = self._calculate_transform()
        self.default_transform = self._calculate_default(self.start_transform)
        self.name = "DynamicObstaclesAhead"
        
        self.spawn_attempted = 1
        self._number_of_attempts = 20

    def _get_sidewalk_waypoint(self):
        waypoint = self.trigger
        while True:
            wp_next = waypoint.get_right_lane()
            self.num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    self.start_dist += 1.5
                    waypoint = wp_next
                break
            else:
                self.start_dist += 1.5
                waypoint = wp_next
        return waypoint

    def _update_transform(self):
        # print("Base transform is blocking objects ", self.start_transform)
        self.start_dist += 0.4
        # print("self.start_dist after update: " + str(self.start_dist))
        self.spawn_attempted += 1
        # if self._spawn_attempted >= self._number_of_attempts:
        #     raise RuntimeError

    def _calculate_transform(self):
        waypoint = self._get_sidewalk_waypoint()
        lane_width = waypoint.lane_width
        if self.trigger.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True
        # print("self.start_dist when calculating: " + str(self.start_dist))
        location, _ = get_location_in_distance_from_wp(waypoint, self.start_dist, stop_at_junction)
        waypoint = self.map.get_waypoint(location)

        position_yaw = waypoint.transform.rotation.yaw + self.offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + self.offset['orientation']
        offset_location = carla.Location(
            self.offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            self.offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location

        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _print_config(self):
        string = [
            "DynamicObstaclesAhead",
            "Start_Transform: "+str(self.start_transform.location),
            "Trigger: "+str(self.trigger.transform.location),
            "Start_Dist: {}".format(self.start_dist),
            "Speed: {}".format(self.speed),
            "Time_To_Reach: {}".format(self.time_to_reach)
        ]
        return string


class CutInDict(ActorDict):
    def __init__(self, trigger, world_map, actor_dict):
        super().__init__(trigger, world_map)
        
        self.name = "CutIn"

        if actor_dict:
            self.start_dist = actor_dict["start_dist"]
            self.trigger = actor_dict["trigger"]
            self.speed = actor_dict["speed"]
            self.delta_speed = actor_dict["delta_speed"]
            self.trigger_distance = actor_dict["trigger_distance"]
            self.direction = actor_dict["direction"]
        else:
            self.speed = 20
            self.delta_speed = random.randint(20, 30) # 10 randomize later
            self.start_dist = 10
            self.trigger_distance = 5 # randomize later
        
            # self.trigger, _ = get_waypoint_in_distance(self.spawn_point, 60)
            if self.trigger.get_left_lane() == None and self.trigger.get_right_lane == None:
                print("lane change not possible")
                self.failed = True
            elif self.trigger.get_left_lane() == None or self.trigger.get_left_lane().lane_type != carla.LaneType.Driving:
                self.direction = "right"
            elif self.trigger.get_right_lane() == None or self.trigger.get_right_lane().lane_type != carla.LaneType.Driving:
                self.direction = "left"
            else:
                self.direction = random.choice(["left, right"])

        self.spawn_point = self.trigger
        self.lane_change_speed = 10

        self.start_transform = self._calculate_transform()
        self.default_transform = self._calculate_default(self.start_transform)


    def _calculate_transform(self):
        if(self.direction == "left"):
            self.spawn_point = self.spawn_point.get_left_lane()
        else:
            self.spawn_point = self.spawn_point.get_right_lane()
        
        spawn_location = self.spawn_point.transform.location
        ahead_location, _ = get_location_in_distance_from_wp(self.trigger, self.start_dist, False)
        self.trigger = self.map.get_waypoint(ahead_location)

        # change_x = ahead_location.x - trigger_location.x
        # change_y = ahead_location.y - trigger_location.y
        actor_location = carla.Location(spawn_location.x,
                                        spawn_location.y,
                                        spawn_location.z)
        waypoint = self.map.get_waypoint(actor_location)
        print("cut in start location: " + str(waypoint))

        return waypoint.transform

    def _print_config(self):
        string = [
            "CutIn",
            "Start_Transform: "+str(self.start_transform.location),
            "Trigger: "+str(self.trigger.transform.location),
            "Start_Dist: {}".format(self.start_dist),
            "Speed: {}".format(self.speed),
            "Delta Speed: {}".format(self.delta_speed),
            "Trigger_Dist: {}".format(self.trigger_distance),
            "Direction: {}".format(self.direction)
        ]
        return string
    