#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot

    TAB          : change sensor position
    `            : next sensor
    [1-9]        : change to sensor [1-9]
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function
# from navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
# from navigation.roaming_agent import RoamingAgent  # pylint: disable=import-error
# from navigation.basic_agent import BasicAgent
from behavior import BehaviorAgent
from navigation.types_behavior import Cautious, Aggressive, Normal

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import re
import time
import weakref

import glob
import os
import random
import sys

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


from vehicle_detection import is_safe_ttc

def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


class World(object):
    def __init__(self, carla_world, hud):
        self.world = carla_world
        self.map = carla_world.get_map()
        self.mapname = carla_world.get_map().name
        self.hud = hud
        self.world.on_tick(hud.on_world_tick)
        self.world.wait_for_tick(10.0)
        self.player = None
        while self.player is None:
            print("Scenario not yet ready")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "hero":
                    self.player = vehicle
        self.vehicle_name = self.player.type_id
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.set_sensor(0, notify=False)
        self.controller = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

    def restart(self):
        cam_index = self.camera_manager._index
        cam_pos_index = self.camera_manager._transform_index
        start_pose = self.player.get_transform()
        start_pose.location.z += 2.0
        start_pose.rotation.roll = 0.0
        start_pose.rotation.pitch = 0.0
        # blueprint = self._get_random_blueprint()
        blueprint = self.world.get_blueprint_library().find("vehicle.audi.etron")

        self.destroy()
        self.player = self.world.spawn_actor(blueprint, start_pose)
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        # count = 0
        # while len(self.world.get_actors().filter(self.vehicle_name)) < 1:
        #     print("Scenario ended -- Terminating")
        #     time.sleep(1)
        #     count += 1
        #     if count > 3:
        #         return False
        if len(self.world.get_actors().filter(self.vehicle_name)) < 1:
            print("Scenario ended -- Terminating")
            return False

        self.hud.tick(self, self.mapname, clock)
        return True

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, control_mode):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        self.world = world
        self._conditions_satisfied = False

        self._manual_input = False
        self._control_mode = control_mode
        self._start_transition = None
        self._transition_prev_time = None
        
        self._manual_request_sent = False
        self._allow_switch_to_manual = False

        self._autonomous_request_sent = False
        self._allow_switch_to_autonomous = False
        self._start_request_period = None

        self._steer_cache = 0.0

    def check_autonomous_conditions(self, world):
        # check time-to-collision condition
        if(world.hud.server_fps != 0):
            (is_safe, ttc, npc_id) = is_safe_ttc(world, world.hud.server_fps)#clock.get_fps()), 
            if (not is_safe):
                print("ttc not safe")
                return False

        # check speed limit condition
        if (world.player.get_speed_limit() == 30):
            world.hud.notification("Autonomous mode not available: Speed limit requirement not satisfied.")
            # print("speed limit not safe")
            return False

        # check weather condition
        weather_params = world.world.get_weather()
        cloudiness_threshold = 50
        precipitation_threshold = 50
        precipitation_deposits_threshold = 50
        wind_intensity_threshold = 50

        #check if any weather condition requirement is not satisfied
        if (weather_params.cloudiness > cloudiness_threshold
            or weather_params.precipitation > precipitation_threshold
            or weather_params.precipitation_deposits > precipitation_deposits_threshold
            or weather_params.wind_intensity > wind_intensity_threshold):
            # print("weather condition not safe")
            return False

        return True

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True     

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame_number = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame_number = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, mapname, clock):
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame_number - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % mapname,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in vehicles:#sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        self._notifications.tick(world, clock)

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item: # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._history = []
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self._history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self._hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self._history.append((event.frame_number, intensity))
        if len(self._history) > 4000:
            self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
        # self._hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            item.append(bp)
        self._index = None

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    tot_target_reached = 0
    num_min_waypoints = 21

    num_mode_switch = 0

    rainy_weather = carla.WeatherParameters(40, 60, 40, 40, 0, 0, 75) 
    weather_changed = False
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        # display = pygame.display.set_mode(
        #     (args.width, args.height),
        #     pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud)

        starting_mode = "Manual"
        controller = KeyboardControl(world, starting_mode)

        if controller._control_mode == "Manual":
            print("\n" + args.behavior + "\n")
            agent = BehaviorAgent(world.player, behavior=args.behavior, ignore_traffic_light=True)
        elif controller._control_mode == "Autonomous":
            agent = BehaviorAgent(world.player, behavior="normal", ignore_traffic_light=True)

        # SET DESTINATION HERE
        destination = world.map.get_waypoint(carla.Location(200, -250, 3)).transform.location
        print("destination location: " + str(destination))

        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)

        clock = pygame.time.Clock()

        # world.world.set_weather(sunny_weather)

        # get weather change info from test_config file
        f = open("test_config.txt","r")
        extreme_weather = f.read().split(",")[27]
        if extreme_weather == "True":
            extreme_weather = True
        else:
            extreme_weather = False

        while True:
            clock.tick_busy_loop(60)
            controller._conditions_satisfied = controller.check_autonomous_conditions(world)
            (safe_ttc, ttc, vehicle_id) = is_safe_ttc(world, clock.get_fps())
            
            if extreme_weather:
                trigger_loc = world.map.get_waypoint(carla.Location(48, -312, 0)).transform.location
                dist = math.sqrt((trigger_loc.x - world.player.get_location().x)**2 + (trigger_loc.y - world.player.get_location().y)**2)
                if dist < 10:
                    world.world.set_weather(rainy_weather)
                    agent.extreme_weather = True
                    weather_changed = True

            if safe_ttc:
                # if controller.parse_events():
                #     return
                pass
            else:
                hazard_vehicle = world.world.get_actor(vehicle_id)
                hazard_location = hazard_vehicle.get_location()
                hazard_waypoint = world.map.get_waypoint(hazard_location)
                ego_location = world.player.get_location()
                ego_waypoint = world.map.get_waypoint(ego_location)
                speed_non_zero = world.player.get_velocity().x != 0 or world.player.get_velocity().y != 0
                print("speed not zero: " + str(speed_non_zero))
                if (world.player.get_velocity().x == 0 and world.player.get_velocity().y == 0):
                    hazard_ahead = False
                elif(world.player.get_velocity().y == 0):
                    hazard_ahead = (hazard_location.x - ego_location.x)/world.player.get_velocity().x >= 0
                elif(world.player.get_velocity().x == 0):
                    hazard_ahead = (hazard_location.y - ego_location.y)/world.player.get_velocity().y >= 0
                else:
                    hazard_ahead = ((hazard_location.x - ego_location.x)/world.player.get_velocity().x >= 0
                                or (hazard_location.y - ego_location.y)/world.player.get_velocity().y >= 0)
                print("hazard ahead: " + str(hazard_ahead))
                same_lane = hazard_waypoint.lane_id == ego_waypoint.lane_id
                print("same lane: " + str(same_lane))
                # if (same_lane
                #     and speed_non_zero
                #     and hazard_ahead):
                if same_lane:
                    safety_control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True, reverse=True, manual_gear_shift=True, gear=0)
                    world.player.apply_control(safety_control)
                    hud.notification("Unsafe time-to-collision ahead. Applying safety brake...")
                # elif same_lane and not hazard_ahead:
                #     safety_control = carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0, hand_brake=True, reverse=True, manual_gear_shift=True, gear=0)
                #     world.player.apply_control(safety_control)
                #     hud.notification("Unsafe time-to-collision behind. Applying safe acceleration...")
            # if controller._conditions_satisfied:
            #     controller._control_mode = "Autonomous"
            # # conditions not satisfied
            # # elif controller._control_mode == "Autonomous":
            # #     controller._control_mode = "Transition"
            # else:
            #     controller._control_mode = "Manual"

            # if controller._control_mode == "Autonomous" and not controller._conditions_satisfied:
            #     # switch into manual mode
            #     controller._start_request_period = time.time()
            #     controller._allow_switch = True
            #     controller._request_sent = True
            #     reaction_time = -1

            #(controller._control_mode == "Manual" and 
            if (controller._control_mode == "Manual" and controller._conditions_satisfied
                and not controller._autonomous_request_sent and not controller._allow_switch_to_autonomous):
                controller._start_request_period = time.time()
                controller._allow_switch_to_autonomous = True
                controller._autonomous_request_sent = True
                reaction_time = -1

                # cautious agent
                if agent.behavior.overtake_counter == -1:
                    # cautious human agents have a faster reaction time 
                    controller._transition_prev_time = random.randint(2, 4)
                    reaction_time = controller._transition_prev_time
                else: # aggressive agent
                    controller._transition_prev_time = random.randint(6, 8)
                    reaction_time = controller._transition_prev_time

            elif (controller._control_mode == "Autonomous" and not controller._conditions_satisfied
                and not controller._manual_request_sent and not controller._allow_switch_to_manual):
                controller._start_request_period = time.time()
                controller._allow_switch_to_manual = True
                controller._manual_request_sent = True
                reaction_time = -1
            
            if controller._start_request_period != None and controller._allow_switch_to_manual:
                cur_time = int(time.time() - controller._start_request_period)
                if(cur_time <= controller._transition_prev_time):#!= controller._transition_prev_time):
                    time_remaining = controller._transition_prev_time - cur_time
                    # print("Time remaining: {}".format(time_remaining))
                    world.hud.notification("Switching into Manual mode. Time remaining: {}".format(time_remaining))
                    # controller._transition_prev_time = cur_time
                    # if cur_time == controller._transition_prev_time:
                # if controller._control_mode == "Autonomous":
                    # agent = BehaviorAgent(world.player, behavior=args.behavior, ignore_traffic_light=True)
                else:
                    # mode transition to Manual
                    num_mode_switch += 1

                    if args.behavior == "aggressive":
                        agent.behavior = Aggressive()
                        controller._control_mode = "Manual"
                        world.hud.notification("switched to aggressive human driver")
                    elif args.behavior == "cautious":
                        agent.behavior = Cautious()
                        controller._control_mode = "Manual"
                        world.hud.notification("switched to cautious human driver")
                    
                    # controller._allow_switch_to_autonomous = True
                    # controller._autonomous_request_sent = False
                    controller._manual_request_sent = False
                    controller._allow_switch_to_manual = False
                    

            elif controller._start_request_period != None and controller._allow_switch_to_autonomous:
                cur_time = int(time.time() - controller._start_request_period)
                if(cur_time <= controller._transition_prev_time):#!= controller._transition_prev_time):
                    time_remaining = controller._transition_prev_time - cur_time
                    # print("Time remaining: {}".format(time_remaining))
                    world.hud.notification("Switching into Autonomous mode. Time remaining: {}".format(time_remaining))
                    # controller._transition_prev_time = cur_time
                # if cur_time == controller._transition_prev_time:
                else:
                    # if controller._control_mode == "Autonomous":
                    #     # agent = BehaviorAgent(world.player, behavior=args.behavior, ignore_traffic_light=True)
                    #     if args.behavior == "aggressive":
                    #         agent.behavior = Aggressive()
                    #         controller._control_mode = "Manual"
                    #         world.hud.notification("switched to aggressive human driver")
                    #     elif args.behavior == "cautious":
                    #         agent.behavior = Cautious()
                    #         controller._control_mode = "Manual"
                    #         world.hud.notification("switched to cautious human driver")
                        
                    #     # controller._request_sent = False
                    #     # controller._allow_switch = False
                    
                    # elif controller._control_mode == "Manual":

                    # mode transition to Autonomous
                    num_mode_switch += 1

                    agent.behavior = Normal()
                    controller._control_mode = "Autonomous"
                    world.hud.notification("switched to autonomous agent")
                    
                    controller._autonomous_request_sent = False
                    # controller._manual_request_sent = False
                    # controller._allow_switch_to_manual = True
                    controller._allow_switch_to_autonomous = False

            if not world.world.wait_for_tick(10.0):
                continue
            
            agent.update_information(world)

            if (not world.tick(clock)):
                print("Failure: scenario terminated")
                break
            # world.render(display)
            # pygame.display.flip()

            # Set new destination when target has been reached
            if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints and args.loop:
                spawn_points = world.map.get_spawn_points()
                random.shuffle(spawn_points)
                agent.reroute(spawn_points)
                tot_target_reached += 1
                world.hud.notification("The target has been reached " +
                                        str(tot_target_reached) + " times.", seconds=4.0)

            elif len(agent.get_local_planner().waypoints_queue) == 0 and not args.loop:
                print("Target reached, mission accomplished...")
                break

            speed_limit = world.player.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step(weather_changed)
            world.player.apply_control(control)

    finally:
        
        f = open("test.log", "a")
        f.write("Total Number of Mode Transitions: {}".format(num_mode_switch))
        f.write("\n##########################################################\n")

        f.close()
        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "aggressive", "autonomous"],
        help='Choose one of the possible agent behaviors (default: cautious) ',
        default='cautious')
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Behavior", "Roaming", "Basic"],
                           help="select which agent to run",
                           default="Behavior")
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':

    main()
