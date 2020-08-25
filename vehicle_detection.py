# implementation for vehicle detection
import carla
import math

# runs one step in the pre-crash simulation
# returns a tuple of location and velocity
def simulation_run_step(location, velocity, acceleration, interval):
    # use kinematic formula: delta x = v0*t + (1/2)a*t^2
    displacement_x = velocity.x * interval + (1/2)*acceleration.x*(interval**2)
    displacement_y = velocity.y * interval + (1/2)*acceleration.y*(interval**2)
    displacement_z = velocity.z * interval + (1/2)*acceleration.z*(interval**2)

    sx = displacement_x + location.x
    sy = displacement_y + location.y
    sz = displacement_z + location.z

    vx = velocity.x + acceleration.x*interval
    vy = velocity.y + acceleration.y*interval
    vz = velocity.z + acceleration.z*interval

    new_location = carla.Location(sx, sy, sz)
    new_velocity = carla.Vector3D(vx, vy, vz)
    
    return (new_location, new_velocity)

def check_for_collision(location1, location2):
    # naive implementation
    return location1.distance(location2) < 3

def time_to_collision(world, ego, vehicle, ttc_threshold, fps):    
    ego_location = ego.get_location()
    ego_velocity = ego.get_velocity()
    ego_acceleration = ego.get_acceleration()

    npc_location = vehicle.get_location()
    npc_velocity = vehicle.get_velocity()
    npc_acceleration = vehicle.get_acceleration()

    min_ttc = ttc_threshold
    interval = 1.0/fps
    t = 0.0
    int_t = int(t)
    while int_t < min_ttc:
        # check if a crash occurs
        (ego_location, ego_velocity) = simulation_run_step(ego_location, ego_velocity, ego_acceleration, interval)
        (npc_location, npc_velocity) = simulation_run_step(npc_location, npc_velocity, npc_acceleration, interval)

        if (check_for_collision(ego_location, npc_location)):
            ego_speed = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            npc_speed = 3.6 * math.sqrt(npc_velocity.x**2 + npc_velocity.y**2 + npc_velocity.z**2)
            if(ego_speed <= npc_speed):
                break;
            cur_waypoint = world.map.get_waypoint(ego_location)
            return t

        t += interval
        int_t = int(t)

    return min_ttc

def is_safe_ttc(world, fps):
    # set the ttc threshold to 4 seconds
    min_ttc = 3

    ego = world.player

    #..., -2, -1, 0(reference line), 1, 2,...
    # same signedness indicates same direction
    ego_location = ego.get_location()
    ego_waypoint = world.map.get_waypoint(ego_location)
    for vehicle in world.world.get_actors().filter('vehicle.*'): # ADD WALKER HERE
        vehicle_location = vehicle.get_location()
        vehicle_waypoint = world.map.get_waypoint(vehicle.get_location())
        same_lane = (ego_waypoint.lane_id == vehicle_waypoint.lane_id)
        # only consider vehicles within a 200 meter radius
        if (vehicle.id != ego.id and ego_location.distance(vehicle_location) < 200
            and same_lane):
            if fps:
                ttc = time_to_collision(world, ego, vehicle, min_ttc, fps)
                if (ttc < min_ttc):
                    print("potential crash with {}".format(vehicle.type_id))
                    print("at distance {}\n".format(ego.get_location().distance(vehicle.get_location())))
                    return (False, ttc, vehicle.id)
    for vehicle in world.world.get_actors().filter('walker.*'): # ADD WALKER HERE
        # print("checking collisions with pedestrians")
        vehicle_location = vehicle.get_location()
        vehicle_waypoint = world.map.get_waypoint(vehicle.get_location())
        same_lane = (ego_waypoint.lane_id == vehicle_waypoint.lane_id)
        # only consider vehicles within a 200 meter radius
        if (vehicle.id != ego.id and ego_location.distance(vehicle_location) < 200
            and same_lane):
            if fps:
                ttc = time_to_collision(world, ego, vehicle, min_ttc, fps)
                if (ttc < min_ttc):
                    print("potential crash with {}".format(vehicle.type_id))
                    print("at distance {}\n".format(ego.get_location().distance(vehicle.get_location())))
                    return (False, ttc, vehicle.id)
    return (True, min_ttc, -1)



def lane_change_ttc(world, location, velocity, acceleration, vehicle, ttc_threshold, fps):    
    ego_location = location
    ego_velocity = velocity
    ego_acceleration = acceleration

    npc_location = vehicle.get_location()
    npc_velocity = vehicle.get_velocity()
    npc_acceleration = vehicle.get_acceleration()

    min_ttc = ttc_threshold
    interval = 1.0/fps
    t = 0.0
    int_t = int(t)
    while int_t < min_ttc:
        # check if a crash occurs
        (ego_location, ego_velocity) = simulation_run_step(ego_location, ego_velocity, ego_acceleration, interval)
        (npc_location, npc_velocity) = simulation_run_step(npc_location, npc_velocity, npc_acceleration, interval)

        if (check_for_collision(ego_location, npc_location)):
            ego_speed = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            npc_speed = 3.6 * math.sqrt(npc_velocity.x**2 + npc_velocity.y**2 + npc_velocity.z**2)

            cur_waypoint = world.map.get_waypoint(ego_location)
            return t

        t += interval
        int_t = int(t)

    return min_ttc