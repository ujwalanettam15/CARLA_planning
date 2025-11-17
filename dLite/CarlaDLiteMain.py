import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import tempDLiteNoGenWP
import sys
# print(sys.getrecursionlimit())
sys.setrecursionlimit(50000)

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Get the world and map
world = client.get_world()
carla_map = world.get_map()

# Spawn a firetruck at a random location (point A)
blueprint_library = world.get_blueprint_library()
firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]

spawn_points = carla_map.get_spawn_points()
# print(f'spawn_points {spawn_points[0]}')
# gen_points = carla_map.generate_waypoints(2.0)
# print(f'gen_points {gen_points[0]}')
# all_waypoints = gen_points + spawn_points[0]
# print(f'all_waypoints {all_waypoints[0]}')
# print(f'gen_points {spawn_points[0].transform}\n')
# for i in range(len(spawn_points)-1):
#     spawn_points[i]=spawn_points[i]
# print(f'spawn_points[0]: {spawn_points[0]}\n')
# print(f'spawn_points[0]: {Waypoint(spawn_points[0])}\n')
# for i in range(len(gen_points)-1):
#     gen_points[i]=gen_points[i].transform

# print(f'gen_points[0]: {gen_points[0]}')
# world.debug.draw_string(spawn_points[0].location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

# Choose a random starting location (point A)
# point_a = random.choice(spawn_points)
point_a = spawn_points[50]
print("point_a:", point_a)
firetruck = world.spawn_actor(firetruck_bp, point_a)

# # Get predefined spawn points
# predefined_spawn_points = carla_map.get_spawn_points()

# # Generate additional waypoints
# generated_waypoints = carla_map.generate_waypoints(2.0)

# # Combine predefined spawn points and generated waypoints
# spawn_points = predefined_spawn_points + [waypoint.transform for waypoint in generated_waypoints]
# point_a = random.choice(spawn_points)
# firetruck = world.spawn_actor(firetruck_bp, point_a)
# Choose a random destination (point B)
point_b = spawn_points[51]
# point_b = random.choice(spawn_points)
# while point_b.location == point_a.location:
#     point_b = random.choice(spawn_points)

start_waypoint = carla_map.get_waypoint(point_a.location)
end_waypoint = carla_map.get_waypoint(point_b.location)
world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print("Firetruck starting at", point_a.location)
print(f"Destination: {point_b.location}")

gen_points = carla_map.generate_waypoints(1)
real_points = []
wp_pts = {}
pos = 0



curr_min = gen_points[0]
for i in gen_points:
    if i.transform.location.distance(start_waypoint.transform.location) < curr_min.transform.location.distance(start_waypoint.transform.location):
        curr_min = i
get_start = curr_min
curr_min = gen_points[0]
for i in gen_points:
    if i.transform.location.distance(end_waypoint.transform.location) < curr_min.transform.location.distance(end_waypoint.transform.location):
        curr_min = i
get_end = curr_min
print(f'gen_points {gen_points[0]}')
# print(f'real_points {real_points[0]}')
for i in gen_points + [get_start] +[get_end]:
    real_points.append(carla_map.get_waypoint(i.transform.location, project_to_road=True))
    wp_pts[carla_map.get_waypoint(i.transform.location, project_to_road=True).id] = pos
    pos+=1
all_waypoints = real_points # + [get_start] +[get_end]
print(f'all_waypoints {all_waypoints[0]}')
# world.debug.draw_string(all_waypoints[0].transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print(f'get_start {get_start}')
print(f'get_end {get_end}')
world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)




print('============================================================')
try:
    # infinite loop that draw string next from start waypoint
    # while True:
    #     start_waypoint = start_waypoint.next(1)[0]
    #     world.debug.draw_string(start_waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=30.0, persistent_lines=True)
    # waypoint_tuple_list = carla_map.get_topology()
    # for i in waypoint_tuple_list:
    #     world.debug.draw_string(i[0].transform.location, 'efw', draw_shadow=False, color=carla.Color(r=250, g=0, b=00), life_time=30.0, persistent_lines=True)

    # dstar_lite = DStarLite(world=world, start_waypoint=get_start, end_waypoint=get_end, all_waypoints=all_waypoints,wp_pts=wp_pts)
    # dstar_lite = DStarLite(world, get_end, get_start, all_waypoints, wp_pts)
    # for i in range(len(all_waypoints)-3):
    #     world.debug.draw_string(all_waypoints[i].transform.location, f'{all_waypoints[i].road_id}', draw_shadow=False, color=carla.Color(r=250, g=0, b=00), life_time=30.0, persistent_lines=True)
    dstar_lite = tempDLiteNoGenWP.DStarLite(world, get_start, get_end, all_waypoints, wp_pts)
    dstar_lite.initialize()
    dstar_lite.main()

finally:
     # Clean up
    firetruck.destroy()
    print('Firetruck destroyed successfully')