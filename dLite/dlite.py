# import carla
# import random
# import time
# import threading

# from PriorityQueueDLite import PriorityQueue, Priority
# import sys
# import os
# import keyboard

# sys.setrecursionlimit(50000)

# sys.path.append(os.path.join(os.path.dirname(__file__), 'grp planning'))

# from global_route_planner import _localize

# class PlannerThread(threading.Thread):
#     """
#     Runs compute_shortest_path() in background. Writes resulting path (list of wp ids)
#     into dstar.path_ids under dstar.data_lock.
#     """
#     def __init__(self, dstar, poll_period=0.1, name="PlannerThread"):
#         super().__init__(daemon=True, name=name)
#         self.dstar = dstar
#         self.poll_period = poll_period
#         self._stop_event = threading.Event()

#     def run(self):
#         try:
#             while not self._stop_event.is_set():
#                 try:
#                     self.dstar.compute_shortest_path_threadsafe()
#                 except Exception as e:
#                     print(f"[PlannerThread] planner exception: {e}")
#                 time.sleep(self.poll_period)
#         except Exception as e:
#             print(f"[PlannerThread] fatal exception: {e}")

#     def stop(self):
#         self._stop_event.set()

# class DStarLite:
#     def __init__(self, world, start_waypoint, end_waypoint, all_waypoints, wp_pts, vehicle):
#         self.world = world
#         self.map = world.get_map()
#         self.start = start_waypoint
#         self.goal = end_waypoint
#         self.U = PriorityQueue()
#         self.km = 0

#         self.g = {}
#         self.rhs = {}
#         self.data_lock = threading.RLock()
#         self.waypoint_data = {}

#         self.s_last = None
#         self.s_current = self.start
#         self.all_waypoints = all_waypoints
#         self.wp_pos = wp_pts
#         self.part1 = []
#         self.resolution = 1.0
#         self.vehicle = vehicle

#         self.all_obst_wps = {}  
#         self.new_obst_wps = {}  
#         self.new_obst = 0
#         self.path_ids = [] 
#         self.planner_thread = None

#         print('init successfully') #debug

#     def _nearest_all_waypoint(self, wp):
#         shortest_dist = float('inf')
#         best = None
#         for z in self.all_waypoints:
#             d = wp.transform.location.distance(z.transform.location)
#             if d < shortest_dist:
#                 shortest_dist = d
#                 best = z
#         return best

#     def build_waypoint_data(self):
#         with self.data_lock:
#             self.waypoint_data = {}
#             for s in self.all_waypoints:
#                 wid = s.id
#                 self.waypoint_data[wid] = {
#                     'g': float('inf'),
#                     'rhs': float('inf'),
#                     'preds': [],     
#                     'succs': [],     
#                     'loc': s.transform.location,
#                     'wp_obj': s
#                 }

#             for s in self.all_waypoints:
#                 succs = []
#                 forward = s.next(self.resolution)
#                 if forward:
#                     for f in forward:
#                         # snap neighbor to nearest canonical waypoint
#                         snapped = self._nearest_all_waypoint(f)
#                         if snapped and snapped.id != s.id:
#                             succs.append(snapped.id)
#                 if s.lane_change & carla.LaneChange.Left:
#                     left_lane = s.get_left_lane()
#                     if left_lane and left_lane.lane_type == carla.LaneType.Driving:
#                         snapped = self._nearest_all_waypoint(left_lane)
#                         if snapped and snapped.id != s.id:
#                             succs.append(snapped.id)

#                         left_next = left_lane.next(self.resolution)
#                         if left_next:
#                             snapped = self._nearest_all_waypoint(left_next[0])
#                             if snapped and snapped.id != s.id:
#                                 succs.append(snapped.id)

#                 if s.lane_change & carla.LaneChange.Right:
#                     right_lane = s.get_right_lane()
#                     if right_lane and right_lane.lane_type == carla.LaneType.Driving:
#                         snapped = self._nearest_all_waypoint(right_lane)
#                         if snapped and snapped.id != s.id:
#                             succs.append(snapped.id)
#                         right_next = right_lane.next(self.resolution)
#                         if right_next:
#                             snapped = self._nearest_all_waypoint(right_next[0])
#                             if snapped and snapped.id != s.id:
#                                 succs.append(snapped.id)

#                 if s.transform.location.distance(self.goal.transform.location) < self.resolution + 0.5:
#                     succs.append(self.goal.id)

#                 succs = list(dict.fromkeys(succs))
#                 self.waypoint_data[s.id]['succs'] = succs

#             for s in self.all_waypoints:
#                 preds = []
#                 backward = s.previous(self.resolution)
#                 if backward:
#                     for b in backward:
#                         snapped = self._nearest_all_waypoint(b)
#                         if snapped and snapped.id != s.id:
#                             preds.append(snapped.id)

#                 if s.lane_change & carla.LaneChange.Left:
#                     left_lane = s.get_left_lane()
#                     if left_lane and left_lane.lane_type == carla.LaneType.Driving:
#                         preds.append(self._nearest_all_waypoint(left_lane).id)
#                         prevs = left_lane.previous(self.resolution)
#                         if prevs:
#                             preds.append(self._nearest_all_waypoint(prevs[0]).id)

#                 if s.lane_change & carla.LaneChange.Right:
#                     right_lane = s.get_right_lane()
#                     if right_lane and right_lane.lane_type == carla.LaneType.Driving:
#                         preds.append(self._nearest_all_waypoint(right_lane).id)
#                         prevs = right_lane.previous(self.resolution)
#                         if prevs:
#                             preds.append(self._nearest_all_waypoint(prevs[0]).id)

#                 if s.transform.location.distance(self.start.transform.location) < self.resolution + 0.5:
#                     preds.append(self.start.id)

#                 preds = [p for p in preds if p != s.id]
#                 preds = list(dict.fromkeys(preds))
#                 self.waypoint_data[s.id]['preds'] = preds

#             if self.goal.id not in self.waypoint_data:
#                 self.waypoint_data[self.goal.id] = {
#                     'g': float('inf'),
#                     'rhs': float('inf'),
#                     'preds': [],
#                     'succs': [],
#                     'loc': self.goal.transform.location,
#                     'wp_obj': self.goal
#                 }
#             if self.start.id not in self.waypoint_data:
#                 self.waypoint_data[self.start.id] = {
#                     'g': float('inf'),
#                     'rhs': float('inf'),
#                     'preds': [],
#                     'succs': [],
#                     'loc': self.start.transform.location,
#                     'wp_obj': self.start
#                 }
#             for wid, data in self.waypoint_data.items():
#                 self.g[wid] = data['g']
#                 self.rhs[wid] = data['rhs']

#     def heuristic(self, waypoint1, waypoint2):
#         return waypoint1.transform.location.distance(waypoint2.transform.location)

#     def heuristic_c_by_id(self, id1, id2):
#         with self.data_lock:
#             if id1 in self.all_obst_wps or id2 in self.all_obst_wps:
#                 return float('inf')
#             loc1 = self.waypoint_data[id1]['loc']
#             loc2 = self.waypoint_data[id2]['loc']
#             return loc1.distance(loc2)

#     def calculate_key_by_id(self, wid):
#         with self.data_lock:
#             g_val = self.waypoint_data[wid]['g']
#             rhs_val = self.waypoint_data[wid]['rhs']
#             h = self.heuristic(self.s_current, self.waypoint_data[wid]['wp_obj'])
#             k1 = min(g_val, rhs_val) + h + self.km
#             k2 = min(g_val, rhs_val)
#             return Priority(k1, k2)

#     def contain(self, wid):
#         return any(item == wid for item in self.U.vertices_in_heap)

#     def update_vertex_by_id(self, wid):
#         with self.data_lock:
#             if self.waypoint_data[wid]['g'] != self.waypoint_data[wid]['rhs'] and self.contain(wid):
#                 self.U.update_by_id(wid, self.calculate_key_by_id(wid))
#             elif self.waypoint_data[wid]['g'] != self.waypoint_data[wid]['rhs'] and not self.contain(wid):
#                 self.U.insert_by_id(wid, self.calculate_key_by_id(wid))
#             elif self.waypoint_data[wid]['g'] == self.waypoint_data[wid]['rhs'] and self.contain(wid):
#                 self.U.remove_by_id(wid)

#     def initialize(self):
#         self.U = PriorityQueue()
#         self.km = 0
#         self.build_waypoint_data()
#         with self.data_lock:
#             for wid in self.waypoint_data:
#                 self.waypoint_data[wid]['rhs'] = float('inf')
#                 self.waypoint_data[wid]['g'] = float('inf')

#             if self.goal.id in self.waypoint_data:
#                 self.waypoint_data[self.goal.id]['rhs'] = 0.0
#             else:

#                 self.waypoint_data[self.goal.id] = {
#                     'g': float('inf'),
#                     'rhs': 0.0,
#                     'preds': [],
#                     'succs': [],
#                     'loc': self.goal.transform.location,
#                     'wp_obj': self.goal
#                 }

#             self.U.insert_by_id(self.goal.id, Priority(self.heuristic(self.start, self.goal), 0))
#             print(f'self.goal {self.goal}')
#             print(f'self.U.top_key {self.U.top_key()}')

#             self.s_last = self.start
#             self.s_current = self.start

#     def compute_shortest_path_threadsafe(self):
#         with self.data_lock:
#             while (self.U.top_key() < self.calculate_key_by_id(self.start.id)) or (self.waypoint_data[self.start.id]['rhs'] > self.waypoint_data[self.start.id]['g']):
#                 u_id = self.U.top_id()  # assume PriorityQueueDLite supports top_id()
#                 if u_id is None:
#                     break
#                 k_old = self.U.top_key()
#                 k_new = self.calculate_key_by_id(u_id)

#                 if k_old < k_new:
#                     self.U.update_by_id(u_id, k_new)
#                 else:
#                     if self.waypoint_data[u_id]['g'] > self.waypoint_data[u_id]['rhs']:
#                         self.waypoint_data[u_id]['g'] = self.waypoint_data[u_id]['rhs']
#                         self.U.remove_by_id(u_id)
#                         for pred_id in self.waypoint_data[u_id]['preds']:
#                             if pred_id not in self.waypoint_data:
#                                 continue
#                             if pred_id != self.goal.id:
#                                 c = self.heuristic_c_by_id(pred_id, u_id)
#                                 new_rhs = min(self.waypoint_data[pred_id]['rhs'], c + self.waypoint_data[u_id]['g'])
#                                 self.waypoint_data[pred_id]['rhs'] = new_rhs
#                             self.update_vertex_by_id(pred_id)
#                     else:
#                         g_old = self.waypoint_data[u_id]['g']
#                         self.waypoint_data[u_id]['g'] = float('inf')
#                         pred_list = list(self.waypoint_data[u_id]['preds']) + [u_id]
#                         for s_id in pred_list:
#                             min_s = float('inf')
#                             for succ_id in self.waypoint_data[s_id]['succs']:
#                                 temp = self.heuristic_c_by_id(s_id, succ_id) + self.waypoint_data[succ_id]['g']
#                                 if temp < min_s:
#                                     min_s = temp
#                             if min_s < float('inf'):
#                                 self.waypoint_data[s_id]['rhs'] = min_s
#                             else:
#                                 self.waypoint_data[s_id]['rhs'] = float('inf')
#                             self.update_vertex_by_id(s_id)
#             path_ids = []
#             cur_id = self.start.id
#             path_ids.append(cur_id)
#             safety_iter = 0
#             while cur_id != self.goal.id and safety_iter < 10000:
#                 safety_iter += 1
#                 next_id = None
#                 min_cost = float('inf')
#                 for succ in self.waypoint_data[cur_id]['succs']:
#                     cost = self.heuristic_c_by_id(cur_id, succ) + self.waypoint_data[succ]['g']
#                     if cost < min_cost:
#                         min_cost = cost
#                         next_id = succ
#                 if next_id is None or next_id == cur_id:
#                     break
#                 path_ids.append(next_id)
#                 cur_id = next_id
#             self.path_ids = path_ids.copy()

#             for wid, d in self.waypoint_data.items():
#                 self.g[wid] = d['g']
#                 self.rhs[wid] = d['rhs']

#     def add_obst_loc_to_obst_wp(self, location):
#         """
#         Convert a given location to nearest canonical waypoint and add to obstacle sets.
#         Thread-safe.
#         """
#         wp = self.map.get_waypoint(location)
#         gen_wp = self._nearest_all_waypoint(wp)
#         if gen_wp is None:
#             return
#         with self.data_lock:
#             self.all_obst_wps[gen_wp.id] = gen_wp
#             self.new_obst_wps[gen_wp.id] = gen_wp
#             self.new_obst += 1
#             if gen_wp.id in self.waypoint_data:
#                 self.waypoint_data[gen_wp.id]['rhs'] = float('inf')
#                 self.waypoint_data[gen_wp.id]['g'] = float('inf')

#     def obs_signal(self, location):
#         self.add_obst_loc_to_obst_wp(location)

#     def rescan(self):
#         with self.data_lock:
#             if self.new_obst == 0:
#                 return False
#             else:
#                 self.new_obst = 0
#                 return True

#     def start_planner_thread(self, poll_period=0.2):
#         if self.planner_thread is None:
#             self.planner_thread = PlannerThread(self, poll_period=poll_period)
#             self.planner_thread.start()
#             print("[DStarLite] Planner thread started")

#     def stop_planner_thread(self):
#         if self.planner_thread:
#             self.planner_thread.stop()
#             self.planner_thread.join(timeout=1.0)
#             self.planner_thread = None
#             print("[DStarLite] Planner thread stopped")

#     def main(self):
#         self.initialize()
#         self.start_planner_thread(poll_period=0.1)
#         try:
#             idx = 0
#             last_path = []
#             while True:
#                 with self.data_lock:
#                     local_path = list(self.path_ids)
#                 if not local_path:
#                     time.sleep(0.05)
#                     continue
#                 if local_path != last_path:
#                     last_path = local_path

#                     veh_loc = self.vehicle.get_location()
#                     nearest_idx = 0
#                     nearest_dist = float('inf')
#                     for i, wid in enumerate(local_path):
#                         loc = self.waypoint_data.get(wid, {}).get('loc', None)
#                         if loc is None:
#                             continue
#                         d = loc.distance(veh_loc)
#                         if d < nearest_dist:
#                             nearest_dist = d
#                             nearest_idx = i
#                     idx = nearest_idx
#                 if idx >= len(local_path):
#                     print("[DStarLite] Reached end of path or no more waypoints.")
#                     break
#                 target_wid = local_path[idx]
#                 target_wp = self.waypoint_data[target_wid]['wp_obj']

#                 self.vehicle.set_transform(target_wp.transform)
#                 self.world.debug.draw_string(target_wp.transform.location, f'{idx}', draw_shadow=False,
#                                             color=carla.Color(r=220, g=0, b=0), life_time=0.5, persistent_lines=False)
#                 idx += 1
#                 time.sleep(0.05)
#                 if target_wp.transform.location.distance(self.goal.transform.location) < 3.5:
#                     print("ARRIVED")
#                     break
#                 if self.rescan():
#                     with self.data_lock:
#                         self.km += self.heuristic(self.s_last, self.start)
#                         self.s_last = self.start
#         finally:
#             self.stop_planner_thread()

# if __name__ == "__main__":
#     client = carla.Client('localhost', 2000)
#     client.set_timeout(10.0)

#     world = client.get_world()
#     carla_map = world.get_map()

#     blueprint_library = world.get_blueprint_library()
#     firetruck_bp = blueprint_library.filter('vehicle.harley-davidson.low_rider')[0]

#     spawn_points = carla_map.get_spawn_points()
#     point_a = spawn_points[50]
#     firetruck = world.spawn_actor(firetruck_bp, point_a)

#     point_b = spawn_points[5]
#     start_waypoint = carla_map.get_waypoint(point_a.location)
#     end_waypoint = carla_map.get_waypoint(point_b.location)
#     world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
#     world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
#     print("Firetruck starting at", point_a.location)
#     print(f"Destination: {point_b.location}")

#     gen_points = carla_map.generate_waypoints(1)
#     real_points = []
#     wp_pts = {}
#     pos = 0

#     curr_min = gen_points[0]
#     for i in gen_points:
#         if i.transform.location.distance(start_waypoint.transform.location) < curr_min.transform.location.distance(start_waypoint.transform.location):
#             curr_min = i
#     get_start = curr_min
#     curr_min = gen_points[0]
#     for i in gen_points:
#         if i.transform.location.distance(end_waypoint.transform.location) < curr_min.transform.location.distance(end_waypoint.transform.location):
#             curr_min = i
#     get_end = curr_min

#     for i in gen_points + [get_start] + [get_end]:
#         real_points.append(carla_map.get_waypoint(i.transform.location, project_to_road=True))
#         wp_pts[carla_map.get_waypoint(i.transform.location, project_to_road=True).id] = pos
#         pos += 1
#     all_waypoints = real_points
#     print(f'all_waypoints {all_waypoints[0]}')
#     world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
#     world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
#     print()
#     try:
#         dstar_lite = DStarLite(world, get_start, get_end, all_waypoints, wp_pts, firetruck)
#         dstar_lite.initialize()
#         dstar_lite.main()

#     finally:
#         try:
#             firetruck.destroy()
#         except Exception:
#             pass
#         print('Firetruck destroyed successfully')

import carla
import random
import time
import threading
import os
import sys
import keyboard
from concurrent.futures import ThreadPoolExecutor, as_completed
from PriorityQueueDLite import PriorityQueue, Priority

sys.setrecursionlimit(50000)
sys.path.append(os.path.join(os.path.dirname(__file__), 'grp planning'))

from global_route_planner import _localize

class PlannerThread(threading.Thread):
    """
    Single planner thread that repeatedly triggers compute_shortest_path.
    compute_shortest_path uses a ThreadPoolExecutor internally to parallelize
    expensive predecessor/successor computations.
    """
    def __init__(self, dstar, poll_period=0.2, name="PlannerThread"):
        super().__init__(daemon=True, name=name)
        self.dstar = dstar
        self.poll_period = poll_period
        self._stop_event = threading.Event()

    def run(self):
        try:
            while not self._stop_event.is_set():
                if getattr(self.dstar, "initialized", False):
                    try:
                        self.dstar.compute_shortest_path(threaded=True)
                    except Exception as e:
                        print(f"[PlannerThread] compute_shortest_path exception: {e}")
                time.sleep(self.poll_period)
        except Exception as e:
            print(f"[PlannerThread] fatal exception: {e}")

    def stop(self):
        self._stop_event.set()


class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint, all_waypoints, wp_pts, vehicle):
        self.world = world
        self.map = world.get_map()
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0

        self.g = {}
        self.rhs = {}
        self.s_last = None
        self.s_current = self.start
        self.all_waypoints = all_waypoints
        self.wp_pos = wp_pts
        self.part1 = []
        self.resolution = 1.0
        self.vehicle = vehicle

        self.all_obst_wps = {}  
        self.new_obst_wps = {}  
        self.new_obst = 0

        self.data_lock = threading.RLock()   
        self.planner_thread = None
        self.executor_workers = max(2, (os.cpu_count() or 2) - 1)
        self.initialized = False  

        print('init successfully')
        self.new_edges_and_old_costs = None
        self.path = []

    def successors(self, waypoint):
        neighbors = []
        forward = waypoint.next(self.resolution)
        if forward:
            neighbors.extend(forward)

        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)
                nxt = left_lane.next(self.resolution)
                if nxt:
                    neighbors.append(nxt[0])

        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)
                nxt = right_lane.next(self.resolution)
                if nxt:
                    neighbors.append(nxt[0])

        for i in range(len(neighbors)):
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            x = neighbors[i]
            for z in self.part1 + [self.goal]:
                if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:
                    x = z
                    initial_dist = neighbors[i].transform.location.distance(z.transform.location)
            neighbors[i] = x

        if waypoint.transform.location.distance(self.goal.transform.location) < self.resolution + .5:
            neighbors.append(self.goal)
        return neighbors

    def predecessors(self, waypoint):
        neighbors = []
        Backward = waypoint.previous(self.resolution)
        if Backward:
            neighbors.extend(Backward)

        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)
                prevs = left_lane.previous(self.resolution)
                if prevs:
                    neighbors.append(prevs[0])

        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)
                prevs = right_lane.previous(self.resolution)
                if prevs:
                    neighbors.append(prevs[0])

        try:
            test = self._localize(waypoint.transform.location)
            self.world.debug.draw_string(test.transform.location, 'SSSSSSSSSS', draw_shadow=False,
                                        color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        except Exception:
            pass

        x = [0] * len(neighbors)
        for i in range(len(neighbors)):
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            for z in self.all_waypoints:
                if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:
                    x[i] = z
                    initial_dist = neighbors[i].transform.location.distance(z.transform.location)

        for i in range(len(neighbors)):
            if x[i] != 0:
                neighbors[i] = x[i]

        self.part1.extend(neighbors)
        if waypoint.transform.location.distance(self.start.transform.location) < self.resolution + 0.5:
            neighbors.append(self.start)
        return neighbors

    def add_obst_loc_to_obst_wp(self, location):
        wp = self.map.get_waypoint(location)
        shortest_dist = float('inf')
        gen_wp = None
        for z in self.all_waypoints:
            d = wp.transform.location.distance(z.transform.location)
            if d < shortest_dist:
                gen_wp = z
                shortest_dist = d
        if gen_wp is None:
            return
        with self.data_lock:
            self.all_obst_wps[gen_wp.id] = gen_wp
            self.new_obst_wps[gen_wp.id] = gen_wp

    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)

    def heuristic_c(self, waypoint1, waypoint2):
        if waypoint1.id in self.all_obst_wps or waypoint2.id in self.all_obst_wps:
            return float('inf')
        return waypoint1.transform.location.distance(waypoint2.transform.location)

    def contain(self, u):
        return any(item == u for item in self.U.vertices_in_heap)

    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        return Priority(
            min(self.g[s.id], self.rhs[s.id]) + self.heuristic(s, self.s_current) + self.km,
            min(self.g[s.id], self.rhs[s.id])
        )

    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0

        for s in self.all_waypoints:
            self.rhs[s.id] = float('inf')
            self.g[s.id] = float('inf')

        self.g[self.goal.id] = float('inf')
        self.rhs[self.goal.id] = 0
        self.g[self.start.id] = float('inf')
        self.rhs[self.start.id] = float('inf')

        self.U.insert(self.goal, Priority(self.heuristic(self.start, self.goal), 0))

        print(f'self.goal {self.goal}')
        print(f'self.U {self.U.top_key()}')
        print(f'self.U {self.U.heap}')
        print(f'self.U {self.U.vertices_in_heap}')
        print(f'goal calculate_key {self.calculate_key(self.goal).k1}')

        self.world.debug.draw_string(self.goal.transform.location, 'goal', draw_shadow=False,
                                    color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

        pred_list = self.predecessors(self.goal)
        print(f'pred_list {pred_list}')
        print(f'pred 1 {pred_list[0] if pred_list else None}')

        self.initialized = True

    def update_vertex(self, u):
        with self.data_lock:
            if self.g[u.id] != self.rhs[u.id] and self.contain(u):
                self.U.update(u, self.calculate_key(u))
            elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
                self.U.insert(u, self.calculate_key(u))
            elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
                self.U.remove(u)

    def compute_shortest_path(self, threaded=False):
        while (self.U.top_key() < self.calculate_key(self.start)) or (self.rhs[self.start.id] > self.g[self.start.id]):
            u = self.U.top()  
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                with self.data_lock:
                    self.U.update(u, k_new)
                continue
            if self.g[u.id] > self.rhs[u.id]:
                with self.data_lock:
                    self.g[u.id] = self.rhs[u.id]
                    self.U.remove(u)
                preds = self.predecessors(u)  
                if threaded and preds:
                    with ThreadPoolExecutor(max_workers=self.executor_workers) as ex:
                        futures = {ex.submit(self._compute_rhs_update_for_pred, pred, u): pred for pred in preds}
                        for fut in as_completed(futures):
                            pred = futures[fut]
                            try:
                                new_rhs = fut.result()
                                with self.data_lock:
                                    if pred.id != self.goal.id:
                                        self.rhs[pred.id] = min(self.rhs[pred.id], new_rhs)
                                    self.update_vertex(pred)
                            except Exception as e:
                                print(f"[compute_shortest_path] pred update exception for {pred}: {e}")
                else:
                    for s in preds:
                        with self.data_lock:
                            if s != self.goal:
                                self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                            self.update_vertex(s)

            else:
                with self.data_lock:
                    g_old = self.g[u.id]
                    self.g[u.id] = float('inf')
                pred = self.predecessors(u)
                pred.append(u)

                if threaded and pred:
                    with ThreadPoolExecutor(max_workers=self.executor_workers) as ex:
                        futures = {ex.submit(self._recompute_rhs_for_vertex, s): s for s in pred}
                        for fut in as_completed(futures):
                            s = futures[fut]
                            try:
                                new_rhs = fut.result()
                                with self.data_lock:
                                    self.rhs[s.id] = new_rhs
                                    self.update_vertex(s)
                            except Exception as e:
                                print(f"[compute_shortest_path] recompute rhs exception for {s}: {e}")
                else:
                    for s in pred:
                        min_s = float('inf')
                        succ = self.successors(s)
                        for s_ in succ:
                            temp = self.heuristic_c(s, s_) + self.g[s_.id]
                            if min_s > temp:
                                min_s = temp
                        with self.data_lock:
                            self.rhs[s.id] = min_s
                            self.update_vertex(s)

    def _compute_rhs_update_for_pred(self, pred, u):
        val = self.heuristic_c(pred, u) + self.g[u.id]
        return val

    def _recompute_rhs_for_vertex(self, s):
        min_s = float('inf')
        succ = self.successors(s)
        for s_ in succ:
            temp = self.heuristic_c(s, s_) + self.g[s_.id]
            if temp < min_s:
                min_s = temp
        return min_s

    def obs_signal(self, location):
        self.new_obst += 1
        self.add_obst_loc_to_obst_wp(location)

    def rescan(self):
        if self.new_obst == 0:
            return False
        else:
            self.new_obst = 0
            return True
        
    def main(self):
        self.s_last = self.start
        self.s_current = self.start
        path = [self.s_current]
        flag = 0

        self.initialize()
        self.planner_thread = PlannerThread(self, poll_period=0.1)
        self.planner_thread.start()

        try:
            self.compute_shortest_path(threaded=True)

            print(f'self.s_current before while{self.s_current}')
            while self.s_current.transform.location.distance(self.goal.transform.location) >= 3.5:
                if self.rhs[self.start.id] == float('inf'):
                    print("There is no known path to the goal.")
                    return

                successor = self.successors(self.s_current)
                if not successor:
                    print("No valid successor found.")
                    return
                min_s = float('inf')
                arg_min = None
                for s_ in successor:
                    temp = self.heuristic_c(self.s_current, s_) + self.g[s_.id]
                    print(temp)
                    if temp <= min_s:
                        min_s = temp
                        arg_min = s_

                self.s_current = arg_min
                self.vehicle.set_transform(self.s_current.transform)

                if self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location) < 25 and flag == 0:
                    flag = 1
                    self.obs_signal(self.all_waypoints[2812].transform.location)
                    self.obs_signal(self.all_waypoints[2816].transform.location)
                    self.obs_signal(self.all_waypoints[2820].transform.location)
                    self.obs_signal(self.all_waypoints[2824].transform.location)
                    print("\n\n\nadded obstacle")

                if self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location) < 15 and flag == 1:
                    flag = 2
                    for i in self.all_waypoints:
                        self.world.debug.draw_string(i.transform.location, f'{self.g[i.id]}', draw_shadow=False,
                                                    color=carla.Color(r=0, g=220, b=220), life_time=20.0,
                                                    persistent_lines=True)

                time.sleep(.01)

                if self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location) < 5:
                    self.world.debug.draw_string(self.s_current.transform.location, f'{self.rhs[self.s_current.id]}',
                                                draw_shadow=False, color=carla.Color(r=220, g=200, b=200), life_time=30.0,
                                                persistent_lines=True)
                    time.sleep(2)

                if self.s_current.transform.location.distance(self.goal.transform.location) < 3.5:
                    print('ARRIVED')

                if self.rescan():
                    with self.data_lock:
                        self.km += self.heuristic_c(self.s_last, self.start)
                        self.s_last = self.start

        finally:
            if self.planner_thread:
                self.planner_thread.stop()
                self.planner_thread.join(timeout=1.0)
            print(f"{self.all_obst_wps}")
            print('Done!')

if __name__ == "__main__":
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()

    blueprint_library = world.get_blueprint_library()
    firetruck_bp = blueprint_library.filter('vehicle.harley-davidson.low_rider')[0]

    spawn_points = carla_map.get_spawn_points()
    point_a = spawn_points[50]
    print("point_a:", point_a)
    firetruck = world.spawn_actor(firetruck_bp, point_a)

    point_b = spawn_points[5]
    start_waypoint = carla_map.get_waypoint(point_a.location)
    end_waypoint = carla_map.get_waypoint(point_b.location)
    world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False,
                            color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False,
                            color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    print("Firetruck starting at", point_a.location)
    print(f"Destination: {point_b.location}")

    gen_points = carla_map.generate_waypoints(1)
    real_points = []
    wp_pts = {}
    pos = 0

    curr_min = gen_points[0]
    for i in gen_points:
        if i.transform.location.distance(start_waypoint.transform.location) < curr_min.transform.location.distance(
                start_waypoint.transform.location):
            curr_min = i
    get_start = curr_min
    curr_min = gen_points[0]
    for i in gen_points:
        if i.transform.location.distance(end_waypoint.transform.location) < curr_min.transform.location.distance(
                end_waypoint.transform.location):
            curr_min = i
    get_end = curr_min

    for i in gen_points + [get_start] + [get_end]:
        real_points.append(carla_map.get_waypoint(i.transform.location, project_to_road=True))
        wp_pts[carla_map.get_waypoint(i.transform.location, project_to_road=True).id] = pos
        pos += 1
    all_waypoints = real_points
    print(f'all_waypoints {all_waypoints[0]}')
    world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False,
                            color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False,
                            color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    print('============================================================')
    try:
        dstar_lite = DStarLite(world, get_start, get_end, all_waypoints, wp_pts, firetruck)
        dstar_lite.initialize()
        dstar_lite.main()
    finally:
        firetruck.destroy()
        print('Firetruck destroyed successfully')
