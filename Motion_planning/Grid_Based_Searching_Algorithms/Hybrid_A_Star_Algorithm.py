import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from queue import PriorityQueue
import math
import random
import dubins

class Node:
    def __init__(self, x, y, theta, g=0, h=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = None
        self._state_key = None

    def __lt__(self, other):
        return self.f < other.f
    
    def get_state_key(self):
        if self._state_key is None:
            x_disc = round(self.x / 0.5)
            y_disc = round(self.y / 0.5)
            theta_disc = round(self.theta / (np.pi/8))
            self._state_key = (x_disc, y_disc, theta_disc)
        return self._state_key

    def __eq__(self, other):
        return self.get_state_key() == other.get_state_key()

    def __hash__(self):
        return hash(self.get_state_key())

class Hybrid_A_Star_Planner:
    def __init__(self, start, goal, obstacles, map_size, goal_tolerance, 
                 resolution=0.5, turning_radius=2.0, wheel_base_length=2.0):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.map_size = map_size
        self.goal_tolerance = goal_tolerance
        self.resolution = resolution
        self.turning_radius = turning_radius
        self.L = wheel_base_length
        self.open_set = PriorityQueue()
        self.open_set.put(self.start)
        self.closed_set = set()
        self.cost_so_far = {self.start.get_state_key(): 0}
        self.goal_reached = False
        self.path = []
        self.node_artists = []
        self.fig, self.ax = plt.subplots()
        self.setup_visualization()
        
        # OPTIMIZATION: Cache collision checks
        self.collision_cache = {}
    
    def generate_motion_primitives(self, node):
        """OPTIMIZED: Fewer primitives for speed"""
        motions = []
        step_size = 1.5 * self.resolution  # Larger steps
        
        # REDUCED: Only 5 angles instead of 7
        angles = [-40, -20, 0, 20, 40]
        
        for angle in angles:
            if angle == 0:
                dx = step_size * math.cos(node.theta)
                dy = step_size * math.sin(node.theta)
                d_theta = 0
            else:
                angle_rad = math.radians(angle)
                d_theta = (step_size * math.tan(angle_rad)) / self.L
                turning_radius = abs(step_size / d_theta) if abs(d_theta) > 1e-6 else np.inf
                dx = turning_radius * (math.sin(node.theta + d_theta) - math.sin(node.theta))
                dy = -turning_radius * (math.cos(node.theta + d_theta) - math.cos(node.theta))
            
            motions.append((dx, dy, d_theta, abs(angle) * 0.05))
        
        return motions
    
    def snap_to_grid(self, x, y, theta):
        xg = round(x / self.resolution) * self.resolution
        yg = round(y / self.resolution) * self.resolution
        thetag = round(theta / (np.pi / 8)) * (np.pi / 8)
        return xg, yg, thetag
    
    def within_goal_tolerance(self, node):
        dist = np.hypot(node.x - self.goal.x, node.y - self.goal.y)
        pos_ok = dist <= self.goal_tolerance
        angle_diff = abs((node.theta - self.goal.theta + np.pi) % (2*np.pi) - np.pi)
        yaw_ok = angle_diff < np.pi/6
        return pos_ok and yaw_ok
    
    def reconstruct_path(self, node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path
    
    def simple_collision_check(self, x1, y1, x2, y2, num_samples=5):
        """FAST: Simple line collision check (no Dubins)"""
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            # Check bounds
            if x < 0 or x > self.map_size[0] or y < 0 or y > self.map_size[1]:
                return False
            
            # Check obstacles
            for ox, oy, r in self.obstacles:
                if np.hypot(x - ox, y - oy) <= r + 0.6:
                    return False
        
        return True
    
    def dubins_collision_check(self, start_node, end_node):
        """CACHED: Full Dubins collision check with caching"""
        cache_key = (start_node.get_state_key(), end_node.get_state_key())
        
        if cache_key in self.collision_cache:
            return self.collision_cache[cache_key]
        
        try:
            start_pose = (start_node.x, start_node.y, start_node.theta)
            end_pose = (end_node.x, end_node.y, end_node.theta)
            path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
            configurations, _ = path.sample_many(0.15)  # Coarser sampling
            
            for x, y, _ in configurations:
                if x < 0 or x > self.map_size[0] or y < 0 or y > self.map_size[1]:
                    self.collision_cache[cache_key] = False
                    return False
                
                for ox, oy, r in self.obstacles:
                    if np.hypot(x - ox, y - oy) <= r + 0.5:
                        self.collision_cache[cache_key] = False
                        return False
            
            self.collision_cache[cache_key] = True
            return True
        except:
            self.collision_cache[cache_key] = False
            return False
    
    def heuristic(self, node):
        """Simple Euclidean heuristic"""
        return np.hypot(node.x - self.goal.x, node.y - self.goal.y)
    
    def optimize_path(self):
        """Quick path optimization"""
        if not self.path or len(self.path) < 3:
            return self.path
        
        optimized = [self.path[0]]
        i = 0
        
        while i < len(self.path) - 1:
            j = len(self.path) - 1
            found = False
            
            while j > i + 1:
                if self.dubins_collision_check(self.path[i], self.path[j]):
                    optimized.append(self.path[j])
                    i = j
                    found = True
                    break
                j -= 1
            
            if not found:
                i += 1
                if i < len(self.path):
                    optimized.append(self.path[i])
        
        return optimized
    
    def setup_visualization(self):
        self.ax.plot(self.start.x, self.start.y, 'go', markersize=12, label='Start')
        
        arrow_length = 2.0
        dx_start = arrow_length * np.cos(self.start.theta)
        dy_start = arrow_length * np.sin(self.start.theta)
        self.ax.arrow(
            self.start.x, self.start.y,  # Origin
            dx_start, dy_start,          # Direction vector
            head_width=0.4,
            head_length=0.3,
            fc='green',
            ec='darkgreen',
            linewidth=3,
            length_includes_head=True,  
            zorder=8
            )
        self.ax.plot(self.goal.x, self.goal.y, 'r*', markersize=15, label='Goal')
        dx_goal = arrow_length * np.cos(self.goal.theta)
        dy_goal = arrow_length * np.sin(self.goal.theta)
        self.ax.arrow(
            self.goal.x, self.goal.y,    # Origin
            dx_goal, dy_goal,            # Direction vector
            head_width=0.4,
            head_length=0.3,
            fc='red',
            ec='darkred',
            linewidth=3,
            length_includes_head=True,   
            zorder=8
        )
        goal_circle = Circle((self.goal.x, self.goal.y), self.goal_tolerance, 
                            color='red', fill=False, linestyle='--', alpha=0.3)
        self.ax.add_artist(goal_circle)
        
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()
        self.ax.set_aspect('equal')
        self.draw_obstacles()
        
        # Add text for progress indicators
        self.progress_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )

    
    def draw_obstacles(self):
        for ox, oy, r in self.obstacles:
            circle = Circle((ox, oy), r, color='gray', alpha=0.6)
            self.ax.add_artist(circle)
    
    def draw_path(self):
        if not self.path or len(self.path) < 2:
            return
        
        for i in range(len(self.path) - 1):
            from_node = self.path[i]
            to_node = self.path[i + 1]
            
            try:
                start_pose = (from_node.x, from_node.y, from_node.theta)
                end_pose = (to_node.x, to_node.y, to_node.theta)
                path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
                configurations, _ = path.sample_many(0.05)
                
                xs, ys = [], []
                for x, y, _ in configurations:
                    if 0 <= x <= self.map_size[0] and 0 <= y <= self.map_size[1]:
                        xs.append(x)
                        ys.append(y)
                
                if xs and ys:
                    self.ax.plot(xs, ys, '-g', linewidth=3, 
                                label='Final Path' if i == 0 else '', zorder=10)
            except:
                continue

def create_obstacles(num_obstacles, map_size, start, goal):
    obstacles = []
    for _ in range(num_obstacles):
        attempts = 0
        while attempts < 100:
            ox = random.uniform(2, map_size[0] - 2)
            oy = random.uniform(2, map_size[1] - 2)
            r = random.uniform(0.8, 1.5)
            
            dist_start = np.hypot(ox - start.x, oy - start.y)
            dist_goal = np.hypot(ox - goal.x, oy - goal.y)
            
            if dist_start > r + 3 and dist_goal > r + 3:
                obstacles.append((ox, oy, r))
                break
            attempts += 1
    
    return obstacles

# Animation state
animation_state = {
    'open_list': None,
    'closed_list': None,
    'cost_so_far': None,
    'initialized': False,
    'nodes_processed': 0,
    'start_time': None
}

def animate(frame_num):
    global planner
    import time
    
    if planner.goal_reached:
        return planner.node_artists
    
    if not animation_state['initialized']:
        animation_state['open_list'] = PriorityQueue()
        animation_state['closed_list'] = set()
        animation_state['cost_so_far'] = {planner.start.get_state_key(): 0}
        animation_state['open_list'].put(planner.start)
        animation_state['initialized'] = True
        animation_state['start_time'] = time.time()
        print(f"Start: ({planner.start.x:.1f}, {planner.start.y:.1f}, {np.rad2deg(planner.start.theta):.0f}°)")
        print(f"Goal: ({planner.goal.x:.1f}, {planner.goal.y:.1f}, {np.rad2deg(planner.goal.theta):.0f}°)")
    
    open_list = animation_state['open_list']
    closed_list = animation_state['closed_list']
    cost_so_far = animation_state['cost_so_far']
    
    # OPTIMIZED: Process more nodes per frame
    nodes_per_frame = 10
    processed = 0
    
    while not open_list.empty() and processed < nodes_per_frame:
        current = open_list.get()
        
        if current in closed_list:
            continue
        
        closed_list.add(current)
        processed += 1
        animation_state['nodes_processed'] += 1
        
        # Update progress every 10 nodes
        if animation_state['nodes_processed'] % 10 == 0:
            elapsed = time.time() - animation_state['start_time']
            dist_to_goal = np.hypot(current.x - planner.goal.x, current.y - planner.goal.y)
            progress_info = (
                f"Frame: {frame_num}\n"
                f"Nodes explored: {animation_state['nodes_processed']}\n"
                f"Open list: {open_list.qsize()}\n"
                f"Closed: {len(closed_list)}\n"
                f"Distance to goal: {dist_to_goal:.2f}\n"
                f"Time: {elapsed:.1f}s"
            )
            planner.progress_text.set_text(progress_info)
            
            # Console progress bar
            if animation_state['nodes_processed'] % 50 == 0:
                print(f"⏳ Frame {frame_num:3d} | Nodes: {animation_state['nodes_processed']:4d} | "
                      f"Open: {open_list.qsize():3d} | Dist: {dist_to_goal:.2f} | "
                      f"Time: {elapsed:.1f}s")
        
        # Visualize every 3rd node only
        if animation_state['nodes_processed'] % 3 == 0:
            planner.node_artists.append(
                planner.ax.plot(current.x, current.y, 'c.', markersize=2, alpha=0.3)[0]
            )
        
        # Check goal - STOP IMMEDIATELY when reached
        if planner.within_goal_tolerance(current):
            elapsed = time.time() - animation_state['start_time']
            planner.goal_reached = True
            planner.path = planner.reconstruct_path(current)
            planner.path = planner.optimize_path()
            planner.draw_path()
            
            final_info = (
                f" GOAL REACHED!\n"
                f"Path cost: {planner.path[-1].f:.2f}\n"
                f"Path nodes: {len(planner.path)}\n"
                f"Total explored: {animation_state['nodes_processed']}\n"
                f"Time: {elapsed:.1f}s"
            )
            planner.progress_text.set_text(final_info)
            
            print(f"\n{'='*60}")
            print(f" GOAL REACHED!")
            print(f"{'='*60}")
            print(f"  Path cost: {planner.path[-1].f:.2f}")
            print(f"  Path nodes: {len(planner.path)}")
            print(f"  Total explored: {animation_state['nodes_processed']}")
            print(f"  Search time: {elapsed:.2f}s")
            print(f"  Nodes/sec: {animation_state['nodes_processed']/elapsed:.0f}")
            print(f"{'='*60}\n")
            
            # Stop processing more nodes this frame
            break
        
        # Try direct Dubins connection if close
        dist_to_goal = np.hypot(current.x - planner.goal.x, current.y - planner.goal.y)
        if dist_to_goal < 2.0:
            goal_node = Node(planner.goal.x, planner.goal.y, planner.goal.theta)
            if planner.dubins_collision_check(current, goal_node):
                try:
                    dub = dubins.shortest_path(
                        (current.x, current.y, current.theta),
                        (goal_node.x, goal_node.y, goal_node.theta),
                        planner.turning_radius
                    )
                    goal_node.g = current.g + dub.path_length()
                    goal_node.h = 0
                    goal_node.f = goal_node.g
                    goal_node.parent = current
                    
                    elapsed = time.time() - animation_state['start_time']
                    planner.goal_reached = True
                    planner.path = planner.reconstruct_path(goal_node)
                    planner.path = planner.optimize_path()
                    planner.draw_path()
                    
                    final_info = (
                        f" DIRECT CONNECTION!\n"
                        f"Path cost: {goal_node.f:.2f}\n"
                        f"Path nodes: {len(planner.path)}\n"
                        f"Total explored: {animation_state['nodes_processed']}\n"
                        f"Time: {elapsed:.1f}s"
                    )
                    planner.progress_text.set_text(final_info)
                    
                    print(f"\n{'='*60}")
                    print(f" DIRECT CONNECTION TO GOAL!")
                    print(f"{'='*60}")
                    print(f"  Path cost: {goal_node.f:.2f}")
                    print(f"  Path nodes: {len(planner.path)}")
                    print(f"  Total explored: {animation_state['nodes_processed']}")
                    print(f"  Search time: {elapsed:.2f}s")
                    print(f"{'='*60}\n")
                    
                    # Stop processing more nodes this frame
                    break
                except:
                    pass
        
        # Expand neighbors
        for dx, dy, dtheta, steering_cost in planner.generate_motion_primitives(current):
            x_new = current.x + dx
            y_new = current.y + dy
            theta_new = (current.theta + dtheta) % (2 * np.pi)
            
            x_new, y_new, theta_new = planner.snap_to_grid(x_new, y_new, theta_new)
            neighbor = Node(x_new, y_new, theta_new)
            
            if neighbor in closed_list:
                continue
            
            if not (0 <= x_new <= planner.map_size[0] and 0 <= y_new <= planner.map_size[1]):
                continue
            
            # OPTIMIZED: Use simple collision check first
            if not planner.simple_collision_check(current.x, current.y, x_new, y_new):
                continue
            
            step_cost = np.hypot(dx, dy) + steering_cost
            tentative_g = current.g + step_cost
            
            state_key = neighbor.get_state_key()
            if state_key in cost_so_far and tentative_g >= cost_so_far[state_key]:
                continue
            
            cost_so_far[state_key] = tentative_g
            neighbor.g = tentative_g
            neighbor.h = planner.heuristic(neighbor)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current
            
            open_list.put(neighbor)
    
    if open_list.empty() and not planner.goal_reached:
        elapsed = time.time() - animation_state['start_time']
        fail_info = (
            f"  NO PATH FOUND\n"
            f"Nodes explored: {animation_state['nodes_processed']}\n"
            f"Time: {elapsed:.1f}s"
        )
        planner.progress_text.set_text(fail_info)
        print(f"\n  No path found after {animation_state['nodes_processed']} nodes!")
    
    return planner.node_artists

if __name__ == "__main__":
    import time
    
    start = Node(2, 2, np.deg2rad(0))
    goal = Node(18, 18, np.deg2rad(-45))
    map_size = [20, 20]
    num_obstacles = 6  # Reduced for faster testing
    
    print("="*60)
    print("HYBRID A* PATH PLANNER")
    print("="*60)
    
    obstacles = create_obstacles(num_obstacles, map_size, start, goal)
    planner = Hybrid_A_Star_Planner(
        start, goal, obstacles, map_size,
        resolution=0.5,
        goal_tolerance=0.5,
        turning_radius=2.0
    )
    
    print("\n  Starting search...")
    print(f"   Map size: {map_size}")
    print(f"   Obstacles: {num_obstacles}")
    print(f"   Resolution: {planner.resolution}")
    print(f"   Turning radius: {planner.turning_radius}")
    print("-" * 60)
    
    start_time = time.time()
    
    ani = animation.FuncAnimation(
        planner.fig,
        animate,
        frames=1000,  # REDUCED: 500 instead of 2000
        interval=20,  # FASTER: 20ms instead of 30ms
        repeat=False,
        blit=False
    )
    
    print("\n  Saving animation...")
    save_start = time.time()
    ani.save("Hybrid_A_Star_animation.gif", writer="pillow", fps=50)  # Higher FPS
    save_time = time.time() - save_start
    total_time = time.time() - start_time
    
    print(f"\n{'='*60}")
    print(f" ANIMATION COMPLETE!")
    print(f"{'='*60}")
    print(f"  Save time: {save_time:.2f}s")
    print(f"  Total time: {total_time:.2f}s")
    print(f"  Output: Hybrid_A_Star_animation.gif")
    print(f"{'='*60}\n")
    
 