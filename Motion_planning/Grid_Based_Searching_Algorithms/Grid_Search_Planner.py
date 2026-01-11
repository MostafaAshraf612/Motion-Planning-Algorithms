import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from queue import PriorityQueue
import random
import dubins
class Node:
    def __init__(self , x , y ,theta = 0.0, cost = 0.0  , parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.heuristic = 0.0
        self.f_cost = 0.0
        self.parent = parent

    def __lt__(self, other):
        return (self.f_cost) < (other.f_cost)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class Grid_Search_Planner:

    def __init__(self,start, goal, obstacles, map_size , planner = "Dijkstra", max_iter=1000 , holonomic=True ,self_turning_radius=1.0):
        self.planner = planner
        self.holonomic = holonomic

        if self.holonomic:
            self.start = Node(start[0], start[1], 0.0 , 0.0)
            self.goal = Node(goal[0], goal[1], 0.0 , 0.0)
        else:
            self.start = Node(start[0], start[1], start[2] , 0.0)
            self.goal = Node(goal[0], goal[1], goal[2] , 0.0)
            self.turning_radius = self_turning_radius

        self.obstacles = obstacles
        self.map_size = map_size
        self.goal_reached = False
        self.max_iter = max_iter
        self.node_artists = []
        self.path = []
        self.fig, self.ax = plt.subplots()
        self.setup_visualization()
        
    def heuristic(self , node):
        return  np.hypot(node.x - self.goal.x, node.y - self.goal.y) # Euclidean distance 
    
    def plan(self):
        open_list = PriorityQueue()
        closed_list = [] # visited nodes
        open_list.put(self.start)
        exploration_directions = [(1,0) , (0,1) , (-1,0) , (0,-1) , (1,1) , (1,-1) , (-1,1) , (-1,-1)]
        while not open_list.empty():

            active_node = open_list.get() # node with lowest cost + heuristic

            if active_node == self.goal:
                self.goal_reached = True
                break # goal reached

            for (move_x , move_y) in exploration_directions:
                new_node = Node(active_node.x + move_x, active_node.y + move_y)

                if new_node in closed_list:
                    continue # already visited

                if (new_node.x < 0 or new_node.x >= self.map_size[0] or
                    new_node.y < 0 or new_node.y >= self.map_size[1]):
                    continue # out of bounds

                if self.node_collision_check(new_node, self.obstacles):
                    continue # collision with obstacle

                new_node.cost = active_node.cost  + np.hypot(move_x, move_y)

                if self.planner == "A_star":
                    new_node.heuristic = self.heuristic(new_node)
                elif self.planner == "Dijkstra":
                    new_node.heuristic = 0.0

                new_node.f_cost = new_node.cost + new_node.heuristic
                new_node.parent = active_node

                if new_node in open_list.queue:
                    if new_node.f_cost < open_list.queue[open_list.queue.index(new_node)].f_cost:
                        open_list.queue.remove(open_list.queue[open_list.queue.index(new_node)])
                        open_list.put(new_node)
                else:
                    open_list.put(new_node)

            closed_list.append(active_node)

        if self.goal_reached:
            return self.reconstruct_path(active_node)
        else:
            print("No path found")
            return None
        
    def path_collision_check(self, start_node, end_node, resolution=0.02):
        """
        Check whether the Dubins path is collision-free.
        """
        start_pose = (start_node.x, start_node.y, start_node.theta)
        end_pose = (end_node.x, end_node.y, end_node.theta)

        # Create Dubins path with the turning radius
        path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)

        # Sample points along the path at intervals of 'resolution'
        configurations, _ = path.sample_many(resolution)

        for x, y, _ in configurations:

            if x < 0 or x > self.map_size[0] or y < 0 or y > self.map_size[1]:
                    return False
            
            for ox, oy, r in self.obstacles:
                if np.linalg.norm((x - ox, y - oy)) <= r + 0.5:  # obstacle radius + margin
                    return False  # Collision detected

        return True  # Path is free of obstacles    
    
    def node_collision_check(self, node, obstacles):
        for obstacle in obstacles:
            dist = np.hypot(node.x - obstacle[0], node.y - obstacle[1])
            if dist <= obstacle[2] + 0.2: # obstacle radius
                return True
        return False
    
    def reconstruct_path(self, node):
        if self.holonomic:
            print("Reconstructing holonomic path...")
            path = []
            while node is not None:
                path.append(node)
                node = node.parent
        else:
            print("Reconstructing non-holonomic path...")
            path = []
            while node is not None:
                path.append(node)
                node = node.parent
        path.reverse()
        return path

    # ============================
    # Visualization helpers
    # ============================

    def setup_visualization(self):
        """
        Draw start, goal, obstacles, grid, and axis limits.
        """
        self.ax.plot(self.start.x, self.start.y, 'go', markersize=12, label='Start')
        self.ax.plot(self.goal.x, self.goal.y, 'r*', markersize=15, label='Goal')
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.grid(True)
        self.ax.legend()
        self.draw_obstacles()

    def draw_obstacles(self):
        """
        Draw circular obstacles on the map.
        """
        for obstacle in self.obstacles:
            circle = Circle((obstacle[0], obstacle[1]), obstacle[2], color='gray', alpha=0.5)
            self.ax.add_artist(circle)
    
    def optimize_path(self):
        if not self.path or len(self.path) < 3:
            return self.path

        # First, set theta for all nodes based on direction to next node
        for i in range(len(self.path) - 1):
            dx = self.path[i+1].x - self.path[i].x
            dy = self.path[i+1].y - self.path[i].y
            self.path[i].theta = np.arctan2(dy, dx)

        # Set last node theta same as second-to-last
        if len(self.path) > 1:
            self.path[-1].theta = self.path[-2].theta

        # INCREASE collision margin for optimization
        for _ in range(4):
            i = 0
            while i < len(self.path) - 2:
                j = len(self.path) - 1
                while j > i + 1:
                    # Use stricter collision check with larger margin
                    if self.path_collision_check(self.path[i], self.path[j]):
                        start_pose = (self.path[i].x, self.path[i].y, self.path[i].theta)
                        end_pose = (self.path[j].x, self.path[j].y, self.path[j].theta)
                        dubins_path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
                        
                        self.path[j].parent = self.path[i]
                        self.path[j].cost = self.path[i].cost + dubins_path.path_length()
                        del self.path[i+1:j]
                        break
                    j -= 1
                i += 1
        return self.path
    
    def draw_path(self):
        """
        Draw the final path using Dubins curves.
        """  
        if not self.path:
                return
        
        if self.holonomic:
            for i in range(len(self.path) - 1):
                xs = []
                ys = []
                # Only include points within bounds
                for node in self.path:
                    if 0 <= node.x <= self.map_size[0] and 0 <= node.y <= self.map_size[1]:
                        xs.append(node.x)
                        ys.append(node.y)

                if xs and ys:  # Only draw if we have valid points
                    self.ax.plot(xs, ys, '-g', linewidth=3, 
                                label='Final Path' if i == 0 else '', zorder=10)
       
        else:
            for i in range(len(self.path) - 1):
                from_node = self.path[i]
                to_node = self.path[i + 1]

                start_pose = (from_node.x, from_node.y, from_node.theta)
                end_pose = (to_node.x, to_node.y, to_node.theta)
                path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
                
                configurations, _ = path.sample_many(0.05)

                xs = []
                ys = []
                
                # Only include points within bounds
                for x, y, _ in configurations:
                    if 0 <= x <= self.map_size[0] and 0 <= y <= self.map_size[1]:
                        xs.append(x)
                        ys.append(y)

                if xs and ys:  # Only draw if we have valid points
                    self.ax.plot(xs, ys, '-g', linewidth=3, 
                            label='Final Path' if i == 0 else '', zorder=10)


def create_obstacles(obstacles_num, map_size , start, goal):
        """
        Randomly generate circular obstacles.
        Each obstacle = (x, y, radius)
        """
        obstacles = []
        for _ in range(obstacles_num):
            while True:
                obs_x = random.uniform(3, map_size[0] - 3)
                obs_y = random.uniform(3, map_size[1] - 3)
                size = random.uniform(0.8, 2.0)
                
                # Check if obstacle overlaps with start or goal
                dist_to_start = np.linalg.norm([obs_x - start[0], obs_y - start[1]])
                dist_to_goal = np.linalg.norm([obs_x - goal[0], obs_y - goal[1]])

                if dist_to_start > size + 2 and dist_to_goal > size + 2:
                    obstacles.append((obs_x, obs_y, size))
                    break
        return obstacles


# ============================
# Animation function
# ============================

def animate(i):
    """
    Animation function that shows tree progression.
    """
    if planner.goal_reached:
        return planner.node_artists
    
    # Run A* algorithm with node visualization
    open_list = PriorityQueue()
    closed_list = []
    open_list.put(planner.start)
    exploration_directions = [(1,0), (0,1), (-1,0), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
    nodes_per_frame = 2
    nodes_processed = 0
    target_nodes = (i + 1) * nodes_per_frame
    
    while not open_list.empty() and nodes_processed < target_nodes:
        active_node = open_list.get()
        nodes_processed += 1
        
        # Draw this node if it's new in this frame
        if nodes_processed > i * nodes_per_frame:
            node_artist = planner.ax.plot(active_node.x, active_node.y, 'co', 
                                    markersize=3, alpha=0.3)[0]
            planner.node_artists.append(node_artist)
        
        if active_node == planner.goal:
            planner.goal_reached = True
            planner.path = planner.reconstruct_path(active_node)

            if not planner.holonomic:
                planner.path = planner.optimize_path()
               
            planner.draw_path()

            print(f"Goal reached at frame {i}!")
            print(f"Path length: {planner.path[-1].f_cost:.2f}")
            print(f"Number of nodes: {len(planner.path)}")
            break
        
        for (move_x, move_y) in exploration_directions:
            new_node = Node(active_node.x + move_x, active_node.y + move_y)
            
            if new_node in closed_list:
                continue
            
            if (new_node.x < 0 or new_node.x >= planner.map_size[0] or
                new_node.y < 0 or new_node.y >= planner.map_size[1]):
                continue

            if planner.node_collision_check(new_node, planner.obstacles):
                continue
            
            new_node.cost = active_node.cost + np.hypot(move_x, move_y)
            if planner.planner == "A_star":
                new_node.heuristic = planner.heuristic(new_node)
            else:  # Dijkstra
                new_node.heuristic = 0.0
            new_node.f_cost = new_node.cost + new_node.heuristic
            new_node.parent = active_node
            
            if new_node in open_list.queue:
                idx = open_list.queue.index(new_node)
                if new_node.cost < open_list.queue[idx].cost:
                    open_list.queue.remove(open_list.queue[idx])
                    open_list.put(new_node)
            else:
                open_list.put(new_node)
        
        closed_list.append(active_node)
    
    if open_list.empty() and not planner.goal_reached:
        print("No path found")
    
    return planner.node_artists

# ============================
# Main execution
# ============================

if __name__ == "__main__":

    start = [1, 1 , np.deg2rad(0.0)]
    goal  = [15, 15 , np.deg2rad(60.0)]
    num_obstacles = 10
    map_size = [20, 20]
   

    print("="*50)
    print("A _Star Planning")
    print("="*50)
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Map size: {map_size}")
    print(f"Number of obstacles: {num_obstacles}")

    obstacles = create_obstacles(num_obstacles, map_size , start, goal)
    A_star = Grid_Search_Planner(start, goal, obstacles, map_size,"A_star", max_iter=200 , holonomic=True )
    A_star_NH = Grid_Search_Planner(start, goal, obstacles, map_size,"A_star", max_iter=200 , holonomic=False , self_turning_radius=1.0)
    Dijkstra = Grid_Search_Planner(start, goal, obstacles, map_size,"Dijkstra", max_iter=200 , holonomic=True )
    Dijkstra_NH = Grid_Search_Planner(start, goal, obstacles, map_size,"Dijkstra", max_iter=200 , holonomic=False , self_turning_radius=1.0)
    
    print("="*50)
    print("A _Star Planning")
    print("="*50)
    print("Starting animation...")
    planner = A_star
    ani = animation.FuncAnimation(
        A_star.fig,
        animate,
        frames=A_star.max_iter,
        interval=50,
        repeat=False,
        blit=False
    )
    print("Saving animation...")
    ani.save("A_star_animation.gif", writer="pillow", fps=30)
    print("Animation completed and saved as 'A_star_animation.gif'")

    print("="*50)
    print("A _Star_NH Planning")
    print("="*50)
    print("Starting animation...")
    planner = A_star_NH
    ani = animation.FuncAnimation(
        A_star_NH.fig,
        animate,
        frames=A_star_NH.max_iter,
        interval=50,
        repeat=False,
        blit=False
    )
    print("Saving animation...")
    ani.save("A_star_NH_animation.gif", writer="pillow", fps=30)
    print("Animation completed and saved as 'A_star_NH_animation.gif'")
   
    print("="*50)
    print("Dijkstra Planning")
    print("="*50)
    print("Starting animation...")
    planner = Dijkstra
    ani = animation.FuncAnimation(
        Dijkstra.fig,
        animate,
        frames=Dijkstra.max_iter,
        interval=50,
        repeat=False,
        blit=False
    )
    print("Saving animation...")
    ani.save("Dijkstra_animation.gif", writer="pillow", fps=30)
    print("Animation completed and saved as 'Dijkstra_animation.gif'")

    print("="*50)
    print("Dijkstra_NH Planning")
    print("="*50)
    print("Starting animation...")
    planner = Dijkstra_NH
    ani = animation.FuncAnimation(
        Dijkstra_NH.fig,
        animate,
        frames=Dijkstra_NH.max_iter,
        interval=50,
        repeat=False,
        blit=False
    )
    print("Saving animation...")
    ani.save("Dijkstra_NH_animation.gif", writer="pillow", fps=30)
    print("Animation completed and saved as 'Dijkstra_NH_animation.gif'")


