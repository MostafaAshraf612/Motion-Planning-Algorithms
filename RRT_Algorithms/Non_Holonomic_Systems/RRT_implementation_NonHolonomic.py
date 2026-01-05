# ============================
# Required libraries
# ============================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
import math
import random
import dubins


# ============================
# Node class
# ============================

class Node:
    """
    Represents a single node in the RRT* tree.
    Each node stores:
    - (x, y): position in the map
    - parent: pointer to parent node (tree structure)
    - cost: accumulated path cost from the start node
    """
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.theta = orientation
        self.parent = None
        self.cost = 0


# ============================
# RRT Planner class
# ============================

class rrt_Planner_Non_holonomic:
    """
    Implements the RRT (Rapidly-exploring Random Tree) algorithm.
    """

    def __init__(self, start: list, goal: list, turning_radius, num_of_obstacles, map_size,
                 step_size=1.0, max_iteration=1250):

        # Start and goal nodes
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], goal[2])

        # Environment parameters
        self.map_size = map_size
        self.obstacles = self.create_obstacles(num_of_obstacles)

        # RRT parameters
        self.turning_radius = turning_radius
        self.step_size = step_size
        self.max_iter = max_iteration
        self.node_list = [self.start]
        self.goal_tolerance = 2.0

        # Path + state flags
        self.path = None
        self.goal_reached = False


        # Visualization setup
        self.fig, self.ax = plt.subplots()
        self.tree_lines = []  # Store line objects for progressive drawing
        self.setup_visualization()

    # ============================
    # Obstacle generation
    # ============================

    def create_obstacles(self, obstacles_num):
        """
        Randomly generate circular obstacles.
        Each obstacle = (x, y, radius)
        """
        obstacles = []
        for _ in range(obstacles_num):
            while True:
                obs_x = random.uniform(3, self.map_size[0] - 3)
                obs_y = random.uniform(3, self.map_size[1] - 3)
                size = random.uniform(0.8, 2.0)
                
                # Check if obstacle overlaps with start or goal
                dist_to_start = np.linalg.norm([obs_x - self.start.x, obs_y - self.start.y])
                dist_to_goal = np.linalg.norm([obs_x - self.goal.x, obs_y - self.goal.y])
                
                if dist_to_start > size + 2 and dist_to_goal > size + 2:
                    obstacles.append((obs_x, obs_y, size))
                    break
        return obstacles

    # ============================
    # Visualization helpers
    # ============================

    def setup_visualization(self):
        """
        Draw start, goal, obstacles, grid, and axis limits.
        """
        self.ax.plot(self.start.x, self.start.y, 'go', markersize=12, label='Start')
        arrow_length = 1.0
        dx_start = arrow_length * np.cos(self.start.theta)
        dy_start = arrow_length * np.sin(self.start.theta)
        self.ax.arrow(self.start.x, self.start.y, dx_start, dy_start,
             head_width=0.4, head_length=0.3, fc='green', ec='green', linewidth=2)

        # Draw goal with arrow showing orientation
        self.ax.plot(self.goal.x, self.goal.y, 'r*', markersize=15, label='Goal')
        dx_goal = arrow_length * np.cos(self.goal.theta)
        dy_goal = arrow_length * np.sin(self.goal.theta)
        self.ax.arrow(self.goal.x, self.goal.y, dx_goal, dy_goal,
                    head_width=0.4, head_length=0.3, fc='red', ec='red', linewidth=2)
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

    # ============================
    # Helper: Dubins path length
    # ============================

    def dubins_path_length(self, from_node, to_node):
        """
        Calculate the length of the Dubins path between two nodes.
        """
        start_pose = (from_node.x, from_node.y, from_node.theta)
        end_pose = (to_node.x, to_node.y, to_node.theta)
        path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
        return path.path_length()

    # ============================
    # Core RRT Algorithm
    # ============================

    def plan(self):
        """
        Offline RRT planning loop (non-animated).
        """
        for _ in range(self.max_iter):

            # Sample random point
            random_node = self.get_random_node()

            # Find closest node in the tree
            nearest_node = self.get_nearest_neighbour(self.node_list, random_node)

            # Extend tree toward random node
            new_node = self.steer_dubins(nearest_node, random_node, self.turning_radius)

            # Collision check
            if self.is_collision_free(nearest_node, new_node):

                # Add to tree
                self.node_list.append(new_node) # type: ignore

            # Goal check
            if self.reached_goal(new_node):
                self.path = self.generate_path(new_node)
                self.goal_reached = True
                return

    # ============================
    # Sampling
    # ============================

    def get_random_node(self):
        """
        Random sampling with goal biasing.
        20% probability to sample the goal directly.
        """
        if random.random() > 0.1:
            x = random.uniform(0, self.map_size[0])
            y = random.uniform(0, self.map_size[1])
            theta = random.uniform(-math.pi, math.pi)
        else:
            x = self.goal.x
            y = self.goal.y
            theta = self.goal.theta
        return Node(x, y, theta)

    # ============================
    # Nearest neighbor search
    # ============================

    def get_nearest_neighbour(self, node_list, current_node):
        """
        Find the closest node in the tree to the given node.
        """
        distances = [
            np.linalg.norm([current_node.x - node.x, current_node.y - node.y])
            for node in node_list
        ]
        return node_list[np.argmin(distances)]

    # ============================
    # Steering function
    # ============================

    def steer_dubins(self, from_node, to_node, turning_radius):
        """
        Move from 'from_node' toward 'to_node' by step_size along Dubins path.
        If the distance is less than step_size, just go to the target.
        """
        try:
            start_pose = (from_node.x, from_node.y, from_node.theta)
            end_pose = (to_node.x, to_node.y, to_node.theta)
            path = dubins.shortest_path(start_pose, end_pose, turning_radius)
            
            # Get the total path length
            path_length = path.path_length()
            
            # If path is shorter than step_size, go directly to target
            if path_length <= self.step_size:
                # Check bounds
                if not (0 <= to_node.x <= self.map_size[0] and 0 <= to_node.y <= self.map_size[1]):
                    return None
                
                new_node = Node(to_node.x, to_node.y, to_node.theta)
                new_node.parent = from_node
                new_node.cost = from_node.cost + path_length  # Use actual distance
                return new_node
            
            # Otherwise, sample at step_size intervals
            configurations, _ = path.sample_many(self.step_size)
            
            # Need at least 2 points (start + one step forward)
            if len(configurations) < 2:
                return None
            
            # Take the point at step_size distance
            new_x, new_y, new_theta = configurations[1]
            
            # Check bounds
            if not (0 <= new_x <= self.map_size[0] and 0 <= new_y <= self.map_size[1]):
                return None
            
            # Create new node
            new_node = Node(new_x, new_y, new_theta)
            new_node.parent = from_node
            new_node.cost = from_node.cost + self.step_size
            
            return new_node
                
        except:
            return None

    # ============================
    # Collision checking
    # ============================

    def is_collision_free(self, start_node, end_node, resolution=0.05):
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
                if np.linalg.norm((x - ox, y - oy)) <= r + 0.3:
                    return False  # Collision detected

        return True  # Path is free of obstacles


    # ============================
    # Goal checking
    # ============================

    def reached_goal(self, node):
        """
        Check if node is close enough to the goal.
        For non-holonomic systems, we need to check if we can connect to goal.
        """
        if node is None:
            return False
        
        # Check if within tolerance distance
        distance = np.linalg.norm([node.x - self.goal.x, node.y - self.goal.y])
        if distance > self.goal_tolerance:
            return False
        
        # Try to connect to the actual goal with its orientation
        return self.is_collision_free(node, self.goal)


    # ============================
    # Path reconstruction
    # ============================

    def generate_path(self, goal):
        """
        Backtrack from goal to start using parent pointers.
        """
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = node.parent
        return path[::-1]

    # ============================
    # Drawing helpers 
    # ============================

    def draw_tree(self, node):
        """
        Draw the Dubins curve between node and its parent.
        FIXED: Returns line object for progressive display.
        """
        if node.parent:
            try:
                start_pose = (node.parent.x, node.parent.y, node.parent.theta)
                end_pose = (node.x, node.y, node.theta)
                
                path = dubins.shortest_path(start_pose, end_pose, self.turning_radius)
                configurations, _ = path.sample_many(0.1)
                
                xs = [conf[0] for conf in configurations]
                ys = [conf[1] for conf in configurations]
                
                line, = self.ax.plot(xs, ys, '-b', linewidth=0.8, alpha=0.4)
                self.tree_lines.append(line)
            except:
                pass

   
    def draw_path(self):
        """
        Draw the final path from start to goal using Dubins curves.
        """
        if not self.path:
            return

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
            


# ============================
# Animation function
# ============================

def animate(i):
    """
    Animation function that shows tree progression.
    """
    if i >= rrt.max_iter or rrt.goal_reached:
        return rrt.tree_lines

    # Sample random node
    rand_node = rrt.get_random_node()
    
    # Find nearest node
    nearest_node = rrt.get_nearest_neighbour(rrt.node_list, rand_node)
    
    # Steer toward random node
    new_node = rrt.steer_dubins(nearest_node, rand_node, rrt.turning_radius)

    # Check if steering was successful and path is collision-free
    if new_node is not None and rrt.is_collision_free(nearest_node, new_node):
        
        # Add to tree
        rrt.node_list.append(new_node)
        
        # Draw the new edge
        rrt.draw_tree(new_node)
        
        # Check if goal reached
        if rrt.reached_goal(new_node):
            # Verify the goal is within bounds
            if (0 <= rrt.goal.x <= rrt.map_size[0] and 
                0 <= rrt.goal.y <= rrt.map_size[1]):
                
                # Create final connection to goal
                final_node = Node(rrt.goal.x, rrt.goal.y, rrt.goal.theta)
                final_node.parent = new_node # type: ignore
                final_node.cost = new_node.cost + rrt.dubins_path_length(new_node, rrt.goal)
                
                rrt.node_list.append(final_node)
                rrt.path = rrt.generate_path(final_node)
                if rrt.path:
                    rrt.draw_path()
                    rrt.goal_reached = True
                    print(f"Goal reached at iteration {i}!")
                    print(f"Path length: {final_node.cost:.2f}")
                    print(f"Number of nodes: {len(rrt.node_list)}")

    return rrt.tree_lines


# ============================
# Main execution
# ============================

if __name__ == "__main__":

    start = [1, 5, 0]
    goal = [15, 15, math.pi]
    num_obstacles = 5
    map_size = [20, 20]
    turning_radius = 1.5

    print("="*50)
    print("RRT Planning for Non-Holonomic System")
    print("="*50)
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Turning radius: {turning_radius}")
    print(f"Map size: {map_size}")
    print(f"Number of obstacles: {num_obstacles}")
    print("="*50)
    
    rrt = rrt_Planner_Non_holonomic(
        start, goal, turning_radius, num_obstacles, map_size,
        step_size=0.8, max_iteration=2000
    )

    print("Starting animation...")
    ani = animation.FuncAnimation(
        rrt.fig,
        animate,
        frames=rrt.max_iter,
        interval=20,
        repeat=False,
        blit=True
    )

    print("Saving animation...")
    ani.save("rrt_NH_animation2.gif", writer="pillow", fps=30)
    print("Animation completed and saved as 'rrt_NH_animation2.gif'")
    print("="*50)