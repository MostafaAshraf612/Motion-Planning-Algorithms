# ============================
# Required libraries
# ============================

import numpy as np                          # Numerical operations (distances, norms)
import matplotlib.pyplot as plt             # Plotting
from matplotlib.patches import Circle       # For drawing circular obstacles
import matplotlib.animation as animation    # Animation support
import math                                 # Trigonometry
import random                               # Random sampling


# ============================
# Node class
# ============================

class Node:
    """
    Represents a single node in the RRT tree.
    Each node stores:
    - (x, y): position in the map
    - parent: pointer to parent node (tree structure)
    - cost: accumulated path length from the start node
            (stored for bookkeeping; RRT does not optimize this)
    """
    def __init__(self , x , y ):
        self.x = x
        self.y = y
        self.parent = None   # Parent node in the tree
        self.cost = 0        # Path length from the start node


# ============================
# RRT Planner class
# ============================

class RRT_Planner:
    """
    Implements the RRT (Rapidly-exploring Random Tree) algorithm.
    This version grows a tree by random sampling without rewiring
    or cost optimization.
    """

    def __init__(self , start:list , goal:list , num_of_obstacles , map_size ,
                 step_size=1.0 , max_iteration = 1000):

        # Start and goal nodes
        self.start = Node(start[0] , start[1])
        self.goal  = Node(goal[0] , goal[1])

        # Environment parameters
        self.map_size   = map_size
        self.obstacles  = self.create_obstacles(num_of_obstacles)

        # RRT parameters
        self.step_size  = step_size              # Maximum extension length per iteration
        self.max_iter   = max_iteration           # Maximum number of iterations
        self.node_list  = [self.start]            # List of nodes in the tree
        self.goal_tolerance = 0.5                 # Distance threshold to consider goal reached

        # Planning result flags
        self.path = None
        self.goal_reached = False

        # Visualization setup
        self.fig, self.ax = plt.subplots()
        self.setup_visualization()


    # ============================
    # Obstacle generation
    # ============================

    def create_obstacles(self , obstacles_num):
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

        # Draw goal with arrow showing orientation
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


    # ============================
    # Core RRT Algorithm
    # ============================

    def plan(self):
        """
        Offline RRT planning loop (tree expansion without animation).
        """
        for _ in range(self.max_iter):

            # Sample a random configuration
            random_node = self.get_random_node()

            # Find the nearest node already in the tree
            nearest_node = self.get_nearest_neighbour(self.node_list , random_node)

            # Steer from the nearest node toward the sampled node
            new_node = self.steer(nearest_node , random_node)

            # Check for collision at the new node
            if self.is_collision_free(nearest_node, new_node):
                # Add the new node to the tree
                self.node_list.append(new_node)

            # Check whether the goal region has been reached
            if self.reached_goal(new_node):
                self.path = self.generate_path(new_node)
                self.goal_reached = True
                return


    # ============================
    # Sampling
    # ============================

    def get_random_node(self):
        """
        Random configuration sampling with goal biasing.
        With 20% probability, the goal configuration is sampled directly.
        """
        if random.random() > 0.2:
            x = random.uniform(0 , self.map_size[0])
            y = random.uniform(0 , self.map_size[1])
        else:
            x = self.goal.x
            y = self.goal.y
        return Node(x , y)


    # ============================
    # Nearest neighbor search
    # ============================

    def get_nearest_neighbour(self , node_list , current_node):
        """
        Find the closest node in the tree to the given node
        using Euclidean distance.
        """
        distances = [
            np.linalg.norm([current_node.x - node.x , current_node.y - node.y])
            for node in node_list
        ]
        return node_list[np.argmin(distances)]


    # ============================
    # Steering function
    # ============================

    def steer(self , from_node , to_node):
        """
        Generate a new node by moving from 'from_node' toward 'to_node'
        by a fixed step size. This enforces incremental tree growth.
        """
        theta = math.atan2(to_node.y - from_node.y,
                           to_node.x - from_node.x)

        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)

        new_node = Node(new_x , new_y)
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node

        return new_node


    # ============================
    # Collision checking
    # ============================

    def is_collision_free(self , start_node , end_node , resolution = 0.1):
        """
        Check whether the node lies inside any obstacle.
        This is a point-based collision check.
        """
        
        distance = np.linalg.norm((start_node.x - end_node.x, start_node.y - end_node.y))
        steps = int(distance / resolution)
        for i in range(steps+1):
            x = start_node.x + (i/steps+0.0001) * (end_node.x - start_node.x)
            y = start_node.y + (i/steps+0.0001) * (end_node.y - start_node.y)
            for ox, oy, r in self.obstacles:
                if np.linalg.norm((x - ox, y - oy)) <= r:
                    return False
        return True


    # ============================
    # Goal checking
    # ============================

    def reached_goal(self , node):
        """
        Check whether the node is within a specified tolerance
        of the goal position.
        """
        return np.linalg.norm(
            (node.x - self.goal.x,
             node.y - self.goal.y)
        ) < self.goal_tolerance


    # ============================
    # Path reconstruction
    # ============================

    def generate_path(self , goal):
        """
        Reconstruct the path by backtracking from the goal node
        to the start node using parent pointers.
        """
        path = []
        node = goal
        while node is not None:
            path.append([node.x , node.y])
            node = node.parent
        return path[::-1]


    # ============================
    # Drawing helpers
    # ============================

    def draw_tree(self, node):
        """
        Draw an edge between a node and its parent to visualize
        the RRT tree expansion.
        """
        if node.parent:
            self.ax.plot([node.x, node.parent.x],
                         [node.y, node.parent.y], "-b")


    def draw_path(self):
        """
        Draw the final path found by the RRT algorithm.
        """
        if self.path:
            self.ax.plot([x[0] for x in self.path],
                         [x[1] for x in self.path],
                         '-g', label='Path')


# ============================
# Animation function
# ============================

def animate(i):
    """
    Performs one RRT iteration per animation frame.
    """
    if i < rrt.max_iter and not rrt.goal_reached:

        rand_node = rrt.get_random_node()
        nearest_node = rrt.get_nearest_neighbour(
            rrt.node_list, rand_node)

        new_node = rrt.steer(nearest_node, rand_node)

        if rrt.is_collision_free(nearest_node , new_node):
            rrt.node_list.append(new_node)
            rrt.draw_tree(new_node)

        if rrt.reached_goal(new_node):
            rrt.path = rrt.generate_path(new_node)
            rrt.draw_path()
            rrt.goal_reached = True

    return []   # Required for matplotlib animation


# ============================
# Main execution
# ============================

if __name__ == "__main__":

    start = [1, 5]
    goal = [18, 15]
    num_obstacles = 15
    map_size = [20, 20]

    rrt = RRT_Planner(start, goal, num_obstacles, map_size)

    ani = animation.FuncAnimation(
        rrt.fig,
        animate,
        frames=rrt.max_iter,
        interval=10,
        repeat=False
    )

    ani.save("rrt_animation.gif", writer="pillow", fps=30)

    print("Animation completed and saved.")
