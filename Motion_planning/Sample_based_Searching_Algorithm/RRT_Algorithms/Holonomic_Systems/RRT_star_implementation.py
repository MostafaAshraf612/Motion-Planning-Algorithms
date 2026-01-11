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
    Represents a single node in the RRT* tree.
    Each node stores:
    - (x, y): position in the map
    - parent: pointer to parent node (tree structure)
    - cost: accumulated path cost from the start node
    """
    def __init__(self , x , y ):
        self.x = x
        self.y = y
        self.parent = None   # Parent in the tree
        self.cost = 0        # Cost-to-come


# ============================
# RRT* Planner class
# ============================

class RRT_Star_Planner:
    """
    Implements the RRT* (Rapidly-exploring Random Tree Star) algorithm.
    RRT* improves RRT by:
    - choosing the lowest-cost parent
    - rewiring nearby nodes to reduce total path cost
    """

    def __init__(self , start:list , goal:list , num_of_obstacles , map_size ,
                 step_size=1.0 , max_iteration = 1000):

        # Start and goal nodes
        self.start = Node(start[0] , start[1])
        self.goal  = Node(goal[0] , goal[1])

        # Environment parameters
        self.map_size   = map_size
        self.obstacles  = self.create_obstacles(num_of_obstacles)

        # RRT* parameters
        self.step_size  = step_size              # Maximum extension length
        self.max_iter   = max_iteration           # Maximum iterations
        self.node_list  = [self.start]            # Tree nodes
        self.goal_tolerance = 0.5                 # Distance threshold to reach goal
        self.search_radius  = 2.0                 # Radius for neighbor search

        # Path + state flags
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
    # Core RRT* Algorithm
    # ============================

    def plan(self):
        """
        Offline RRT* planning loop (non-animated).
        """
        for _ in range(self.max_iter):

            # Sample random point
            random_node = self.get_random_node()

            # Find closest node in the tree
            nearest_node = self.get_nearest_neighbour(self.node_list , random_node)

            # Extend tree toward random node
            new_node = self.steer(nearest_node , random_node)

            # Collision check
            if self.is_collision_free(nearest_node ,new_node):

                # Find nearby nodes
                neighbours = self.get_neighbours(new_node)

                # Choose best parent
                new_node = self.choose_parent(neighbours , nearest_node , new_node)

                # Add to tree
                self.node_list.append(new_node)

                # Rewire nearby nodes
                self.rewire(new_node , neighbours)

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
        Find the closest node in the tree to the given node.
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
        Move from 'from_node' toward 'to_node' by step_size.
        This enforces incremental tree growth.
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
            x = start_node.x + i/steps * (end_node.x - start_node.x)
            y = start_node.y + i/steps * (end_node.y - start_node.y)
            for ox, oy, r in self.obstacles:
                if np.linalg.norm((x - ox, y - oy)) <= r:
                    return False
        return True


    # ============================
    # Neighbor search
    # ============================

    def get_neighbours(self , current_node):
        """
        Find all nodes within the search radius.
        """
        return [
            node for node in self.node_list
            if np.linalg.norm((current_node.x - node.x,
                               current_node.y - node.y)) < self.search_radius
        ]


    # ============================
    # Parent selection (RRT* improvement)
    # ============================

    def choose_parent(self , neighbours , nearest_node , current_node):
        """
        Choose the neighbor that gives minimum cost-to-come.
        """
        min_cost = nearest_node.cost + np.linalg.norm(
            (nearest_node.x - current_node.x,
             nearest_node.y - current_node.y)
        )
        best_node = nearest_node

        for node in neighbours:
            cost = node.cost + np.linalg.norm(
                (node.x - current_node.x,
                 node.y - current_node.y)
            )
            if cost < min_cost and self.is_collision_free(node , current_node):
                min_cost = cost
                best_node = node

        current_node.parent = best_node
        current_node.cost   = min_cost

        return current_node


    # ============================
    # Rewiring (key RRT* step)
    # ============================

    def rewire(self , node , neighbours):
        """
        Try to improve nearby nodes by re-routing them through 'node'.
        """
        for neighbour in neighbours:
            cost = node.cost + np.linalg.norm(
                (node.x - neighbour.x,
                 node.y - neighbour.y)
            )
            if cost < neighbour.cost and self.is_collision_free(node , neighbour):
                neighbour.parent = node
                neighbour.cost   = cost


    # ============================
    # Goal checking
    # ============================

    def reached_goal(self , node):
        """
        Check if node is close enough to the goal.
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
        Backtrack from goal to start using parent pointers.
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
        Draw an edge between a node and its parent.
        """
        if node.parent:
            self.ax.plot([node.x, node.parent.x],
                         [node.y, node.parent.y], "-b")


    def draw_path(self):
        """
        Draw the final optimal path.
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
    Performs one RRT* iteration per animation frame.
    """
    if i < rrt_star.max_iter and not rrt_star.goal_reached:

        rand_node = rrt_star.get_random_node()
        nearest_node = rrt_star.get_nearest_neighbour(
            rrt_star.node_list, rand_node)

        new_node = rrt_star.steer(nearest_node, rand_node)

        if rrt_star.is_collision_free(nearest_node ,new_node):
            neighbors = rrt_star.get_neighbours(new_node)
            new_node = rrt_star.choose_parent(
                neighbors, nearest_node, new_node)

            rrt_star.node_list.append(new_node)
            rrt_star.rewire(new_node, neighbors)
            rrt_star.draw_tree(new_node)

        if rrt_star.reached_goal(new_node):
            rrt_star.path = rrt_star.generate_path(new_node)
            rrt_star.draw_path()
            rrt_star.goal_reached = True

    return []   # Required for matplotlib animation


# ============================
# Main execution
# ============================

if __name__ == "__main__":

    start = [1, 5]
    goal = [18, 15]
    num_obstacles = 15
    map_size = [20, 20]
    
    rrt_star = RRT_Star_Planner(start, goal, num_obstacles, map_size)

    ani = animation.FuncAnimation(
        rrt_star.fig,
        animate,
        frames=rrt_star.max_iter,
        interval=10,
        repeat=False
    )

    ani.save("rrt_star_animation.gif", writer="pillow", fps=30)

    print("Animation completed and saved.")
