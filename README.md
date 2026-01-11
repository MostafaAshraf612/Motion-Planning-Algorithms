# Motion Planning Algorithms

> Implementation, visualization, and performance comparison of sampling-based motion planning algorithms for **holonomic** and **non-holonomic** robotic systems.

---

## 🚀 Overview

This repository implements and compares **two motion planning algorithms**, each in **two variants**:

- **Holonomic version** (free motion in x–y space)
- **Non-holonomic version** (kinematic constraints, e.g. car-like robots)

The main focus of this project is to **analyze performance trade-offs** between the algorithms in terms of:
- Planning time
- Path quality (length & smoothness)
- Suitability for constrained motion systems

---

## 🧠 Implemented Algorithms

- **Algorithm 1:** Rapidly-Exploring Random Tree (**RRT**)
- **Algorithm 2:** Optimal Rapidly-Exploring Random Tree (**RRT\***)

Each algorithm is implemented for:
- ✅ **Holonomic systems**
- 🚗 **Non-holonomic systems** (steering & motion constraints)

---

## ⚙️ System Types

### 🟢 Holonomic Systems
- Robot can move freely in any direction.
- State: **(x, y)**
- Motion primitives: straight-line interpolation.
- Simpler collision checking.

### 🚗 Non-Holonomic Systems
- Motion is constrained by kinematics (e.g., car-like robot).
- State: **(x, y, θ)**
- Motion primitives follow feasible trajectories.
- Collision checking is performed along curved paths.

---

## 🔍 Algorithm Walkthrough

### 1️⃣ RRT — Rapidly-Exploring Random Tree

**Core Idea:** Quickly explore the configuration space to find *any* feasible path.

**Steps:**
1. Initialize the tree with the start node.
2. Randomly sample a configuration in the space.
3. Find the nearest node in the tree.
4. Steer toward the sampled point:
   - Straight-line motion (holonomic)
   - Feasible kinematic motion (non-holonomic)
5. Check for collisions along the path.
6. Add the new node to the tree if it is collision-free.
7. Repeat steps 2–6 until the goal is reached or the maximum number of iterations is exceeded.

**Characteristics:**
- Fast initial solution.
- Path may be jagged or sub-optimal.
- Does not optimize path cost.

---

### 2️⃣ RRT* — Optimal Rapidly-Exploring Random Tree

**Core Idea:** Improve RRT by **optimizing path cost** through selecting the best parent and rewiring neighbors.

**Steps:**
1. Randomly sample a configuration in the space.
2. Find the nearest node in the tree to the sampled point.
3. Move in the direction of the sampled point, with a step size limit, to create a new node.
4. Identify all nearby nodes within a specified search radius of the new node.
5. Choose the parent node that minimizes the total cost from the start to the new node.
6. Insert the new node into the tree.
7. **Rewire neighboring nodes** if connecting through the new node reduces their total cost:
   - Re-evaluate nearby nodes
   - Check if connecting them via the new node lowers their cost
   - Update their parent accordingly
8. Repeat steps 1–7 until the goal is reached or the maximum number of iterations is exceeded.

**Characteristics:**
- Slower per iteration than RRT.
- Produces shorter and smoother paths.
- Asymptotically optimal as iterations increase.

---

### 🔄 Holonomic vs Non-Holonomic Behavior

| Aspect | Holonomic | Non-Holonomic |
|--------|-----------|---------------|
| State Space | (x, y) | (x, y, θ) |
| Motion | Free | Constrained by kinematics |
| Collision Check | Simple | Along feasible trajectory |
| Planning Difficulty | Lower | Higher |
| Path Smoothness | High | Depends on steering |

---

## 📊 Performance Comparison (with Visual Results)

### Algorithm Comparison Table

| Algorithm | System Type     | Planning Speed     | Path Quality       | Visualization |
|-----------|----------------|------------------|------------------|---------------|
| RRT       | Holonomic       | **⭐⭐⭐⭐**<br>Fast | **⭐⭐**<br>Longer | ![RRT Holonomic](Motion_planning/Sample_based_Searching_Algorithm/RRT_Algorithms/Results/Holonomic/rrt_animation.gif) |
| RRT       | Non-Holonomic   | **⭐⭐⭐**<br>Medium | **⭐⭐**<br>Longer | ![RRT Non-Holonomic](Motion_planning/Sample_based_Searching_Algorithm/RRT_Algorithms/Results/Non_Holonomic/RRT_NH_animation.gif) |
| RRT*      | Holonomic       | **⭐⭐**<br>Slower | **⭐⭐⭐⭐**<br>Short | ![RRT* Holonomic](Motion_planning/Sample_based_Searching_Algorithm/RRT_Algorithms/Results/Holonomic/rrt_star_animation.gif) |
| RRT*      | Non-Holonomic   | **⭐**<br>Slowest | **⭐⭐⭐⭐**<br>Best  | ![RRT* Non-Holonomic](Motion_planning/Sample_based_Searching_Algorithm/RRT_Algorithms/Results/Non_Holonomic/RRT_star_NH_animation.gif) |

> 📌 GIFs show tree expansion, collision-free exploration, and final paths.

---

## 📈 Key Performance Insights

- **RRT vs RRT\***
  - RRT finds solutions faster.
  - RRT* consistently produces shorter and smoother paths.
- **Holonomic vs Non-Holonomic**
  - Non-holonomic planning is computationally heavier.
  - Constraints significantly affect exploration and convergence.
- **Best Use Cases**
  - RRT → fast feasibility checks.
  - RRT* → high-quality navigation and optimal paths.

---
