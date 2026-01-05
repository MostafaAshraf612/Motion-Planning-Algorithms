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
5. Check for collision.
6. Add the new node if collision-free.
7. Repeat until the goal is reached or iterations end.

**Characteristics:**
- Very fast initial solution.
- Path is often jagged and sub-optimal.
- No cost optimization.

---

### 2️⃣ RRT* — Optimal RRT

**Core Idea:** Improve RRT by **optimizing path cost** through choosing best parent and rewiring .

**Steps:**
1. Sample a random configuration.
2. Get the nearest node to the newly created random node
3. Move in the direction of the random node with a distance <= step size to create the new node 
5. Find nearby nodes to the new node within a specified search radius.
6. Choose the parent that minimizes total cost from start to new node.
7. Insert the new node.
8. **Rewire neighboring nodes** if a cheaper path exists:
   (redo step 6 in the reverse order --> check if one of the neighbours of the new node can be connected to it to get a lower total cost).
9. Repeat — path quality improves with more samples.

**Characteristics:**
- Slower than RRT.
- Produces shorter, smoother paths.
- Asymptotically optimal.

---

## 🔄 Holonomic vs Non-Holonomic Behavior

| Aspect | Holonomic | Non-Holonomic |
|------|-----------|---------------|
| State Space | (x, y) | (x, y, θ) |
| Motion | Free | Constrained |
| Collision Check | Simple | Along trajectory |
| Planning Difficulty | Lower | Higher |
| Path Smoothness | Higher | Depends on steering |

---

## 📊 Performance Comparison (with Visual Results)

### Algorithm Comparison Table

| Algorithm | System Type | Planning Speed | Path Quality | Visualization |
|---------|------------|---------------|--------------|---------------|
| **RRT** | Holonomic | ⭐⭐⭐⭐ Fast | ⭐⭐ Longer | ![https://github.com/MostafaAshraf612/Motion-Planning-Algorithms/blob/main/RRT_Algorithms/Resullts/Holonomic/rrt_animation.gif](./gifs/rrt_holonomic.gif) |
| **RRT** | Non-Holonomic | ⭐⭐⭐ Medium | ⭐⭐ Longer | ![https://github.com/MostafaAshraf612/Motion-Planning-Algorithms/blob/main/RRT_Algorithms/Resullts/Non_Holonomic/RRT_NH_animation.gif](./gifs/rrt_nonholonomic.gif) |
| **RRT\*** | Holonomic | ⭐⭐ Slower | ⭐⭐⭐⭐ Short | ![https://github.com/MostafaAshraf612/Motion-Planning-Algorithms/blob/main/RRT_Algorithms/Resullts/Holonomic/rrt_star_animation.gif](./gifs/rrt_star_holonomic.gif) |
| **RRT\*** | Non-Holonomic | ⭐ Slowest | ⭐⭐⭐⭐ Best | ![https://github.com/MostafaAshraf612/Motion-Planning-Algorithms/blob/main/RRT_Algorithms/Resullts/Non_Holonomic/RRT_star_NH_animation.gif](./gifs/rrt_star_nonholonomic.gif) |

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
