import numpy as np
import random_init
import env
from copy import deepcopy


class PSOPathPlanner:
    def __init__(self, num_control_points, pop_size, max_iter, c1, c2, w, wdamp):
        self.env = env.Env()
        self.num_control_points = num_control_points
        self.pop_size = pop_size
        self.max_iter = max_iter
        self.c1 = c1
        self.c2 = c2
        self.w = w
        self.wdamp = wdamp
        self.dim = 2 * num_control_points  # Each control point has x and y

    def cost_function(self, particle):
        real_position = self.denormalize_position(particle, 0, 1500, 0, 1200)
        points = real_position.reshape(-1, 2)
        path_points = np.vstack([self.env.start, points, self.env.goal])
        total_length = 0
        for i in range(len(path_points) - 1):
            p1, p2 = path_points[i], path_points[i + 1]
            if self.env.check_collision(p1, p2):
                return np.inf  # Invalid path
            total_length += np.linalg.norm(p2 - p1)
        return total_length

    def denormalize_position(self, position, xmin, xmax, ymin, ymax):
        x_real = position[0::2] * (xmax - xmin) + xmin
        y_real = position[1::2] * (ymax - ymin) + ymin
        return np.stack([x_real, y_real], axis=-1).flatten()

    def optimize(self):
        # Initialize particles
        pop = np.random.uniform(0, 1, (self.pop_size, self.dim))
        # pop = random_init.generate_random_position(0, 1500, 0, 1200, 3)
        # pop = random_init.initialize_pop(0, 1500, 0, 1200, self.num_control_points, self.pop_size)

        velocities = np.zeros_like(pop)
        personal_best = pop.copy()
        personal_best_cost = np.array([self.cost_function(p) for p in pop])
        global_best = personal_best[np.argmin(personal_best_cost)]
        global_best_cost = np.min(personal_best_cost)

        for it in range(self.max_iter):
            for i in range(self.pop_size):
                velocities[i] = self.w * velocities[i] \
                                + self.c1 * np.random.rand(self.dim) * (personal_best[i] - pop[i]) \
                                + self.c2 * np.random.rand(self.dim) * (global_best - pop[i])

                pop[i] += velocities[i]
                pop[i] = np.clip(pop[i], 0, 1)  # Ensure particles stay within bounds

                # Calculate the cost
                cost = self.cost_function(pop[i])
                if cost < personal_best_cost[i]:
                    personal_best[i] = pop[i].copy()
                    personal_best_cost[i] = cost

                if cost < global_best_cost:
                    global_best = pop[i].copy()
                    global_best_cost = cost

            self.w *= self.wdamp
            print(f"Iteration {it + 1}: Best Cost = {global_best_cost}")

        return global_best, global_best_cost