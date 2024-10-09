import numpy as np
import env

def generate_random_position(x_min, x_max, y_min, y_max, num_points):
    position = []
    environment = env.Env
    obstacles_list = environment.obs_rectangle()
    obstacles = [{'leftdown': [x[0], x[1]], 'length': x[2], 'height': x[3]} for x in obstacles_list]
    # print(obstacles)
    while len(position) < 2 * num_points:
        # Generate random x, y within the allowed range
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        if not is_in_obstacle([x, y], obstacles):
            position.extend([x, y])  # Append x, y to the list
    return np.array(position)


def is_in_obstacle(point, obstacles):
    x, y = point
    for obs in obstacles:
        left_x, bottom_y = obs['leftdown']
        right_x = left_x + obs['length']
        top_y = bottom_y + obs['height']
        if left_x <= x <= right_x and bottom_y <= y <= top_y:
            return True
    return False

def initialize_pop(x_min, x_max, y_min, y_max, num_points, pop_size):
    pop = []
    while len(pop) < pop_size:
        individual = generate_random_position(x_min, x_max, y_min, y_max, num_points)
        pop.append(individual)

    return pop

# print(initialize_pop(0, 1500, 0, 1200, 3, 5))
