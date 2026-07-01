import numpy as np
import random
from copy import deepcopy

#MI PARTICLE FILTER

class particle():

    def __init__(self):

        self.x = (random.random()-0.5)*2  # initial x position
        self.y = (random.random()-0.5)*2 # initial y position
        self.orientation = random.uniform(-np.pi,np.pi) # initial orientation
        self.weight = 1.0

    def set(self, new_x, new_y, new_orientation):
        '''set: sets a robot coordinate, including x, y and orientation'''

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def move_odom(self, odom, noise): 
        '''
        move_odom: Takes in Odometry data and moves the robot based on the odometry data
        Devuelve una particula (del robot) actualizada
        '''      
        dist  = odom['t']       
        delta_rot1  = odom['r1']
        delta_rot2 = odom['r2']
        a1, a2, a3, a4 = noise

        delta_rot1_hat = delta_rot1 + np.random.normal(0, a1 * abs(delta_rot1) + a2 * abs(dist))
        dist_hat = dist + np.random.normal(0, a3 * abs(dist) + a4 * (abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 + np.random.normal(0, a1 * abs(delta_rot2) + a2 * abs(dist))

        x_new = self.x + dist_hat * np.cos(self.orientation + delta_rot1_hat)
        y_new = self.y + dist_hat * np.sin(self.orientation + delta_rot1_hat)
        theta_new = self.orientation + delta_rot1_hat + delta_rot2_hat

        self.set(x_new, y_new,theta_new )

    def set_weight(self, weight):
        '''set_weights: sets the weight of the particles'''
        #noise parameters
        self.weight  = float(weight)


class RobotFunctions:

    def __init__(self, num_particles=0):
        self.particles = []
        self.best_particle = None
        if num_particles != 0:
            self.num_particles = num_particles
            self.particles = []

            for _ in range(self.num_particles):
                self.particles.append(particle())
    
    def get_weights(self,):
        if self.num_particles != 0:
            weights = np.array([p.weight for p in self.particles])
            weights /= np.sum(weights)  # Normalize weights
            return weights

        else:
            return np.array([])

    def get_particle_states(self,):
        if self.num_particles == 0:
            return np.array([])

        samples = np.array([[p.x, p.y, p.orientation] for p in self.particles])
        return samples    

    def move_particles(self, deltas):
        for part in self.particles:
            part.move_odom(deltas, [0.1, 0.1, 0.001, 0.001])

    def get_selected_state(self,):
        #hago el promedio ponderado de las particulas
        weights = self.get_weights()
        states = self.get_particle_states()
        x = np.sum(weights * states[:, 0])
        y = np.sum(weights * states[:, 1])
        orientation = np.sum(weights * states[:, 2])
        print(f"Estimated state: x={x:.2f}, y={y:.2f}, theta={orientation:.2f}")

        return [x, y, orientation]

    def update_particles(self, data, map_data, likelihood_grid):
        #1. Para cada partícula, uso scan_refererence para obtener los puntos del scan en coordenadas globales
        weights = []
        for p in self.particles:
            last_odom = [p.x, p.y, p.orientation]
            points_map = self.scan_refererence(data.ranges, data.range_min, data.range_max, data.angle_min, data.angle_max, data.angle_increment, last_odom)

            #2. Calculo el peso de cada partícula en base a la distancia entre los puntos del scan y el mapa de likelihood (grid)
            points_x = points_map[0]
            points_y = points_map[1]
            xi = ((points_x - map_data.info.origin.position.x) / map_data.info.resolution).astype(int)
            yi = ((points_y - map_data.info.origin.position.y) / map_data.info.resolution).astype(int)
            mask = (xi >= 0) & (xi < likelihood_grid.shape[1]) & (yi >= 0) & (yi < likelihood_grid.shape[0])
            xi = xi[mask]
            yi = yi[mask]

            if len(xi) > 0:
                likelihood = likelihood_grid[yi, xi] / 100.0
                log_vals = np.log(likelihood + 1e-9)
                weight = np.sum(log_vals)

            else:
                weight = -np.inf

            weights.append(weight)

        # ***
        #3. Normalizo los pesos (común a las dos ramas)
        weights = np.array(weights)
        weights = np.exp(weights - np.max(weights))  # Evita problemas numéricos
        weights /= np.sum(weights)

        # Guardo los pesos en las partículas y registro la mejor (común a ambas ramas)
        for p, w in zip(self.particles, weights):
            p.set_weight(w)

        best_idx = int(np.argmax(weights))
        bp = self.particles[best_idx]
        self.best_particle = (bp.x, bp.y, bp.orientation)

        # Decido si resamplear según Neff
        neff = 1.0 / np.sum(weights**2)

        if neff < self.num_particles * 0.7:
            # --- Resampleo con SUS + jitter ---
            cumulative_sum = np.cumsum(weights)
            N = self.num_particles
            seed = np.random.uniform(0, 1/N)
            new_particles = []
            i = 0
            for j in range(N):
                u = seed + j * (1/N)
                while u > cumulative_sum[i]:
                    i += 1
                new_p = deepcopy(self.particles[i])
                new_p.x += np.random.normal(0, 0.01)
                new_p.y += np.random.normal(0, 0.01)
                new_p.orientation += np.random.normal(0, 0.02)
                new_particles.append(new_p)

            self.particles = new_particles
            # tras resamplear, todas las partículas pesan igual
            for p in self.particles:
                p.set_weight(1.0 / self.num_particles)
        # else: no resampleo. Los pesos ya quedaron guardados arriba.

            

    def scan_refererence(self, ranges, range_min, range_max, angle_min, angle_max, angle_increment, last_odom):
        tx, ty, theta = last_odom
        ranges = np.array(ranges)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        valid = (ranges > range_min) & (ranges < range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)
        t = theta #+ np.pi
        cos_t = np.cos(t)
        sin_t = np.sin(t)
        global_x = tx + local_x * cos_t - local_y * sin_t
        global_y = ty + local_x * sin_t + local_y * cos_t
        return np.array([global_x, global_y])