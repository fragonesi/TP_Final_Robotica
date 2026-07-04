import numpy as np
import random
from copy import deepcopy


class particle():

    def __init__(self):
        self.x = (random.random()-0.5)*2
        self.y = (random.random()-0.5)*2
        self.orientation = random.uniform(-np.pi, np.pi)
        self.weight = 1.0

    def set(self, new_x, new_y, new_orientation):
        """
        Sets the position and orientation of the particle. 
        """
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def move_odom(self, odom, noise):
        """
        Moves the particle based on odometry data with added noise.
        """
        dist = odom['t']
        delta_rot1 = odom['r1']
        delta_rot2 = odom['r2']
        a1, a2, a3, a4 = noise

        delta_rot1_hat = delta_rot1 + np.random.normal(0, a1*abs(delta_rot1) + a2*abs(dist))
        dist_hat = dist + np.random.normal(0, a3*abs(dist) + a4*(abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 + np.random.normal(0, a1*abs(delta_rot2) + a2*abs(dist))

        x_new = self.x + dist_hat * np.cos(self.orientation + delta_rot1_hat)
        y_new = self.y + dist_hat * np.sin(self.orientation + delta_rot1_hat)
        theta_new = self.orientation + delta_rot1_hat + delta_rot2_hat
        self.set(x_new, y_new, theta_new)

    def set_weight(self, weight):
        """
        Sets the weight of the particle.
        """
        self.weight = float(weight)


class RobotFunctions:

    def __init__(self, num_particles=0, offset_lidar_rad=0.0, usar_intensidades=False):
        self.particles = []
        self.best_particle = None
        self.offset_lidar_rad = offset_lidar_rad   # compensación del LIDAR del TB4 (90° en rad)
        self.usar_intensidades = usar_intensidades   # True para TB4: descarta rayos con intensidad 0

        if num_particles != 0:
            self.num_particles = num_particles
            for _ in range(self.num_particles):
                self.particles.append(particle())

    def get_weights(self):
        """
        Returns the weights of all particles as a numpy array.
        """
        if self.num_particles != 0:
            weights = np.array([p.weight for p in self.particles])
            weights /= np.sum(weights)
            return weights
        return np.array([])

    def get_particle_states(self):
        """
        Returns the states (x, y, orientation) of all particles as a numpy array.
        """
        if self.num_particles == 0:
            return np.array([])
        return np.array([[p.x, p.y, p.orientation] for p in self.particles])

    def move_particles(self, deltas):
        """
        Moves all particles based on odometry deltas with added noise.
        """
        for part in self.particles:
            part.move_odom(deltas, [13.0, 13.0, 0.1, 0.1])

    def get_selected_state(self):
        """
        Returns the estimated state (x, y, orientation) based on the weighted average of particles.
        """
        weights = self.get_weights()
        states = self.get_particle_states()
        x = np.sum(weights * states[:, 0])
        y = np.sum(weights * states[:, 1])
        orientation = np.sum(weights * states[:, 2])
        print(f"Estimated state: x={x:.2f}, y={y:.2f}, theta={orientation:.2f}")
        return [x, y, orientation]

    def update_particles(self, scan_msg, map_data, likelihood_grid):
        """
        Updates the weights of the particles based on the likelihood field and resamples them using Stochastic Universal Sampling (SUS).
        """
        # Extraer intensidades si aplica (para filtrado en TB4)
        intensities = None
        if self.usar_intensidades and scan_msg.intensities:
            intensities = np.array(scan_msg.intensities)

        weights = []
        for p in self.particles:
            last_odom = [p.x, p.y, p.orientation]
            points_map = self.scan_refererence(
                scan_msg.ranges,
                scan_msg.range_min, scan_msg.range_max,
                scan_msg.angle_min, scan_msg.angle_max,
                scan_msg.angle_increment,
                last_odom,
                intensities=intensities,
            )

            points_x = points_map[0]
            points_y = points_map[1]
            xi = ((points_x - map_data.info.origin.position.x) / map_data.info.resolution).astype(int)
            yi = ((points_y - map_data.info.origin.position.y) / map_data.info.resolution).astype(int)
            mask = (xi >= 0) & (xi < likelihood_grid.shape[1]) & (yi >= 0) & (yi < likelihood_grid.shape[0])
            xi = xi[mask]
            yi = yi[mask]

            if len(xi) > 0:
                likelihood = likelihood_grid[yi, xi] / 100.0
                log_vals = np.log(likelihood + 0.001)
                weight = np.sum(log_vals)
            else:
                weight = -np.inf

            weights.append(weight)

        weights = np.array(weights)
        weights = np.exp(weights - np.max(weights))
        weights /= np.sum(weights)

        for p, w in zip(self.particles, weights):
            p.set_weight(w)

        # Resampling SUS
        cumulative_sum = np.cumsum(weights)
        N = self.num_particles
        seed = np.random.uniform(0, 1/N)
        new_particles = []
        i = 0
        for j in range(N):
            u = seed + j * (1/N)
            while u > cumulative_sum[i]:
                i += 1
            new_particles.append(deepcopy(self.particles[i]))
        self.particles = new_particles

    def scan_refererence(self, ranges, range_min, range_max, angle_min, angle_max, angle_increment, last_odom, intensities=None):
        """
        Transforms LIDAR scan points from the robot's local frame to the global map frame based on the last odometry reading.
        """
        tx, ty, theta = last_odom

        ranges = np.array(ranges)

        # Ángulos en frame LIDAR + compensación de offset → frame robot
        angles = angle_min + np.arange(len(ranges)) * angle_increment + self.offset_lidar_rad

        # Máscara de rayos válidos por rango
        valid = (ranges > range_min) & (ranges < range_max)

        # Máscara adicional por intensidad (TB4: descartar intensidad == 0)
        if intensities is not None and len(intensities) == len(ranges):
            valid = valid & (intensities != 0.0)

        ranges = ranges[valid]
        angles = angles[valid]

        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)

        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        global_x = tx + local_x * cos_t - local_y * sin_t
        global_y = ty + local_x * sin_t + local_y * cos_t

        return np.array([global_x, global_y])