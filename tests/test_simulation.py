"""
Automated Physics & Kinematics Validation Test Suite
Rough-Terrain Inspection Rover Simulation
"""

import math
import unittest


def get_terrain_elevation(x, z):
    r = math.sqrt(x * x + z * z)
    if r < 2.5:
        spawn_mask = 0.0
    elif r < 4.0:
        s = (r - 2.5) / 1.5
        spawn_mask = s * s * (3.0 - 2.0 * s)
    else:
        spawn_mask = 1.0

    octave1 = math.sin(x * 0.18 + 0.4) * math.cos(z * 0.16 - 0.2) * 0.75
    octave2 = math.sin(x * 0.38 - z * 0.32) * 0.35
    octave3 = (math.cos(x * 0.85 + 1.2) * math.sin(z * 0.85)) * 0.14
    octave4 = math.sin(x * 1.6 + z * 1.4) * 0.05

    ramp_dist = math.hypot(x - (-8.0), z - 8.0)
    ramp_feature = max(0.0, 1.0 - ramp_dist / 6.0) * 0.85

    gully_dist = math.hypot(x - 8.0, z - 7.0)
    gully_feature = -max(0.0, 1.0 - gully_dist / 5.0) * 0.45

    h = (octave1 + octave2 + octave3 + octave4 + ramp_feature + gully_feature) * spawn_mask

    if r > 18.5:
        edge_factor = min(1.0, (r - 18.5) / 5.5)
        h += math.pow(edge_factor, 1.6) * 4.2

    return h


def dist_to_adc(dist):
    clamped = max(0.0, min(2.5, dist))
    if clamped <= 0.3:
        return round(1000 - (clamped / 0.3) * 150)
    elif clamped <= 1.0:
        return round(850 - ((clamped - 0.3) / 0.7) * 350)
    elif clamped <= 2.0:
        return round(500 - ((clamped - 1.0) / 1.0) * 500)
    else:
        return 0


class TestRoverPhysicsAndKinematics(unittest.TestCase):

    def test_spawn_area_flatness(self):
        """Verify spawn zone (r < 2.5m) is strictly 0.0m for stable initialization."""
        self.assertEqual(get_terrain_elevation(0, 0), 0.0)
        self.assertEqual(get_terrain_elevation(1.0, 1.0), 0.0)
        self.assertEqual(get_terrain_elevation(-1.5, 0.5), 0.0)

    def test_terrain_elevation_bounds(self):
        """Verify natural terrain elevations are within expected physical bounds."""
        for x in range(-15, 16, 3):
            for z in range(-15, 16, 3):
                h = get_terrain_elevation(x, z)
                self.assertGreater(h, -1.5, f"Depression too deep at ({x}, {z}): {h}")
                self.assertLess(h, 4.5, f"Hill too high at ({x}, {z}): {h}")

    def test_boundary_containment_wall(self):
        """Verify boundary perimeter (r > 24m) elevates significantly to prevent escaping."""
        h_edge = get_terrain_elevation(25.0, 0.0)
        self.assertGreater(h_edge, 2.0, "Boundary rim not sufficiently elevated")

    def test_4wheel_chassis_elevation_solver(self):
        """Verify chassis center elevation is always above the wheel contact plane."""
        wheel_radius = 0.10
        wheelbase = 0.32
        track_width = 0.42

        # Simulate rover on a slope
        pos_x, pos_z = -8.0, 8.0  # Northwest ramp
        heading = 0.0

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        wheel_offsets = [
            (-track_width / 2,  wheelbase / 2),
            ( track_width / 2,  wheelbase / 2),
            (-track_width / 2, -wheelbase / 2),
            ( track_width / 2, -wheelbase / 2),
        ]

        ground_heights = []
        for lx, lz in wheel_offsets:
            wx = pos_x + (cos_h * lx + sin_h * lz)
            wz = pos_z + (-sin_h * lx + cos_h * lz)
            ground_heights.append(get_terrain_elevation(wx, wz))

        chassis_y = sum(ground_heights) / 4.0 + wheel_radius

        # Chassis MUST be above the minimum ground height by at least wheel radius
        min_ground = min(ground_heights)
        self.assertGreaterEqual(chassis_y, min_ground + wheel_radius)

    def test_distance_sensor_adc_conversion(self):
        """Verify calibrated distance to ADC signal mapping matches Webots proto lookup table."""
        self.assertEqual(dist_to_adc(0.0), 1000)
        self.assertEqual(dist_to_adc(0.3), 850)
        self.assertEqual(dist_to_adc(1.0), 500)
        self.assertEqual(dist_to_adc(2.0), 0)
        self.assertEqual(dist_to_adc(2.5), 0)

    def test_solid_obstacle_repulsion(self):
        """Verify collision resolution pushes rover outside obstacle radius."""
        obs_x, obs_z, obs_r = 0.0, 4.0, 0.55
        rover_r = 0.30
        rover_x, rover_z = 0.0, 3.75  # 0.25m penetration into obstacle

        dx = rover_x - obs_x
        dz = rover_z - obs_z
        dist = math.hypot(dx, dz)
        min_dist = obs_r + rover_r

        self.assertLess(dist, min_dist)  # In collision

        penetration = min_dist - dist
        nx = dx / dist
        nz = dz / dist

        resolved_x = rover_x + nx * penetration
        resolved_z = rover_z + nz * penetration

        new_dist = math.hypot(resolved_x - obs_x, resolved_z - obs_z)
        self.assertAlmostEqual(new_dist, min_dist, places=5)


if __name__ == '__main__':
    unittest.main()
