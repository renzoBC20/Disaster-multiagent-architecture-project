"""
Camera - RGB pinhole camera with semantic rendering.

Responsibilities:
- Ray-cast through pinhole model to render RGB image
- Height-aware occlusion (z-test for nearest surface)
- Fixed 5-color semantic palette
- Publish image_raw (rgb8) and camera_info
"""

import numpy as np
from typing import Tuple
import time


# Fixed RGB color palette for semantic classes (0-255 range)
SEMANTIC_COLORS = {
    0: (150, 120, 90),   # GROUND: brownish
    1: (255, 0, 0),      # OBSTACLE: red (critical victims - circles)
    2: (255, 103, 0),    # HAZARD: orange #FF6700 (wounded victims - circles)
    3: (0, 100, 0),      # TARGET: dark green (safe victims - circles)
    4: (200, 170, 60),   # WATER: yellow (trees - squares)
    5: (139, 69, 19),    # BUILDING: brown (buildings - squares)
    6: (255, 0, 255),    # DEBRIS: magenta (debris - squares)
    7: (64, 64, 64),     # VEHICLE: dark gray (vehicles - squares)
    255: (135, 206, 235) # SKY: light blue
}


class PinholeCamera:
    """RGB pinhole camera with semantic rendering."""

    def __init__(self, width: int = 128, height: int = 128,
                 fov_deg: float = 90.0, rate_hz: float = 2.0):
        """
        Initialize pinhole camera.

        Args:
            width: Image width in pixels
            height: Image height in pixels
            fov_deg: Horizontal field of view in degrees
            rate_hz: Camera update rate in Hz
        """
        self.width = width
        self.height = height
        self.fov_deg = fov_deg
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.time_since_update = 0.0

        # Compute intrinsic matrix
        self.fx = width / (2.0 * np.tan(np.radians(fov_deg) / 2.0))
        self.fy = self.fx  # Square pixels
        self.cx = width / 2.0
        self.cy = height / 2.0

    def update(self, dt: float) -> bool:
        """
        Check if camera should capture image.

        Args:
            dt: Timestep in seconds

        Returns:
            True if camera should render, False otherwise
        """
        self.time_since_update += dt

        if self.time_since_update >= self.dt:
            self.time_since_update = 0.0
            return True

        return False

    def _add_black_borders(self, image: np.ndarray) -> np.ndarray:
        """
        Add black borders (contours) to all non-sky shapes in the image.
        
        Uses vectorized NumPy operations to detect edges efficiently.
        
        Args:
            image: RGB image array (height, width, 3)
            
        Returns:
            Image with black borders added
        """
        height, width = image.shape[:2]
        result = image.copy()
        black = np.array([0, 0, 0], dtype=np.uint8)
        
        # Sky color reference
        sky_color = np.array([135, 206, 235], dtype=np.uint8)
        
        # Check all 4 neighbors (up, down, left, right) using vectorized operations
        # Top neighbors
        top = image[:-2, 1:-1, :]
        # Bottom neighbors
        bottom = image[2:, 1:-1, :]
        # Left neighbors
        left = image[1:-1, :-2, :]
        # Right neighbors
        right = image[1:-1, 2:, :]
        # Current pixels
        current = image[1:-1, 1:-1, :]
        
        # Calculate color differences (L2 norm)
        diff_top = np.linalg.norm(current.astype(float) - top.astype(float), axis=2)
        diff_bottom = np.linalg.norm(current.astype(float) - bottom.astype(float), axis=2)
        diff_left = np.linalg.norm(current.astype(float) - left.astype(float), axis=2)
        diff_right = np.linalg.norm(current.astype(float) - right.astype(float), axis=2)
        
        # Check if neighbor is significantly different (threshold > 30)
        edge_top = diff_top > 30
        edge_bottom = diff_bottom > 30
        edge_left = diff_left > 30
        edge_right = diff_right > 30
        
        # Check if neighbors are not sky color using L2 norm distance
        dist_top = np.linalg.norm(top.astype(float) - sky_color.astype(float), axis=2)
        dist_bottom = np.linalg.norm(bottom.astype(float) - sky_color.astype(float), axis=2)
        dist_left = np.linalg.norm(left.astype(float) - sky_color.astype(float), axis=2)
        dist_right = np.linalg.norm(right.astype(float) - sky_color.astype(float), axis=2)
        
        # Neighbor is sky if distance is small (within tolerance)
        sky_top = dist_top < 8.0
        sky_bottom = dist_bottom < 8.0
        sky_left = dist_left < 8.0
        sky_right = dist_right < 8.0
        
        # Edge detected if neighbor is different AND not sky
        edge_mask = (edge_top & ~sky_top) | (edge_bottom & ~sky_bottom) | \
                    (edge_left & ~sky_left) | (edge_right & ~sky_right)
        
        # Draw black borders
        result[1:-1, 1:-1, :][edge_mask] = black
        
        return result

    def render(self, world, drone_x: float, drone_y: float, drone_z: float,
               drone_yaw: float, drone_pitch: float, rover_x: float = None, 
               rover_y: float = None) -> np.ndarray:
        """
        Render RGB image from drone camera.

        Uses pinhole camera model with ray-casting to render semantic labels
        as RGB colors. Implements height-aware occlusion (z-test).
        Optimized with vectorized NumPy operations.

        Args:
            world: World object with height_map and semantic_map
            drone_x, drone_y, drone_z: Drone position
            drone_yaw, drone_pitch: Drone orientation

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        start_time = time.time()

        # Camera always points straight down (nadir view)
        # Since camera frame Z already points down (z_cam = -1), no additional pitch needed
        camera_pitch = drone_pitch  # Just use drone pitch as-is

        # Precompute rotation matrices
        cos_yaw = np.cos(drone_yaw)
        sin_yaw = np.sin(drone_yaw)
        cos_pitch = np.cos(camera_pitch)
        sin_pitch = np.sin(camera_pitch)

        # Create pixel coordinate grids (vectorized)
        u_coords, v_coords = np.meshgrid(np.arange(self.width), np.arange(self.height))

        # Compute ray directions in camera frame for ALL pixels at once
        # Camera coordinate system: X=right, Y=down, Z=forward
        # For a downward-looking camera, Z should point down (negative in world Z)
        x_cam = (u_coords - self.cx) / self.fx
        y_cam = (v_coords - self.cy) / self.fy
        z_cam = -np.ones_like(x_cam)  # Negative Z = camera points down

        # Normalize ray directions
        ray_length = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        x_cam /= ray_length
        y_cam /= ray_length
        z_cam /= ray_length

        # Rotate rays to world frame (vectorized)
        # First apply pitch (rotation around x-axis)
        # Standard rotation matrix: Rx(θ) where positive pitch is nose-up
        # Note: Camera already points down (z_cam < 0), pitch adjusts from there
        x_pitch = x_cam
        y_pitch = y_cam * cos_pitch - z_cam * sin_pitch
        z_pitch = y_cam * sin_pitch + z_cam * cos_pitch

        # Then apply yaw (rotation around z-axis)
        x_world = x_pitch * cos_yaw - y_pitch * sin_yaw
        y_world = x_pitch * sin_yaw + y_pitch * cos_yaw
        z_world = z_pitch

        # Cast rays for all pixels (vectorized)
        image = self._cast_rays_vectorized(
            world,
            drone_x, drone_y, drone_z,
            x_world, y_world, z_world
        )
        
        # Draw rover if provided
        if rover_x is not None and rover_y is not None:
            # Project rover position to image coordinates
            # For downward-pointing camera, the projection is simpler
            # World to camera coordinates (relative to drone)
            dx = rover_x - drone_x
            dy = rover_y - drone_y
            dz = drone_z  # Altitude of drone (positive Z is up in world, negative in camera)
            
            # Transform to camera frame using only yaw rotation
            # Camera points down, so no pitch rotation needed
            x_cam = dx * cos_yaw + dy * sin_yaw
            y_cam = -dx * sin_yaw + dy * cos_yaw
            z_cam = -dz  # Camera Z points down (negative world Z)
            
            # Project to image plane using pinhole model
            # For downward camera: u = fx * (x / -z) + cx, v = fy * (y / -z) + cy
            if dz > 0:  # Only draw if rover is below drone
                u_rover = self.fx * (x_cam / dz) + self.cx
                v_rover = self.fy * (y_cam / dz) + self.cy
                
                # Check if rover is within image bounds
                if 0 <= u_rover < self.width and 0 <= v_rover < self.height:
                    # Draw rover as a gray square
                    # Approximate size: rover footprint ~1m, scale by altitude
                    rover_size_pixels = int(self.fx * (1.0 / dz))  # Size in pixels
                    rover_size_pixels = max(10, min(rover_size_pixels, 40))  # Clamp size to make it more visible
                    
                    # Define square corners
                    u0 = int(u_rover - rover_size_pixels / 2)
                    u1 = int(u_rover + rover_size_pixels / 2)
                    v0 = int(v_rover - rover_size_pixels / 2)
                    v1 = int(v_rover + rover_size_pixels / 2)
                    
                    # Draw blue square (RGB - 0, 0, 255 is blue) with black border
                    color_blue = np.array([0, 0, 255], dtype=np.uint8)
                    color_black = np.array([0, 0, 0], dtype=np.uint8)
                    
                    # Fill the entire square
                    image[max(0, v0):min(self.height, v1), max(0, u0):min(self.width, u1)] = color_blue
                    
                    # Draw black border (1 pixel)
                    border_width = 1
                    # Top and bottom borders
                    image[max(0, v0):min(self.height, v0 + border_width), max(0, u0):min(self.width, u1)] = color_black
                    image[max(0, v1 - border_width):min(self.height, v1), max(0, u0):min(self.width, u1)] = color_black
                    # Left and right borders
                    image[max(0, v0):min(self.height, v1), max(0, u0):min(self.width, u0 + border_width)] = color_black
                    image[max(0, v0):min(self.height, v1), max(0, u1 - border_width):min(self.width, u1)] = color_black

        # Add black borders to all shapes (post-processing)
        image = self._add_black_borders(image)

        elapsed = time.time() - start_time
        # Log performance occasionally (every ~5 seconds at 2 Hz)
        if not hasattr(self, '_render_count'):
            self._render_count = 0
            self._total_time = 0.0
        self._render_count += 1
        self._total_time += elapsed
        if self._render_count % 10 == 0:
            avg_time = self._total_time / 10
            fps = 1.0 / avg_time if avg_time > 0 else 0
            print(f"Camera render: {avg_time*1000:.1f}ms avg ({fps:.1f} FPS max) - {self.width}x{self.height}")
            self._total_time = 0.0

        return image

    def _cast_ray(self, world, origin_x: float, origin_y: float, origin_z: float,
                  dir_x: float, dir_y: float, dir_z: float) -> Tuple[int, int, int]:
        """
        Cast a ray and return the RGB color of the first intersection.

        Uses height-aware occlusion: checks ray-ground intersection and
        compares with terrain height.

        Args:
            world: World object
            origin_x, origin_y, origin_z: Ray origin (camera position)
            dir_x, dir_y, dir_z: Ray direction (normalized)

        Returns:
            RGB color tuple (r, g, b)
        """
        # Maximum ray length
        max_distance = 100.0
        step_size = world.resolution

        # Cast ray by stepping along direction
        for distance in np.arange(0.0, max_distance, step_size):
            # Current ray position
            ray_x = origin_x + distance * dir_x
            ray_y = origin_y + distance * dir_y
            ray_z = origin_z + distance * dir_z

            # Check if ray has hit the ground/terrain
            terrain_height = world.get_height(ray_x, ray_y)

            if ray_z <= terrain_height:
                # Ray intersected terrain, get semantic class
                semantic = world.get_semantic(ray_x, ray_y)
                return SEMANTIC_COLORS.get(int(semantic), SEMANTIC_COLORS[0])

        # No intersection - return sky color (light blue)
        return (135, 206, 235)  # Sky blue

    def _cast_rays_vectorized(self, world, origin_x: float, origin_y: float, origin_z: float,
                               dir_x: np.ndarray, dir_y: np.ndarray, dir_z: np.ndarray) -> np.ndarray:
        """
        Cast rays for all pixels at once using OPTIMIZED vectorized operations.

        For downward-looking camera over flat terrain, we calculate exact ground
        intersection instead of stepping along rays. This is ~100× faster!

        Args:
            world: World object
            origin_x, origin_y, origin_z: Ray origin (camera position)
            dir_x, dir_y, dir_z: Ray directions for all pixels (height x width arrays)

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        height, width = dir_x.shape

        # Sky color for all pixels initially
        sky_color = np.array([135, 206, 235], dtype=np.uint8)
        image = np.full((height, width, 3), sky_color, dtype=np.uint8)

        # For rays pointing down (dir_z < 0), calculate exact ground intersection
        # Ray equation: P = origin + t * dir
        # Ground intersection: origin_z + t * dir_z = 0 (ground at z=0)
        # Solve for t: t = -origin_z / dir_z

        # Only process rays pointing downward
        pointing_down = dir_z < 0

        if not pointing_down.any():
            return image  # All sky

        # Calculate intersection distance for downward rays
        # Avoid division by zero
        t = np.zeros_like(dir_z)
        t[pointing_down] = -origin_z / dir_z[pointing_down]

        # Calculate intersection points
        intersect_x = origin_x + t * dir_x
        intersect_y = origin_y + t * dir_y

        # Convert to grid indices
        # World grid is centered: origin is at (-size[0]/2, -size[1]/2)
        grid_i = ((intersect_x + world.size[0]/2) / world.resolution).astype(int)
        grid_j = ((intersect_y + world.size[1]/2) / world.resolution).astype(int)

        # Check which intersections are within world bounds
        valid_mask = pointing_down & \
                     (grid_i >= 0) & (grid_i < world.grid_shape[0]) & \
                     (grid_j >= 0) & (grid_j < world.grid_shape[1])

        if valid_mask.any():
            # Get semantic labels for valid intersections
            valid_i = grid_i[valid_mask]
            valid_j = grid_j[valid_mask]
            semantic_labels = world.semantic_map[valid_j, valid_i]

            # Convert semantic labels to RGB colors (vectorized)
            # Build color array from semantic labels
            for semantic_val, rgb in SEMANTIC_COLORS.items():
                mask = (semantic_labels == semantic_val)
                if mask.any():
                    # Get positions in image where this semantic value appears
                    positions = np.argwhere(valid_mask)
                    local_positions = positions[np.argwhere(mask).flatten()]
                    for pos in local_positions:
                        image[pos[0], pos[1]] = rgb

        return image

    def get_camera_info(self) -> dict:
        """
        Get camera intrinsic parameters.

        Returns:
            Dictionary with K, P, distortion_model, D, width, height
        """
        K = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P = [K | 0] for monocular camera
        P = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return {
            'K': K,
            'P': P,
            'distortion_model': 'plumb_bob',
            'D': [0.0, 0.0, 0.0, 0.0, 0.0],
            'width': self.width,
            'height': self.height,
        }
