"""
World - 2D grid world representation with semantic labels.

Responsibilities:
- Maintain 2D occupancy grid with semantic classes
- Support queries for height, semantic class at (x, y)
- Load from scenario YAML configuration
"""

import numpy as np
from enum import IntEnum
from typing import Tuple, Optional


class SemanticClass(IntEnum):
    """Semantic labels for world surfaces."""
    GROUND = 0      # Flat navigable ground
    OBSTACLE = 1    # Red - Critical victims (circles)
    HAZARD = 2      # Orange - Wounded victims (circles)
    TARGET = 3      # Dark green - Safe victims (circles)
    WATER = 4       # Yellow - Trees (squares)
    BUILDING = 5    # Brown - Buildings (squares)
    DEBRIS = 6      # Magenta - Debris (squares)
    VEHICLE = 7     # Dark gray - Vehicles (squares)
    FREE = 8        # Reserved for future use


class World:
    """2D grid world with semantic labels and height map."""

    def __init__(self, size: Tuple[float, float] = (100.0, 100.0),
                 resolution: float = 0.5):
        """
        Initialize world grid.

        Args:
            size: World size in meters (x, y)
            resolution: Grid cell size in meters
        """
        self.size = size
        self.resolution = resolution

        # Create grid dimensions
        self.grid_shape = (
            int(size[0] / resolution),
            int(size[1] / resolution)
        )

        # Initialize grids
        self.height_map = np.zeros(self.grid_shape, dtype=np.float32)
        self.semantic_map = np.full(self.grid_shape, SemanticClass.GROUND, dtype=np.int8)

    def get_height(self, x: float, y: float) -> float:
        """
        Get height at world position (x, y).

        Args:
            x, y: World coordinates in meters

        Returns:
            Height in meters (0.0 if out of bounds)
        """
        i, j = self._world_to_grid(x, y)
        if self._in_bounds(i, j):
            return float(self.height_map[i, j])
        return 0.0

    def get_semantic(self, x: float, y: float) -> SemanticClass:
        """
        Get semantic class at world position (x, y).

        Args:
            x, y: World coordinates in meters

        Returns:
            Semantic class (GROUND if out of bounds)
        """
        i, j = self._world_to_grid(x, y)
        if self._in_bounds(i, j):
            return SemanticClass(self.semantic_map[i, j])
        return SemanticClass.GROUND

    def set_region(self, x: float, y: float, radius: float,
                   height: float, semantic: SemanticClass, shape: str = 'circular') -> None:
        """
        Set region with height and semantic label.

        Args:
            x, y: Center position in meters
            radius: Radius in meters (for circular) or half-width (for square)
            height: Height value
            semantic: Semantic class
            shape: 'circular' or 'square' - shape of the region
        """
        i_center, j_center = self._world_to_grid(x, y)
        radius_cells = int(radius / self.resolution)

        if shape == 'square':
            # Square region: fill all cells within bounding box
            for i in range(max(0, i_center - radius_cells),
                          min(self.grid_shape[0], i_center + radius_cells + 1)):
                for j in range(max(0, j_center - radius_cells),
                              min(self.grid_shape[1], j_center + radius_cells + 1)):
                    self.height_map[i, j] = height
                    self.semantic_map[i, j] = semantic
        else:
            # Circular region: check distance from center
            for i in range(max(0, i_center - radius_cells),
                          min(self.grid_shape[0], i_center + radius_cells + 1)):
                for j in range(max(0, j_center - radius_cells),
                              min(self.grid_shape[1], j_center + radius_cells + 1)):
                    dist = np.sqrt((i - i_center)**2 + (j - j_center)**2)
                    if dist <= radius_cells:
                        self.height_map[i, j] = height
                        self.semantic_map[i, j] = semantic

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        i = int((x + self.size[0]/2) / self.resolution)
        j = int((y + self.size[1]/2) / self.resolution)
        return i, j

    def _in_bounds(self, i: int, j: int) -> bool:
        """Check if grid indices are valid."""
        return 0 <= i < self.grid_shape[0] and 0 <= j < self.grid_shape[1]
