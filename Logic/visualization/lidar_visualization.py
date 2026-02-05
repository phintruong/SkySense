#!/usr/bin/env python3
"""
LiDAR Visualization Module

Provides real-time radar-style visualization of RPLIDAR scan data using Pygame.
Displays distance-based color coding and reference circles for spatial reference.
"""

import math
import pygame
import logging
import sys
import os
from typing import List, Tuple, Optional

# Add parent directory to path for imports
_LOGIC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _LOGIC_DIR not in sys.path:
    sys.path.insert(0, _LOGIC_DIR)

from hardware.rplidar_reader import RPLidarReader

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Display configuration
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
MIN_DISTANCE_MM = 50
MAX_DISTANCE_MM = 3000
SCALE = (WIDTH // 2) / MAX_DISTANCE_MM

# Color definitions
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)


def polar_to_cartesian(angle_deg: float, distance_m: float) -> Tuple[int, int]:
    """
    Convert polar coordinates to Cartesian screen coordinates.
    
    Args:
        angle_deg: Angle in degrees (0-359)
        distance_m: Distance in meters
        
    Returns:
        Tuple (x, y) screen coordinates
    """
    angle_rad = math.radians(angle_deg)
    distance_mm = distance_m * 1000.0  # Convert to millimeters for scaling
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * math.cos(angle_rad))
    y = CENTER[1] - int(r * math.sin(angle_rad))  # Negative for screen coordinates
    return x, y


def get_distance_color(distance_mm: float) -> Tuple[int, int, int]:
    """
    Get color based on distance threshold.
    
    Args:
        distance_mm: Distance in millimeters
        
    Returns:
        RGB color tuple
    """
    if distance_mm <= 1000:  # 0-1.0 m: Close (red)
        return RED
    elif distance_mm <= 2000:  # 1.0-2.0 m: Medium (yellow)
        return YELLOW
    else:  # 2.0-3.0 m: Far (green)
        return GREEN


class LidarVisualizer:
    """
    Professional LiDAR data visualization using Pygame.
    """
    
    def __init__(self, title: str = "LiDAR Radar System"):
        """
        Initialize visualization window.
        
        Args:
            title: Window title
        """
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.SysFont(None, 20)
        self.font_title = pygame.font.SysFont(None, 28, bold=True)
        self.running = True
        
    def draw_reference_circles(self):
        """Draw reference distance circles and labels."""
        # Draw circles every 50 cm (500 mm)
        for r in range(500, MAX_DISTANCE_MM + 1, 500):
            pygame.draw.circle(self.screen, DARK_GREEN, CENTER, int(r * SCALE), 1)
            # Draw distance label
            label = self.font_small.render(f"{r//10} cm", True, WHITE)
            label_x = CENTER[0] + int(r * SCALE) - 25
            label_y = CENTER[1]
            self.screen.blit(label, (label_x, label_y))
    
    def draw_scan_points(self, scan_data: List[Tuple[float, float]]):
        """
        Draw scan points with distance-based coloring.
        
        Args:
            scan_data: List of tuples [(angle_degrees, distance_meters), ...]
        """
        for angle, distance in scan_data:
            px, py = polar_to_cartesian(angle, distance)
            distance_mm = distance * 1000.0
            color = get_distance_color(distance_mm)
            pygame.draw.circle(self.screen, color, (px, py), 2)
    
    def draw_title(self, title: str):
        """
        Draw title text at bottom of screen.
        
        Args:
            title: Title text to display
        """
        title_surface = self.font_title.render(title, True, WHITE)
        title_x = WIDTH // 2 - title_surface.get_width() // 2
        title_y = HEIGHT - 40
        self.screen.blit(title_surface, (title_x, title_y))
    
    def update(self, scan_data: List[Tuple[float, float]], title: str = "LiDAR Radar System"):
        """
        Update display with new scan data.
        
        Args:
            scan_data: List of tuples [(angle_degrees, distance_meters), ...]
            title: Window title
        """
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
        
        # Clear screen
        self.screen.fill(BLACK)
        
        # Draw reference circles
        self.draw_reference_circles()
        
        # Draw scan points
        self.draw_scan_points(scan_data)
        
        # Draw title
        self.draw_title(title)
        
        # Update display
        pygame.display.flip()
        self.clock.tick(60)
    
    def quit(self):
        """Close visualization window."""
        pygame.quit()


def visualize_lidar(port: str = '/dev/ttyUSB0', motor_gpio: Optional[int] = None,
                    title: str = "LiDAR Radar System"):
    """
    Main visualization function for real-time LiDAR display.

    Args:
        port: Serial port for RPLIDAR
        motor_gpio: GPIO pin for motor control (BCM numbering, e.g., 12)
        title: Window title
    """
    reader = RPLidarReader(port=port, motor_gpio_pin=motor_gpio)
    visualizer = LidarVisualizer(title=title)
    
    try:
        # Connect and start RPLIDAR
        if not reader.connect():
            logger.error("Failed to connect to RPLIDAR")
            return
        
        if not reader.start():
            logger.error("Failed to start RPLIDAR")
            reader.disconnect()
            return
        
        logger.info("LiDAR visualization active. Close window to stop.")
        
        # Main visualization loop
        for scan_data in reader.iter_scans():
            if not visualizer.running:
                break
            
            visualizer.update(scan_data, title)
            logger.debug(f"Displayed scan with {len(scan_data)} points")
    
    except KeyboardInterrupt:
        logger.info("Visualization interrupted by user")
    except Exception as e:
        logger.error(f"Visualization error: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        reader.disconnect()
        visualizer.quit()
        logger.info("Visualization shutdown complete")


def main():
    """Standalone execution of LiDAR visualization."""
    import sys

    port = '/dev/ttyUSB0'
    motor_gpio = None

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        motor_gpio = int(sys.argv[2])

    visualize_lidar(port=port, motor_gpio=motor_gpio)


if __name__ == "__main__":
    main()

