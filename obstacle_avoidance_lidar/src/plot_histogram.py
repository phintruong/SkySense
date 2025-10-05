"""
Matplotlib script for visualizing obstacle avoidance histogram data.

This script subscribes to diagnostic topics from the obstacle avoidance node
and plots sector clearance over time for debugging and analysis.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import time
from typing import List, Deque, Optional


class HistogramPlotter(Node):
    """
    Node that subscribes to diagnostic topics and plots histogram data.
    
    This node visualizes the obstacle avoidance system's decision-making
    process by plotting sector clearance over time.
    """
    
    def __init__(self):
        super().__init__('histogram_plotter')
        
        # Declare parameters
        self.declare_parameter('plot_window_size', 100)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('save_plots', False)
        self.declare_parameter('plot_directory', '/tmp/obstacle_avoidance_plots')
        
        # Get parameters
        self.plot_window_size = self.get_parameter('plot_window_size').value
        self.update_rate = self.get_parameter('update_rate').value
        self.save_plots = self.get_parameter('save_plots').value
        self.plot_directory = self.get_parameter('plot_directory').value
        
        # Data storage
        self.min_distances: Deque[float] = deque(maxlen=self.plot_window_size)
        self.chosen_sectors: Deque[int] = deque(maxlen=self.plot_window_size)
        self.velocities: Deque[Tuple[float, float, float]] = deque(maxlen=self.plot_window_size)
        self.timestamps: Deque[float] = deque(maxlen=self.plot_window_size)
        
        # Current data
        self.current_min_distance: Optional[float] = None
        self.current_chosen_sector: Optional[int] = None
        self.current_velocity: Optional[Tuple[float, float, float]] = None
        
        # Create subscribers
        self.min_distance_subscription = self.create_subscription(
            Float32,
            '/obstacle_avoidance_node/min_distance',
            self.min_distance_callback,
            10
        )
        
        self.chosen_sector_subscription = self.create_subscription(
            Int32,
            '/obstacle_avoidance_node/chosen_sector',
            self.chosen_sector_callback,
            10
        )
        
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/obstacle_avoidance_node/velocity_debug',
            self.velocity_callback,
            10
        )
        
        # Initialize matplotlib
        self._setup_plots()
        
        # Create timer for plotting
        self.plot_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_plots
        )
        
        self.get_logger().info('Histogram plotter initialized')
        self.get_logger().info(f'Plot window size: {self.plot_window_size}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
    
    def _setup_plots(self):
        """Setup matplotlib figures and axes."""
        # Create figure with subplots
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Obstacle Avoidance System Diagnostics')
        
        # Configure subplots
        self.ax1.set_title('Minimum Distance Over Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Distance (m)')
        self.ax1.grid(True)
        
        self.ax2.set_title('Chosen Sector Over Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Sector Index')
        self.ax2.grid(True)
        
        self.ax3.set_title('Velocity Commands Over Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Velocity (m/s)')
        self.ax3.grid(True)
        
        self.ax4.set_title('Yaw Rate Over Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Yaw Rate (rad/s)')
        self.ax4.grid(True)
        
        # Initialize plot lines
        self.line1, = self.ax1.plot([], [], 'b-', label='Min Distance')
        self.line2, = self.ax2.plot([], [], 'r-', label='Chosen Sector')
        self.line3_vx, = self.ax3.plot([], [], 'g-', label='Vx')
        self.line3_vy, = self.ax3.plot([], [], 'b-', label='Vy')
        self.line4, = self.ax4.plot([], [], 'm-', label='Yaw Rate')
        
        # Add legends
        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()
        self.ax4.legend()
        
        # Adjust layout
        plt.tight_layout()
        
        # Show plot
        plt.show(block=False)
    
    def min_distance_callback(self, msg: Float32):
        """Callback for minimum distance messages."""
        self.current_min_distance = msg.data
    
    def chosen_sector_callback(self, msg: Int32):
        """Callback for chosen sector messages."""
        self.current_chosen_sector = msg.data
    
    def velocity_callback(self, msg: Twist):
        """Callback for velocity debug messages."""
        self.current_velocity = (msg.linear.x, msg.linear.y, msg.angular.z)
    
    def update_plots(self):
        """Update the plots with new data."""
        try:
            current_time = time.time()
            
            # Add new data points if available
            if self.current_min_distance is not None:
                self.min_distances.append(self.current_min_distance)
                self.timestamps.append(current_time)
            
            if self.current_chosen_sector is not None:
                self.chosen_sectors.append(self.current_chosen_sector)
            
            if self.current_velocity is not None:
                self.velocities.append(self.current_velocity)
            
            # Update plots if we have data
            if len(self.timestamps) > 1:
                self._update_plot_data()
                
                # Save plots if requested
                if self.save_plots and len(self.timestamps) % 50 == 0:
                    self._save_plots()
            
        except Exception as e:
            self.get_logger().error(f'Error updating plots: {e}')
    
    def _update_plot_data(self):
        """Update the plot data and redraw."""
        try:
            if not self.timestamps:
                return
            
            # Convert to numpy arrays
            times = np.array(self.timestamps)
            times = times - times[0]  # Relative time
            
            # Update minimum distance plot
            if self.min_distances:
                self.line1.set_data(times, list(self.min_distances))
                self.ax1.relim()
                self.ax1.autoscale_view()
            
            # Update chosen sector plot
            if self.chosen_sectors:
                self.line2.set_data(times, list(self.chosen_sectors))
                self.ax2.relim()
                self.ax2.autoscale_view()
            
            # Update velocity plots
            if self.velocities:
                vx_data = [v[0] for v in self.velocities]
                vy_data = [v[1] for v in self.velocities]
                yaw_data = [v[2] for v in self.velocities]
                
                self.line3_vx.set_data(times, vx_data)
                self.line3_vy.set_data(times, vy_data)
                self.line4.set_data(times, yaw_data)
                
                self.ax3.relim()
                self.ax3.autoscale_view()
                self.ax4.relim()
                self.ax4.autoscale_view()
            
            # Redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f'Error updating plot data: {e}')
    
    def _save_plots(self):
        """Save current plots to file."""
        try:
            import os
            os.makedirs(self.plot_directory, exist_ok=True)
            
            timestamp = int(time.time())
            filename = f"{self.plot_directory}/obstacle_avoidance_plot_{timestamp}.png"
            
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Saved plot to {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving plots: {e}')
    
    def create_histogram_plot(self, sector_distances: List[float], 
                            sector_angles: List[float], safe_distance: float):
        """
        Create a static histogram plot of sector distances.
        
        Args:
            sector_distances: List of minimum distances for each sector
            sector_angles: List of center angles for each sector
            safe_distance: Safe distance threshold
        """
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            
            # Plot 1: Sector distances as bar chart
            ax1.bar(range(len(sector_distances)), sector_distances, alpha=0.7)
            ax1.axhline(y=safe_distance, color='r', linestyle='--', label=f'Safe Distance ({safe_distance}m)')
            ax1.set_xlabel('Sector Index')
            ax1.set_ylabel('Distance (m)')
            ax1.set_title('Sector Distances')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Polar plot of sector distances
            angles_deg = [math.degrees(angle) for angle in sector_angles]
            ax2 = plt.subplot(122, projection='polar')
            ax2.plot([math.radians(angle) for angle in angles_deg], sector_distances, 'b-', linewidth=2)
            ax2.fill_between([math.radians(angle) for angle in angles_deg], 0, sector_distances, alpha=0.3)
            ax2.set_title('Sector Distances (Polar)')
            ax2.set_ylim(0, max(sector_distances) * 1.1)
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Error creating histogram plot: {e}')


def main(args=None):
    """
    Main function to run the histogram plotter.
    """
    rclpy.init(args=args)
    
    try:
        plotter = HistogramPlotter()
        
        # Keep the node running
        rclpy.spin(plotter)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in histogram plotter: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

