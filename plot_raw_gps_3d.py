"""
Raw GPS Data 3D Visualization

This script plots ONLY the raw GPS data in a 3D coordinate system.
No Kalman filtering or any other calculations are applied.

Usage:
    python plot_raw_gps_3d.py
"""

import os
import sys
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Add src directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.dataset.uav import PX4_GPSDataReader
from src.common.datatypes import SensorType


class RawGPS3DPlotter:
    """
    Simple 3D plotter for raw GPS data without any filtering.
    """

    def __init__(self):
        """Initialize the plotter."""
        self.raw_positions = []
        self.timestamps = []
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None

    def lat_lon_alt_to_xyz(self, lat, lon, alt):
        """
        Convert GPS coordinates to local Cartesian coordinates.
        Uses simple approximation for small distances.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            
        Returns:
            Tuple of (x, y, z) in meters
        """
        if self.ref_lat is None:
            self.ref_lat = lat
            self.ref_lon = lon
            self.ref_alt = alt
            return 0.0, 0.0, 0.0

        # Convert degrees to radians
        lat_rad = math.radians(lat)
        ref_lat_rad = math.radians(self.ref_lat)

        # Earth radius in meters
        R = 6371000

        # Calculate x, y, z in meters from reference point
        x = R * math.cos(ref_lat_rad) * math.radians(lon - self.ref_lon)
        y = R * math.radians(lat - self.ref_lat)
        z = alt - self.ref_alt

        return x, y, z

    def add_gps_point(self, timestamp, lat, lon, alt):
        """
        Add a GPS data point.
        
        Args:
            timestamp: GPS timestamp
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        x, y, z = self.lat_lon_alt_to_xyz(lat, lon, alt)
        self.raw_positions.append([x, y, z])
        self.timestamps.append(timestamp)

    def plot_3d_trajectory(self):
        """
        Plot the raw GPS trajectory in 3D.
        """
        if len(self.raw_positions) == 0:
            print("No GPS data to plot")
            return

        raw_positions = np.array(self.raw_positions)

        # Create 3D plot
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Plot raw GPS trajectory
        ax.plot(raw_positions[:, 0], raw_positions[:, 1], raw_positions[:, 2],
                'b.-', alpha=0.7, label='Raw GPS Trajectory', markersize=4, linewidth=1.5)

        # Mark start point (green)
        ax.scatter(raw_positions[0, 0], raw_positions[0, 1], raw_positions[0, 2],
                   c='green', s=200, marker='o', label='Start Point', edgecolors='black', linewidths=2)

        # Mark end point (red)
        ax.scatter(raw_positions[-1, 0], raw_positions[-1, 1], raw_positions[-1, 2],
                   c='red', s=200, marker='s', label='End Point', edgecolors='black', linewidths=2)

        # Set labels and title
        ax.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y (meters)', fontsize=12, fontweight='bold')
        ax.set_zlabel('Z (meters)', fontsize=12, fontweight='bold')
        ax.set_title('Raw GPS Trajectory in 3D (No Filtering)', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11)

        # Set equal aspect ratio for better visualization
        max_range = np.array([
            raw_positions[:, 0].max() - raw_positions[:, 0].min(),
            raw_positions[:, 1].max() - raw_positions[:, 1].min(),
            raw_positions[:, 2].max() - raw_positions[:, 2].min()
        ]).max() / 2.0

        mid_x = (raw_positions[:, 0].max() + raw_positions[:, 0].min()) * 0.5
        mid_y = (raw_positions[:, 1].max() + raw_positions[:, 1].min()) * 0.5
        mid_z = (raw_positions[:, 2].max() + raw_positions[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Add grid
        ax.grid(True, alpha=0.3)

        # Add statistics text box
        stats_text = f"Total GPS Points: {len(raw_positions)}\n"
        stats_text += f"Start: ({raw_positions[0, 0]:.2f}, {raw_positions[0, 1]:.2f}, {raw_positions[0, 2]:.2f})\n"
        stats_text += f"End: ({raw_positions[-1, 0]:.2f}, {raw_positions[-1, 1]:.2f}, {raw_positions[-1, 2]:.2f})"
        
        ax.text2D(0.02, 0.98, stats_text, transform=ax.transAxes,
                  fontsize=10, verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        return fig

    def save(self, filename):
        """Save the plot to file."""
        fig = self.plot_3d_trajectory()
        if fig:
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved: {filename}")

    def show(self):
        """Display the plot."""
        self.plot_3d_trajectory()
        plt.show()


def main():
    """Main function to plot raw GPS data."""
    print("\n" + "=" * 70)
    print(" " * 20 + "RAW GPS 3D VISUALIZATION")
    print("=" * 70)

    # Get data path
    base_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(base_dir, "data", "UAV")

    # Check if data exists
    if not os.path.exists(data_path):
        print(f"Error: Data path not found: {data_path}")
        return

    log_name = "log0001"
    gps_file = os.path.join(data_path, f"{log_name}/px4/09_00_22_sensor_gps_0.csv")

    if not os.path.exists(gps_file):
        print(f"Error: GPS data file not found: {gps_file}")
        return

    print(f"\nData directory: {data_path}")
    print(f"Log: {log_name}")
    print(f"GPS file: 09_00_22_sensor_gps_0.csv")
    print("\nProcessing GPS data...\n")

    # Initialize GPS data reader
    gps_reader = PX4_GPSDataReader(
        path=gps_file,
        sensor_type=SensorType.PX4_GPS
    )

    # Initialize plotter
    plotter = RawGPS3DPlotter()

    gps_count = 0
    valid_count = 0

    print("Reading GPS data points...")

    for data in gps_reader:
        # Extract GPS coordinates
        lat = data.lat / 1e7  # Convert from int to degrees
        lon = data.lon / 1e7
        alt = data.alt / 1e3  # Convert from mm to meters

        gps_count += 1

        # Skip invalid GPS data
        if lat == 0.0 or lon == 0.0 or abs(lat) < 1e-6 or abs(lon) < 1e-6:
            continue

        # Add to plotter
        plotter.add_gps_point(data.timestamp, lat, lon, alt)
        valid_count += 1

        if valid_count == 1:
            print(f"First valid GPS point: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}m")

        if valid_count % 50 == 0:
            print(f"  Processed {valid_count} valid GPS points...")

        # uncomment when you want to limit the number of points plotted
        #num_points = 1000  # Maximum number of points to plot
        #if valid_count >= num_points:
        #    break

    print(f"\nTotal GPS records read: {gps_count}")
    print(f"Valid GPS points: {valid_count}")

    if valid_count < 2:
        print("\nError: Not enough valid GPS data for visualization")
        print("Need at least 2 valid GPS points")
        return

    print(f"\nGenerating 3D plot...")

    # Save the plot
    output_file = "output_raw_gps_3d.png"
    #plotter.save(output_file)

    print("\n" + "=" * 70)
    print("VISUALIZATION COMPLETE!")
    print("=" * 70)
    print(f"\nGenerated file: {output_file}")
    print("\nDisplaying plot... (close window to exit)")
    print("=" * 70 + "\n")

    # Show the plot
    plotter.show()



if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as e:
        print(f"\nError: Required file not found: {e}")
        print("Make sure the UAV dataset is properly extracted in the data/UAV directory.")
    except Exception as e:
        print(f"\nError during visualization: {e}")
        import traceback
        traceback.print_exc()

