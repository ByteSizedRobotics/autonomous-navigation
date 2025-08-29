# Code used to confirm the lidar works
# Connect the lidar via the microUSB - USB cable to laptop
# Check the port name, on mac/linux use "ls /dev/cu.*" in terminal
# Idk how on windows maybe check Device Manager

import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
from matplotlib.animation import FuncAnimation

# MAC-SPECIFIC PORT NAME 
# Change this when using a different laptop or device
# On mac/linux, run "ls /dev/cu.*" in terminal to get the port name
PORT_NAME = '/dev/cu.usbserial-10'
# PORT_NAME = '/dev/cu.usbserial-0001'  # Alternate format (check your specific port)
# PORT_NAME = '/dev/cu.usbserial-XXXX'  # XXXX = Last 4 digits of your serial number

lidar = RPLidar(PORT_NAME)

# Set up the figure and polar plot
fig = plt.figure(dpi=120)       # Increase DPI for higher resolution
ax = plt.subplot(111, projection='polar')
ax.set_theta_zero_location('N') # Set 0° to be at the top
ax.set_theta_direction(-1)      # Clockwise rotation
scat = ax.scatter([], [], s=2, c='blue', alpha=0.5)  # Smaller points with transparency

# Change this to change max distance displayed, distance is in mm (1000mm = 1m)
ax.set_rmax(1000) 

def update(frame):
    try:
        scan = next(lidar.iter_scans())
    except StopIteration:
        lidar.stop()
        lidar.disconnect()
        return scat,
    
    # Extract angles and distances
    angles = np.deg2rad([item[1] for item in scan])  # Convert degrees to radians
    distances = [item[2] for item in scan]           # Extract distances
    
    # Filter out invalid points (distance = 0)
    valid = [d > 0 for d in distances]
    angles = np.array(angles)[valid]
    distances = np.array(distances)[valid]
    
    # Update scatter plot data
    scat.set_offsets(np.c_[angles, distances])
    return scat,

def clean_exit():
    """Clean up function for closing the lidar connection"""
    print("Stopping lidar...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Lidar connection closed.")

def on_key(event):
    if event.key == 'up':
        ax.set_rmax(ax.get_rmax() + 500)  # Zoom out
    elif event.key == 'down':
        ax.set_rmax(max(500, ax.get_rmax() - 500))  # Zoom in
    plt.draw()

try:
    # Start the lidar motor
    lidar.start_motor()
    
    # Set up animation
    ani = FuncAnimation(fig, update, interval=50, blit=True)
    
    # Show the plot
    plt.show()

    fig.canvas.mpl_connect('key_press_event', on_key)


except KeyboardInterrupt:
    pass
finally:
    clean_exit()