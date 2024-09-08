import serial
import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Serial port configuration
ser = serial.Serial(port='COM7', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

# Initialize data storage for real-time plotting and tracking
lidar_data = []
previous_nearest_distance = None  # Track previous nearest distance for comparison
approaching_objects = {}  # Dictionary to store approaching objects with time data

# Function to decode a single packet of LIDAR data
def decode_string(string):
    data = []
    for byte in string.strip("\n").split(":")[:21]:
        data.append(int(byte, 16))
    
    start = data[0]
    if start != 0xFA:
        print(f"Invalid start byte: {start}")
        return
    
    idx = data[1] - 0xA0
    for i in range(4):
        angle = idx * 4 + i
        dist_mm = data[4 + i * 4] | ((data[5 + i * 4] & 0x1F) << 8)
        quality = data[6 + i * 4] | (data[7 + i * 4] << 8)
        
        if quality > 0:
            lidar_data.append((angle, dist_mm))

# Function to read and process data from the LIDAR
def read_lidar_data():
    byte = ser.read(1)
    started = False
    string = "Start"
    
    while True:
        if byte != b'':
            enc = (byte.hex() + ":")
            if enc == "fa:":
                if started:
                    try:
                        decode_string(string)
                    except Exception as e:
                        print(f"Error decoding string: {e}")
                started = True
                string = "fa:"
            elif started:
                string += enc
        byte = ser.read(1)
        if len(lidar_data) >= 360:
            break

# Function to estimate the speed of approaching objects based on time
def estimate_speed(distance_now, distance_prev, time_diff):
    if time_diff > 0:
        speed = abs(distance_now - distance_prev) / time_diff  # Speed in mm per second
        return speed
    return 0

# Function to update the plot in real-time
def update_plot(frame):
    global previous_nearest_distance, approaching_objects

    # Clear old data points
    ax.clear()

    # Convert polar coordinates to Cartesian coordinates
    x_coords = []
    y_coords = []
    
    for angle, distance in lidar_data:
        angle_rad = math.radians(angle)
        x = distance * math.cos(angle_rad) / 10  # Convert mm to cm
        y = distance * math.sin(angle_rad) / 10  # Convert mm to cm
        x_coords.append(x)
        y_coords.append(y)

    # Update plot with new data
    ax.scatter(x_coords, y_coords, s=5, c='orange', label='LIDAR Points')

    # Sensor position with label
    ax.scatter(0, 0, color='blue', s=100, label="LIDAR Sensor Position")
    ax.text(0, 0, "LIDAR", fontsize=10, color='blue')

    # Find and label nearest and farthest points
    if lidar_data:
        # Nearest and farthest object detection
        min_distance = min(lidar_data, key=lambda x: x[1])
        max_distance = max(lidar_data, key=lambda x: x[1])

        # Nearest point (green)
        min_x = min_distance[1] * math.cos(math.radians(min_distance[0])) / 10
        min_y = min_distance[1] * math.sin(math.radians(min_distance[0])) / 10
        ax.scatter(min_x, min_y, color='green', s=50, label=f'Nearest: {min_distance[1] / 10:.1f}cm')
        ax.text(min_x, min_y, f'Nearest: {min_distance[1] / 10:.1f}cm', fontsize=9, color='green')

        # Farthest point (red)
        max_x = max_distance[1] * math.cos(math.radians(max_distance[0])) / 10
        max_y = max_distance[1] * math.sin(math.radians(max_distance[0])) / 10
        ax.scatter(max_x, max_y, color='red', s=50, label=f'Farthest: {max_distance[1] / 10:.1f}cm')
        ax.text(max_x, max_y, f'Farthest: {max_distance[1] / 10:.1f}cm', fontsize=9, color='red')

        # Object detection: Check for approaching objects
        current_time = time.time()
        if previous_nearest_distance is not None:
            if min_distance[1] < previous_nearest_distance and min_distance[1] < 5000:  # 5000mm = 500cm
                if min_distance[0] not in approaching_objects:
                    # Add object if it's newly detected
                    approaching_objects[min_distance[0]] = {
                        'distance': min_distance[1],
                        'timestamp': current_time,
                        'previous_distance': previous_nearest_distance
                    }
                else:
                    # Update existing object's data
                    approaching_objects[min_distance[0]]['distance'] = min_distance[1]
                    approaching_objects[min_distance[0]]['timestamp'] = current_time

        # Update the previous nearest distance
        previous_nearest_distance = min_distance[1]

        # Mark approaching objects and calculate speed
        approaching_object_count = 0
        for obj_angle, obj in list(approaching_objects.items()):
            obj_angle_rad = math.radians(obj_angle)
            obj_dist = obj['distance'] / 10  # Convert mm to cm
            obj_x = obj_dist * math.cos(obj_angle_rad)
            obj_y = obj_dist * math.sin(obj_angle_rad)

            # Calculate speed
            time_diff = current_time - obj['timestamp']
            speed = estimate_speed(obj['distance'], obj['previous_distance'], time_diff) / 10  # Convert to cm/s

            # Mark approaching object with a blue triangle
            ax.scatter(obj_x, obj_y, color='blue', s=100, marker='^', label=f'Approaching: {obj_dist:.1f}cm')
            ax.text(obj_x, obj_y, f'Approaching: {obj_dist:.1f}cm\nSpeed: {speed:.2f}cm/s', fontsize=9, color='blue')
            approaching_object_count += 1

        # Remove objects that are no longer approaching
        approaching_objects = {
            k: v for k, v in approaching_objects.items() if current_time - v['timestamp'] < 1
        }

        # Show alert if approaching objects are detected, else clear the alert
        if approaching_object_count > 0:
            ax.text(-400, 450, f"Alert: {approaching_object_count} object(s) approaching!", fontsize=12, color='blue')
        else:
            ax.text(-400, 450, "", fontsize=12, color='blue')

    # Project label
    ax.text(-400, 500, "Project by: Halit Osman Efkere", fontsize=12, color='blue')

    ax.set_xlim(-500, 500)  # Limits in cm
    ax.set_ylim(-500, 500)  # Limits in cm
    ax.set_title('Real-time LIDAR Data Visualization')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.grid(True)

# Set up the plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update_plot, interval=100)

# Main loop to read LIDAR data and update the plot
if __name__ == "__main__":
    print("Start reading LIDAR data...")
    while True:
        lidar_data.clear()  # Clear previous data
        read_lidar_data()   # Read new LIDAR data
        plt.pause(0.001)    # Small delay for real-time updates
    plt.show()
