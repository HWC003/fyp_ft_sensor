#Subscriber Node for the sensor data 
import rospy
from std_msgs.msg import Float32MultiArray

# Callback function to process incoming sensor data
def sensor_data_callback(msg):
    # The received message is a Float32MultiArray with the sensor data
    fx, fy, fz, mx, my, mz = msg.data

    # Print the sensor data
    print(f"Received sensor data:")
    print(f"FT data: {fx:.5f} {fy:.5f} {fz:.5f} {mx:.5f} {my:.5f} {mz:.5f}")

# Initialize the ROS subscriber node
rospy.init_node('sensor_data_subscriber', anonymous=True)

# Create the subscriber to the 'sensor_data' topic
rospy.Subscriber('sensor_data', Float32MultiArray, sensor_data_callback)

# Spin the node to keep it listening for messages
rospy.spin()