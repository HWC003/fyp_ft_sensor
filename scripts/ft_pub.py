#!/usr/bin/env python3

#Publisher for force torque sensor data

import rospy
from std_msgs.msg import Float32MultiArray
from test_tcp import M8128TCPClient 

#Callback function to publish the sensor data
def sensor_data_callback(fx, fy, fz, mx, my, mz):
    """
    Callback function to publish the sensor data to the 'sensor_data' topic.
    :param fx: Force in X-axis
    :param fy: Force in Y-axis
    :param fz: Force in Z-axis
    :param mx: Torque in X-axis
    :param my: Torque in Y-axis
    :param mz: Torque in Z-axis
    """
    #Create the message
    current_data = Float32MultiArray(data = [fx, fy, fz, mx, my, mz])

    #Publish the message
    pub.publish(current_data)
    rospy.loginfo(f"Published data: {current_data.data}")

if __name__ == "__main__":

    #Initialise the ROS publisher node
    rospy.init_node('ft_sensor_node', anonymous=True)

    #Create the publisher object
    pub = rospy.Publisher('/sri/ft_sensor_data', Float32MultiArray, queue_size=10)

    #Sensor connection details
    SENSOR_IP = "192.168.1.108"  # Replace with your sensor's IP
    SENSOR_PORT = 4008           # Replace with your sensor's port

    #Initialise the client
    client = M8128TCPClient(SENSOR_IP, SENSOR_PORT)
    client.set_data_callback(sensor_data_callback)

    try:
        client.open_tcp()

        #Set sampling rate
        if client.send_command("SMPF", "200"):
            rospy.loginfo("Sampling rate set sucessfully.")

        #Set checksum mode
        if client.send_command("DCKMD", "SUM"):
            rospy.loginfo("Checksum mode set successfully.")

        #Start streaming data
        if client.send_command("GSD", ""):
            rospy.loginfo("Getting data from the sensor (Press Ctrl+C to stop)...")
            client.stream_data()

    except rospy.ROSInterruptException:
        rospy.loginfo("Streaming stopped by user.")
    except Exception as e:
        rospy.logerr(f"Error while streaming data: {e}")
    finally:
        client.close_tcp()
        rospy.loginfo("TCP connection closed.")
