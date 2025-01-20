#!/usr/bin/env python3
import time
import socket
import struct
from collections import deque
import rospy
#from std_msgs.msg import Float32MultiArray


# CircularBuffer Class
class CircularBuffer:
    def __init__(self, capacity=1024):
        self.buffer = deque(maxlen=capacity)

    def write(self, data):
        self.buffer.extend(data)

    def read(self, length):
        if length > len(self.buffer):
            raise ValueError("Not enough data in buffer.")
        return bytearray([self.buffer.popleft() for _ in range(length)])

    def clear(self, length):
        for _ in range(min(length, len(self.buffer))):
            self.buffer.popleft()

    def available(self):
        return len(self.buffer)


# M8218Parser Class
class M8218Parser:
    def __init__(self):
        self.circular_buffer = CircularBuffer()
        self.callback_function = None

    def set_callback(self, callback):
        self.callback_function = callback

    def parse_data_from_buffer(self):
        """
        Parse a data frame from the circular buffer.
        :return: A tuple containing delLen (data to delete) and parsed values (fx, fy, fz, mx, my, mz).
        """
        del_len = 0
        buffer_length = self.circular_buffer.available()
        if buffer_length < 31:  # Minimum frame size with checksum mode
            return del_len, None, None, None, None, None, None

        # Get the raw buffer data
        data = self.circular_buffer.read(buffer_length)

        # Find the header index
        head_index = self.parse_get_head_index(data)
        if head_index == -1:
            del_len = buffer_length - 1  # Discard all but the last byte
            return del_len, None, None, None, None, None, None

        # Validate the frame length
        if head_index + 31 > buffer_length:  # Ensure enough data for the full frame
            del_len = head_index  # Remove invalid data up to the header
            return del_len, None, None, None, None, None, None

        # Check the payload length
        frame_start = head_index + 2
        frame_len = data[frame_start] * 256 + data[frame_start + 1]
        if frame_len != 27:  # Frame length must be 27 bytes
            del_len = frame_start  # Skip the invalid frame
            return del_len, None, None, None, None, None, None

        # Extract floating-point values (fx, fy, fz, mx, my, mz)
        index = frame_start + 4
        try:
            fx, fy, fz, mx, my, mz = struct.unpack("ffffff", data[index:index + 24])
        except struct.error:
            del_len = index
            return del_len, None, None, None, None, None, None

        # Extract the checksum
        index += 24
        received_checksum = data[index]
        del_len = index + 1  # Mark this frame as processed

        # Calculate checksum
        calculated_checksum = 0x00
        for i in range(head_index + 6, head_index + 6 + 24):
            calculated_checksum = (calculated_checksum + data[i]) & 0xFF

        # Validate checksum
        if calculated_checksum != received_checksum:
            return del_len, None, None, None, None, None, None

        # Return parsed data
        return del_len, fx, fy, fz, mx, my, mz

    def parse_get_head_index(self, data):
        """
        Parse the buffer to find the header index.
        :param data: The raw buffer data.
        :return: The index of the frame header, or -1 if not found.
        """
        for i in range(len(data) - 1):
            # Assuming the header is identified by specific byte patterns (replace `HEADER_BYTE_1` and `HEADER_BYTE_2`)
            if data[i] == 0xAA and data[i + 1] == 0x55:  # Replace with actual header bytes
                return i
        return -1

    def on_received_data(self, data):
        """
        Handle incoming raw data and parse it.
        :param data: Raw binary data.
        """
        if not data:
            return False

        # Write data to the circular buffer
        self.circular_buffer.write(data)

        while self.circular_buffer.available() >= 31:  # Minimum frame size
            del_len, fx, fy, fz, mx, my, mz = self.parse_data_from_buffer()
            if del_len > 0:
                self.circular_buffer.clear(del_len)
                if fx is not None and self.callback_function:
                    self.callback_function(fx, fy, fz, mx, my, mz)
            else:
                break


# M8128TCPClient Class
class M8128TCPClient:
    def __init__(self, ip, port):
        self.server_ip = ip
        self.server_port = port
        self.tcp_socket = None
        self.parser = M8218Parser()

    def set_data_callback(self, callback):
        """
        Set the callback for parsed data.
        """
        self.parser.set_callback(callback)

    def open_tcp(self):
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"Connecting to {self.server_ip}:{self.server_port}...")
            self.tcp_socket.connect((self.server_ip, self.server_port))
            print("TCP connection established.")
        except Exception as e:
            print(f"Error opening TCP connection: {e}")
            self.tcp_socket = None

    def close_tcp(self):
        if self.tcp_socket:
            self.tcp_socket.close()
            print("TCP connection closed.")
    
    def send_command(self, command, params=""):
        """
        Send a command to the sensor and wait for acknowledgment.
        :param command: The command to send (e.g., "SMPF").
        :param params: Parameters for the command (e.g., "200").
        :return: True if acknowledgment is received, False otherwise.
        """
        if not self.tcp_socket:
            print("TCP connection is not established.")
            return False

        # Construct the AT command
        at_command = f"AT+{command}={params}\r\n"
        try:
            print(f"Sending command: {at_command.strip()}")
            self.tcp_socket.sendall(at_command.encode())

            if command == "GSD":
                print("Command sent, streaming will start.")
                return True

            # Wait for acknowledgment
            start_time = time.time()
            while True:
                if time.time() - start_time > 5:  # Timeout after 5 seconds
                    print("No response received (timeout).")
                    return False

                # Receive raw response
                response = self.tcp_socket.recv(1024)
                if response:
                    try:
                        decoded_response = response.decode().strip()  # Attempt to decode
                        print(f"Received response: {decoded_response}")
                        if decoded_response.startswith(f"ACK+{command}"):
                            self.parse_ack(decoded_response)
                            return True
                        else:
                            print("Unexpected response.")
                            return False
                    except UnicodeDecodeError:
                        # Handle raw binary data for non-decodable responses
                        print(f"Raw response received (binary): {response}")
                        return False
                    
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def parse_ack(self, response):
        """
        Parse the acknowledgment response from the sensor.
        :param response: The response string (e.g., "ACK+SMPF=200").
        """
        if "=" in response:
            self.command_ack, self.params_ack = response[4:].split("=", 1)
        else:
            self.command_ack = response[4:]
            self.params_ack = ""
        self.is_ack_received = True
        print(f"Parsed ACK: {self.command_ack}, Params: {self.params_ack}")

    def stream_data(self):
        """
        Start streaming data and process it with the parser.
        """
        if not self.tcp_socket:
            rospy.logerr("TCP connection is not established.")
            return

        try:
            rospy.loginfo("Streaming data from the sensor (Press Ctrl+C to stop)...")
            while not rospy.is_shutdown():
                data = self.tcp_socket.recv(1024)
                if data:
                    self.parser.on_received_data(data)
        except rospy.ROSInterruptException:
            rospy.loginfo("Streaming stopped by user.")
        except Exception as e:
            rospy.logerr(f"Error while streaming data: {e}")
    

# Example Callback Function
def example_callback(fx, fy, fz, mx, my, mz):
    print(f"FT data: {fx:.5f} {fy:.5f} {fz:.5f} {mx:.5f} {my:.5f} {mz:.5f}")

# Main Script
if __name__ == "__main__":
    SENSOR_IP = "192.168.1.108"  # Replace with your sensor's IP
    SENSOR_PORT = 4008           # Replace with your sensor's port

    client = M8128TCPClient(SENSOR_IP, SENSOR_PORT)
    client.set_data_callback(example_callback)

    try:
        client.open_tcp()

        if client.send_command("SMPF", "200"):  # Set sampling rate
            print("Sampling rate set successfully.")

        if client.send_command("DCKMD", "SUM"):  # Set checksum mode
            print("Checksum mode set successfully.")

        if client.send_command("GSD", ""):  # Start streaming
            client.stream_data()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close_tcp()
