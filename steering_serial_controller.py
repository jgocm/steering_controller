import serial
import math

class SerialDriver:
    '''
    A class for sending commands and receiving data to/from a KY170DD01005-08G 
    ELECTRIC STEERING MOTOR through serial port.

    Baudrate must be configured to 115200 as specified by the User Manual.

    Send messages must be 4 bytes long.

    Feedback messages are 5 bytes long.

    Time interval between two commands when sending commands continuously is
    20ms < Time interval < 500ms.

    '''

    def __init__(self,
                 debug=False,
                 port='/dev/ttyUSB0', 
                 baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.debug = debug
        self.DEFAULT_HEX_MESSAGE = bytearray([0XAD, 0X00, 0X00, 0XAD])

    def open_serial_port(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate)
            print(f"Serial port '{self.port}' opened successfully.")
        except Exception as e:
            print(f"Failed to open serial port '{self.port}': {str(e)}")

    def send_hex_message(self, hex_message):
        if self.ser:
            try:
                # Send the message
                self.ser.write(hex_message)

                # Read 5 bytes feedback data
                feedback_data = self.ser.read(5).hex()
                return feedback_data

            except Exception as e:
                print(f"Failed to send message: {str(e)}")
                return None
                
        else:
            print("Serial port is not open. Call 'open_serial_port()' first.")
            return None

    def close_serial_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Serial port '{self.port}' closed.")
        else:
            print("Serial port is not open.")

    def encode_speed_msg_to_bytes(self, desired_speed=0, motor_enable=True):
        # Defines message header based on datasheet specifications
        if motor_enable:
            # Motor Enable
            byte1 = 0xAD
        else:
            # Motor Disable
            byte1 = 0xAC            

        # Encode Speed Value to Hex
        byte2, byte3 = self.encode_speed_value_to_hex(desired_speed)

        # Encode Check Sum
        byte4 = self.encode_check_sum(byte1, byte2, byte3)

        hex_message = [byte1, byte2, byte3, byte4]

        return hex_message

    def encode_check_sum(self, byte1, byte2, byte3):
        # Compute checksum from previous bytes
        checksum = byte1 + byte2 + byte3

        # Take the low digit (lowest byte) of the checksum
        checksum = checksum & 0xFF

        return checksum

    def encode_speed_value_to_hex(self, speed):
        # Limit speed command to +-1000
        if abs(speed)>1000:
            speed = 1000*math.copysign(1, speed)
        
        # Get first 16 bits
        hex_value = int(speed) & 0xFFFF

        # Convert the hex value to two separate bytes in hexadecimal format
        byte1 = (hex_value >> 8) & 0xFF
        byte2 = hex_value & 0xFF
        return byte1, byte2

    def encode_bytes_to_hex_msg(self, bytes_list):  # not in use
        msg = []
        for msg_byte in bytes_list:
            msg.append(msg_byte)

        if len(msg)==4:
            hex_message = bytearray(msg)
            return hex_message
        else:
            print("Too many arguments for encoding message!")
            return self.DEFAULT_HEX_MESSAGE

    def set_motor_speed(self, desired_speed):
        hex_message = self.encode_speed_msg_to_bytes(desired_speed)
        ret = self.send_hex_message(hex_message)
        return ret

    def disable_motor(self):
        hex_message = self.encode_speed_msg_to_bytes(0, False)
        ret = self.send_hex_message(hex_message)
        return ret

    def decode_controller_response(self, ret):
        # Convert string to byte array
        byte_array = bytes.fromhex(ret)
        
        # First ybte checks if message was valid
        if byte_array[0]==0xAC:
            if self.debug: print("Check passed!")
        elif byte_array[0]==0xA8:
            if self.debug: print("Check failed")

        # Two bytes for checking speed
        # TODO: convert hexadecimal speed to negative for opposite direction
        speed = int(byte_array[1] + byte_array[2]) 

        # Two bytes for checking motor current
        # TODO: convert hexadecimal current to negative for opposite direction
        current = int(byte_array[3] + byte_array[4]) 

        return speed, current

    def set_motor_speed_and_read_feeback(self, desired_rpm):
        ret = self.set_motor_speed(desired_rpm)
        speed, current = self.decode_controller_response(ret)
        return speed, current

# Example usage:
if __name__ == "__main__":
    import time

    # Init Steering Driver and serial port communication
    steering_driver = SerialDriver()
    steering_driver.open_serial_port()

    start_time = time.time()
    elapsed_time = 0
    while elapsed_time<5:
        # Send the hexadecimal message
        ret = steering_driver.set_motor_speed(500)
        speed, current = steering_driver.decode_controller_response(ret)
        elapsed_time = time.time()-start_time
        print(f"Speed: {speed}, Current: {current}, Elapsed time: {elapsed_time:.3f}")
    
    # Close the serial port
    steering_driver.close_serial_port()
