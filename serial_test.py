import serial
import time

# Define the serial port and baud rate
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)  # Replace 'COM1' with your serial port

# Hexadecimal message to send
#hex_message = bytearray([0xAD, 0x03, 0xE8, 0x98])
hex_message = bytearray([0xAD, 0xFC, 0x18, 0xC1])
#hex_message = bytearray([0xE8, 0x00, 0x00, 0x00])

try:
    # Send the hexadecimal message
    start = time.time()
    while time.time()-start<5:
        ser.write(hex_message)
        print(f"Sent: {hex_message.hex()}, Elapsed time: {time.time()-start}")
        # Read and print the response
        response = ser.read(4)  # Adjust the number of bytes to read as needed
        print(f"Received: {response.hex()}")

finally:
    # Close the serial port
    ser.close()