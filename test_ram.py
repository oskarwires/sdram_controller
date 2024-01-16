import serial
import time

# Open the serial port
ser = serial.Serial('COM4', 115200, timeout=1)

def write_ram(addr, value):
    if value >= 255:
        raise ValueError("Can't send a value over 255")
    bytes_to_send = [ord('w'), addr, value]
    ser.write(bytearray(bytes_to_send))
    time.sleep(0.01)

def read_ram(addr):
    bytes_to_send = [ord('r'), addr]
    ser.write(bytearray(bytes_to_send))
    time.sleep(0.01)
    response = ser.read(ser.inWaiting())
    if response:
      # Convert the first byte to an integer
      decimal_value = int.from_bytes(response, byteorder='big')
      return decimal_value
    else:
      return "No response"
    

write_ram(2,143)
write_ram(13,102)
write_ram(3,2)
value_1 = read_ram(2)
value_2 = read_ram(13)
value_3 = read_ram(3)
print(f"Value at addr 2 is: {value_1}")
print(f"Value at addr 13 is: {value_2}")
print(f"Value at addr 3 is: {value_3}")

# Close the serial port
ser.close()
