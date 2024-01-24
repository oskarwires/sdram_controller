import serial
import time

BurstLength = 4

# Open the serial port
ser = serial.Serial('COM4', 115200, timeout=0.1)

def write_ram(bank, col, row, value):
    if bank < 0 or bank > 1 or col < 0 or col > 7 or row < 0 or row > 7:
        raise ValueError("Invalid input values. Bank, col, and row must be within their respective ranges.")
    if value >= 255:
        raise ValueError("Can't send a value over 255 (8 bits)")
    if value < 0:
        raise ValueError("Can only send unsigned (positive) values")
    addr = (bank << 6) | (col << 3) | row
    bytes_to_send = [ord('w'), addr, value]
    ser.write(bytearray(bytes_to_send))
    time.sleep(0.1)
    print(f"Write: {value} to bank {bank} col {col} row {row} (addr: {addr})")

def read_ram(bank, col, row, BurstLength):
    if bank < 0 or bank > 1 or col < 0 or col > 7 or row < 0 or row > 7:
        raise ValueError("Invalid input values. Bank, col, and row must be within their respective ranges.")
    addr = (bank << 6) | (col << 3) | row
    bytes_to_send = [ord('r'), addr]
    ser.write(bytearray(bytes_to_send))
    time.sleep(0.1)
    
    responses = []

    for _ in range(BurstLength):
        time.sleep(0.01)
        # Read exactly one byte
        response = ser.read(1)
        if response:
            decimal_value = int.from_bytes(response, byteorder='big')
            responses.append(decimal_value)
        else:
            responses.append("No response")

    for value in responses:
            print(f"Read: {value} at bank {bank} col {col} row {row} (addr: {addr})")



    
ser.flushInput()
ser.flushOutput()
write_ram(0,0,1,143) # bank, col, row
write_ram(1,0,2,102)
write_ram(0,2,0,2)
read_ram(0, 0, 1, BurstLength);
read_ram(1, 0, 2, BurstLength);
read_ram(0, 2, 0, BurstLength);
time.sleep(1);
read_ram(1, 0, 2, BurstLength);
time.sleep(1);
read_ram(0, 0, 1, BurstLength);


# Close the serial port
ser.close()
