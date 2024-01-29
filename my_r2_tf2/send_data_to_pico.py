import serial 
import time

serial_port = '/dev/ttyACM0'
baud_rate = 115200  

with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    # Open serial port
    time.sleep(2) 

    # Define floats to send
    float1 = 82.2321
    float2 = 8221.5621
    float3 = 71821.8928
    float4 = 1210.2121

    # Convert to bytes
    data = (str(float1) + '|' + 
            str(float2) + '|' +
            str(float3) + '|' +
            str(float4)) + "#"
    
    # Send data
    ser.write(data.encode())  
    print(f"Sent: {data}")