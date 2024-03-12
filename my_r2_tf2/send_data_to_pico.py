import serial 
import time

serial_port = '/dev/ttyACM0'
baud_rate = 115200   
ser = serial.Serial(serial_port, baud_rate, timeout=1)


class trying:
        def __init__(self):
                while True:
                        trying.call_back(ser)

        @staticmethod
        def call_back(ser):
                # Open serial port
                # time.sleep(0.5) 
                at = time.time()
                print("Start",at)
                # Define floats to send
                float1 = int(82.2321)
                float2 = int(221.5621)
                float3 = int(121.8928)
                float4 = int(210.2121)

                # Convert to bytes
                data = (str(float1) + '|' + 
                        str(float2) + '|' +
                        str(float3) + '|' +
                        str(float4)) + "#"
                
                # Send data
                ser.write(data.encode())  
                print(f"Sent: {data}")
                print("End",time.time())
                print("Time taken", time.time() - at)


if __name__ == "__main__":
        obj = trying()
