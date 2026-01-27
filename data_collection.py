import serial
import keyboard
import csv
import time

ARDUINO_PORT = 'COM3'  # Change to match Arduino
BAUD_RATE = 9600

print("=== Posture Data Collection ===")

try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=2)
    time.sleep(2)
    print(f"Connected to {ARDUINO_PORT}\n")
except:
    print(f"Can't connect to {ARDUINO_PORT}")
    exit()

filename = f"posture_{time.strftime('%Y%m%d_%H%M%S')}.csv"
f = open(filename, 'w', newline='')
writer = csv.writer(f)
writer.writerow(['timestamp', 'angle', 'gyro', 'state'])

current_state = 0
count = 0

print("Controls: 1=Upright, 2=Slouching, 3=Transient, Q=Quit")
print("Recording...\n")

try:
    while True: 
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            
            try:
                timestamp, angle, gyro = line.split(',')
                
                if keyboard.is_pressed('1'):
                    if current_state != 0:
                        current_state = 0
                        print("→ Upright")
                    time.sleep(0.2)
                elif keyboard.is_pressed('2'):
                    if current_state != 1:
                        current_state = 1
                        print("→ Slouching")
                    time.sleep(0.2)
                elif keyboard.is_pressed('3'):
                    if current_state != 2:
                        current_state = 2
                        print("→ Transient")
                    time.sleep(0.2)
                elif keyboard.is_pressed('q'):
                    print("\nStopping...")
                    break 
                
                writer.writerow([timestamp, angle, gyro, current_state])
                count += 1
                
                if count % 100 == 0:
                    print(f"  {count} readings...")
                    
            except:
                pass
                
except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    f.close()
    ser.close()
    print(f"Saved {count} readings to {filename}")
