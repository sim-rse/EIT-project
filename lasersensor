import serial
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

FS = 500.0
BAUD = 444444
PORT = 'COM6'



#Notch filter
b_notch, a_notch = signal.iirnotch(50.0, 30.0, FS)

#Bandpass filter
b_band, a_band = signal.butter(4, [20, 200], btype='bandpass', fs=FS)

zi_notch = signal.lfilter_zi(b_notch, a_notch) * 0
zi_band = signal.lfilter_zi(b_band, a_band) * 0

ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
if ser.is_open:
    print(f"Connectie geslaagd met {PORT} @ {BAUD} baud")
else:
    print("Connectie gefaald")
    exit()


ser.write(f's200\n'.encode('ascii'))

print("Bezig met filteren. Ctrl+F2 om te stoppen.")

threshold = 350

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        sub = "laser: "

        if not line:
            continue

        try:
            if sub in line:
                part = line.split(sub)[1]
                waarde_laser = part.split(",")[0].strip()
                final_waarde_laser = float(waarde_laser)
                print(final_waarde_laser)
                if final_waarde_laser < threshold:
                    print("laser detected")
                    drive = True
                else:
                    print("laser not detected")
                    drive = False

        except ValueError:
            continue

except KeyboardInterrupt:
    exit()

finally:
    if ser.is_open:
        ser.close()
        print("Poort gesloten.")
