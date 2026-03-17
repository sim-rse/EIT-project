from statistics import median

import serial
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

lijst_raw = []
lijst_filtered = []

lijst_filtered_2 = []
lijst_filtered_intervals = []

lines_per_gemiddelde = 50

baseline = None
baseline_check = []
baseline_teller = 0
baseline_done = False

teller = 0
gemiddelde_lijst = []
glob_gemiddelden = []
check_lines = 50
spier_gespannen = False
drempel = 50
spier_lijst = []

printsnelheid = 10

FS = int(1000/printsnelheid)
BAUD = 444444
PORT = 'COM6'

b_notch, a_notch = signal.iirnotch(50.0, 30.0, FS)

zi_notch = signal.lfilter_zi(b_notch, a_notch) * 0

ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
if ser.is_open:
    print(f"Connectie geslaagd met {PORT} @ {BAUD} baud")
else:
    print("Connectie gefaald")
    exit()


ser.write(f's200\n'.encode('ascii'))

print("Bezig met filteren. Ctrl+F2 om te stoppen.")

try:
    while True:
        teller += 1
        baseline_teller += 1
        line = ser.readline().decode('utf-8').strip()
        sub = "spierkracht: "

        if not line:
            continue

        try:
            if sub in line:
                part = line.split(sub)[1]
                waarde_spierkracht = part.split(",")[0].strip()
                raw_value = float(waarde_spierkracht)

                notch_out, zi_notch = signal.lfilter(b_notch, a_notch, [raw_value], zi=zi_notch)
                notch_out = notch_out[0]

                if baseline_teller < 5*1000/printsnelheid:
                    if not type(notch_out) == list:
                        baseline_check.append(notch_out)
                elif not baseline_done:
                    baseline = median(baseline_check)
                    baseline_done = True
                    print("")
                    print("")
                    print("")
                    print("")
                    print("")
                    print("baseline inputted: ", baseline)
                    print("")
                    print("")
                    print("")
                    print("")
                    print("")

                print(f"Raw: {raw_value:<8} | Filtered: {notch_out:.2f}")
                lijst_raw.append(raw_value)
                lijst_filtered.append(notch_out)

                if teller < check_lines:
                    if not isinstance(notch_out, list):
                        gemiddelde_lijst.append(notch_out)
                else:
                    ogenbl_gemiddelde = sum(gemiddelde_lijst)/len(gemiddelde_lijst)
                    if not baseline is None:
                        if ogenbl_gemiddelde < baseline - drempel  or baseline + drempel < ogenbl_gemiddelde:
                            spier_gespannen = True
                            spier_lijst.append("True")
                        else:
                            spier_gespannen = False
                            spier_lijst.append("False")
                    teller = 0



        except ValueError:
            continue

except KeyboardInterrupt:
    for dirty in lijst_filtered:
        if not isinstance(dirty, list):
            lijst_filtered_2.append(abs(dirty))

    averages_all = [sum(lijst_filtered_2[i:i + lines_per_gemiddelde]) / len(lijst_filtered_2[i:i + lines_per_gemiddelde]) for i in range(0, len(lijst_filtered_2), 4)]

    x0 = []
    for i in range(len(lijst_raw)):
        x0.append(i)
    plt.plot(x0, lijst_raw, label='Raw')
    plt.ylim(-800, 800)

    plt.legend()
    plt.show()


    x1 = []
    for i in range(len(lijst_filtered)):
        x1.append(i)
    plt.plot(x1, lijst_filtered, label='Filtered')
    plt.ylim(-800, 800)

    plt.legend()
    plt.show()


    x2 = []
    for j in range(len(averages_all)):
        x2.append(j)
    plt.plot(x2, averages_all, label='Gemiddelde')
    plt.ylim(-800, 800)
    plt.axhline(y=baseline, color='red', linestyle='--')

    plt.legend()
    plt.show()


    x3 = []
    for k in range(len(lijst_filtered_2)):
        x3.append(k)
    plt.plot(x3, lijst_filtered_2, label='Absolute waarden')
    plt.ylim(-800, 800)

    plt.legend()
    plt.show()

    print(spier_lijst)
    print("baseline = ", baseline)



finally:
    if ser.is_open:
        ser.close()
        print("Poort gesloten.")
