import serial
from tempfile import TemporaryFile
import csv
import pandas as pd


ser = serial.Serial(
    port='COM3',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

x_vals = []
y_vals = []
z_vals = []

exx = []
exy = []
exz = []

eyx = []
eyy = []
eyz = []

ezx = []
ezy = []
ezz = []

# Dokler ni v seznamu vseh podatkov (dataCounter * 8) - Beremo iz serijske povezave
while len(z_vals) < 8000:

    # Preberemo vratico in jo dekodiramo 
    tmp = ser.readline()
    tmp = tmp.decode('ascii')
    try:
        # Prvi znak v vrstici nam pove za kater podatek gre
        tip = tmp[0]
        
        # Vse kar pride za prvim znakom je podatek (tmp[1:])
        if tip == 'x':
            x = int(float(tmp[1:]))
            x_vals.append(x)
        if tip == 'y':
            y = int(float(tmp[1:]))
            y_vals.append(y)
        if tip == 'z':
            z = int(float(tmp[1:]))
            z_vals.append(z)

        if tip == 'a':
            e11 = int(float(tmp[1:]))
            exx.append(e11)
        if tip == 'b':
            e12 = int(float(tmp[1:]))
            exy.append(e12)
        if tip == 'p':
            e13 = int(float(tmp[1:]))
            exz.append(e13)

        if tip == 'd':
            e21 = int(float(tmp[1:]))
            eyx.append(e21)
        if tip == 'k':
            e22 = int(float(tmp[1:]))
            eyy.append(e22)
        if tip == 'f':
            e23 = int(float(tmp[1:]))
            eyz.append(e23)

        if tip == '*':
            e31 = int(float(tmp[1:]))
            ezx.append(e31)
        if tip == '?':
            e32 = int(float(tmp[1:]))
            ezy.append(e32)
        if tip == '+':
            e33 = int(float(tmp[1:]))
            ezz.append(e33)


    # Preden LIDAR začne pošiljat pakete, prihaja do errorjev
    # ze lepši prikaz samo izpišemo vrsto errorja    
    except IndexError:
        print("IE")
    except UnicodeDecodeError:
        print("UDE")


# Podatke zdruzimo in jih shranimo v csv datoteko
rows = zip(x_vals,y_vals,z_vals,exx,exy,exz,eyx,eyy,eyz,ezx,ezy,ezz)

with open('data.csv', "w") as f:
    writer = csv.writer(f)
    for row in rows:
        writer.writerow(row)

df = pd.read_csv('data.csv')
df.to_csv('data.csv', index=False)



        
