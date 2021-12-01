import matplotlib.pyplot as plt
import csv

x = []
y = []
z = []

Ex = []
Ey = []
Ez = []

e11 = []
e12 = []
e13 = []

e21 = []
e22 = []
e23 = []

e31 = []
e32 = []
e33 = []

tmpx = []
tmpy = []
tmpz = []

mu =[]


with open('data.csv', 'r') as csvfile:
    datareader = csv.reader(csvfile,delimiter=',')
    for row in datareader:
        # Preberemo csv datoteko
        # dobimo sezname x,y,z
        # ter seznam členov kovariančne matrike po vrsticah Ex,Ey,Ez
        x.append(int(float(row[0])))
        y.append(int(float(row[1])))
        z.append(-int(float(row[2])))
        tmpx.append(int(float(row[3])))
        tmpx.append(int(float(row[4])))
        tmpx.append(int(float(row[5])))
        Ex.append(tmpx)
        tmpy.append(int(float(row[6])))
        tmpy.append(int(float(row[7])))
        tmpy.append(int(float(row[8])))
        Ey.append(tmpy)
        tmpz.append(int(float(row[9])))
        tmpz.append(int(float(row[10])))
        tmpz.append(int(float(row[11])))
        Ez.append(tmpz)
        tmpx = []
        tmpy = []
        tmpz = []


#Za lepši prikaz odstranimo točki, ki so zelo oddaljene
for i in range(len(x)):
    if abs(x[i]) > 4000:
        x[i] = 0
        y[i] = 0
        z[i] = 0

for i in range(len(y)):
    if abs(y[i]) > 4000:
        x[i] = 0
        y[i] = 0
        z[i] = 0

for i in range(len(y)):
    if abs(z[i]) > 3000:
        x[i] = 0
        y[i] = 0
        z[i] = 0



# Setup the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(-90,-30)
ax.view_init(elev=44, azim=9)
plt.xlabel('x')
plt.ylabel('y')


color = list()
for elem in z:
    color.append(-elem)

ax.scatter3D(x, y, z, c = color,  cmap='winter')

plt.show()