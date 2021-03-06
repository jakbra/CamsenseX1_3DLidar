# Camsense X1 3D Lidar

The goal of this project was to upgrade a low cost 2D LIDAR to a 3D LIDAR system, using an additional motor. The motor would tilt the LIDAR and knowing the angle of the tilt we can determine the 3-dimensional coordinates of the objects that the LIDAR measures. Such a system requires a special construction, which would be constructed using the LEGO Mindstorm framework. The motor used for tilting is part of the same LEGO framework. A low-cost ESP32 microcontroller is used for controlling the whole system. At the end of the paper the developed system was evaluated and shows promising results despite its cost.

<p align="center">
<img src="./images/Konstrukcija4.jpg" width="256"/>
<p align="center">
  
### Error evaluation
The accuracy of the coordinate measurments (theta, phi, r), was experimentaly measured. The variance of the individual measurmets is used to compute the covariance matrix, which is then represented as an three dimesional confidence elipsoid of the scaned points.
  
<p align="center">
<img src="./images/coordinates.jpg" width="200"/>
<p align="center">
  
### Executing program

ESP runs the cpp code and is conected to the PC via USB port.
The data from the scan is sent to the PC after PIN 23 activation.
saveData.py must be running. A csv file (data.csv) is created.

Run showPoints.py to plot the points of the scan.

Run showElips.py to represent each point as an error elipsoid.
  
##Results
  
<p align="center">
<img src="./images/ScatterAndElipse.png" width="720"/>
<p align="center">

## Authors

Jakob Bračun

bracun.jakob@gmail.com


## Acknowledgments

* [Vidicon](https://github.com/Vidicon/camsense-X1)
* [yishii](https://github.com/yishii/LiDAR_Camsense_X1_M5Stack)
* [j-fujimoto](https://github.com/j-fujimoto/CamsenseX1)
* [roundsToThree](https://github.com/roundsToThree/Camsense-X1-Previewer)

Discord:
https://discord.gg/zRGJcqa





