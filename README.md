# Camsense X1 3D Lidar

## Getting Started



### Dependencies

* ESP32Encoder.h https://github.com/madhephaestus/ESP32Encoder

### Executing program

Na mikrokrmilnik se naloži main.cpp. Mikrokrmilnik je preko serijeske povezave povezan na PC.

Na PC-ju se zažene saveData.py. 

Po začetku meritev (button = PIN 23), mikrokrmilnik začne pošiljati
podatke.

saveData.py ustvari data.csv

Z showPoints.py preberemo data.csv in prikaženo točke v prostoru.

Z showElips.py preberemo data.csv in prikažno elipse v prostoru.


## Help

showElips.py ne deluje hitro. Grafa se po izrisu ne da premikati. (Vsaj na mojem PC-ju)

## Authors

Jakob Bračun

bracun.jakob@gmail.com


## Acknowledgments

Največji repozitorij:
* [Vidicon](https://github.com/Vidicon/camsense-X1)

Še nekaj uporabnih repositorijev:
* [yishii](https://github.com/yishii/LiDAR_Camsense_X1_M5Stack)
* [j-fujimoto](https://github.com/j-fujimoto/CamsenseX1)
* [roundsToThree](https://github.com/roundsToThree/Camsense-X1-Previewer)

Discord:
https://discord.gg/zRGJcqa





