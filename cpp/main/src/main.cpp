#include <ESP32Encoder.h>
#include <Arduino.h>
#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// HARDWARE

//Setting LIDAR
#define LIDAR_TX 27 //LIDAR serial povezava na pinu 27
#define _USE_MATH_DEFINES
HardwareSerial HardwareLaser(2);

//Buttons
const int button = 23;

// Motor
// Izhodi, ki se povežejo na H-mostič (Levo, desno, hitrost)
int MotorPin1 = 18;                 
int MotorPin2 = 19;                   
int enable = 14;

// Encoder
// Ustvarimo objekt "encoder"
// error in angleCounter, globane spemenljivke za regulacijo zasuka motorja
ESP32Encoder encoder;
int error;
int32_t angleCounter;


//////////////////////////////////////////////////////////////////////////////////
// Setting PWM properties
// Spremenljike, ki definirajo lastnosti PWM pina, določenega v void setup()

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 60;

//////////////////////////////////////////////////////////////////////////////////
// Timer interrupt
// Narejen po primeru "https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/"

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//////////////////////////////////////////////////////////////////////////////////
// PAKETI

// Header: 4 bajti, ki so vedno na začetku paketa
const uint8_t header[] = { 0x55, 0xaa, 0x03, 0x08 };

// Struktura paketa
typedef struct {
  uint8_t header0;
  uint8_t header1;
  uint8_t header2;
  uint8_t header3;
  uint16_t rotation_speed; // rpm*64
  uint16_t angle_begin; // (angle+640)*64
  uint16_t distance_0;
  uint8_t reserved_0;
  uint16_t distance_1;
  uint8_t reserved_1;
  uint16_t distance_2;
  uint8_t reserved_2;
  uint16_t distance_3;
  uint8_t reserved_3;
  uint16_t distance_4;
  uint8_t reserved_4;
  uint16_t distance_5;
  uint8_t reserved_5;
  uint16_t distance_6;
  uint8_t reserved_6;
  uint16_t distance_7;
  uint8_t reserved_7;
  uint16_t angle_end;  //(angle+640)*64
  uint16_t crc;
} __attribute__((packed)) LidarPacket_t;

// Stanja, ki se uporabljajo pri razčljenjevanju paketov
typedef enum
{
  STATE_WAIT_HEADER = 0,
  STATE_READ_HEADER,
  STATE_READ_PAYLOAD,
  STATE_READ_DONE
} State_t;

//////////////////////////////////////////////////////////////////////////////////
// State variables
// Spremenljivke definirajo stanje programa (merimo || mirujemo)
volatile int flag = 0;
volatile bool startMove = false;

bool measure = false;

typedef enum
{
  STATE_0 = 0,
  STATE_1,
} measureState_t;

int dataCounter = 0;

//////////////////////////////////////////////////////////////////////////////////
// covert funkcije

// Ker je zapis začetnega in končnega kota v bajtih oblike: (angle+640)*64
// ga s to funkcijo pretvorimo
uint16_t convertDegree(uint16_t input)
{
  
  return (input - 40960) / 64;
  
}

// Ker je zapis rotacijske hitrosti v bajtih oblike: rpm*64
// ga s to funkcijo pretvorimo
uint16_t convertSpeed(uint16_t input)
{
  return input / 64;
}

// Funkcija vzame začetni in končni kot, ter razliko razdeli na 8 enakih delov,
// dele zapiše v seznam "map", upošteva se tudi situacijo ko je sta kota na prelomu kroga (360 -> 0 stopinj)
void remapDegrees(uint16_t minAngle, uint16_t maxAngle, uint16_t *map)
{
  int16_t delta = maxAngle - minAngle;
  if (maxAngle < minAngle) {
    delta += 360;
  }

  if ((map == NULL) || (delta < 0)) {
    return;
  }
  for (int32_t cnt = 0; cnt < 8; cnt++)
  {
    map[cnt] = minAngle + (delta * cnt / 7);
    if (map[cnt] >= 360) {
      map[cnt] -= 360;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
// Timer interupt funkcija
// Narejen po primeru "https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/"
// Funkcija, ki se izvaja ob stalnih časovnih intervalih, v tem primeru je to premikanje motorja med dvema skrajnima legama.
// 
// Smer vrtenja določa spremenljivka "flag" -> case 0: željeni kot = 90; case 1: željeni kot = 0 -> 
// ko je error = 0 se zamenjata
// 
// Ker želimo, da se motor premika samo ko smo v fazi merjenja je premikanje pogojeno z "startMove"

Ker želimo da se motor vrti 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  int ref;

  if(startMove){
  switch (flag) {
  case 0:
  ref = 90;


  error = ref - (int32_t)encoder.getCount(); // (int32_t)encoder.getCount() nam vrne število preštetih pulzov enkoderja

  // Vrtenje CW
  if (error > 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
  }
  // Vrtenje CCW 
  else if (error < 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
  }
  // Mirovanje & spremeni se "flag"
  else if (error == 0){
    ledcWrite(pwmChannel, dutyCycle);
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, HIGH);
    flag = 1;
 
  }
    break;

  // Enaka zgodba, zeljen zasuk je sedaj 0 stopinj
  case 1:
  ref = 0;

  error = ref - (int32_t)encoder.getCount();

  if (error > 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
  }
  else if (error < 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
  }
  else if (error == 0){
    ledcWrite(pwmChannel, dutyCycle);
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, HIGH);
    flag = 0;
 
  }
    break;
  default:
    break;
}
}

  portEXIT_CRITICAL_ISR(&timerMux);
 
}


//////////////////////////////////////////////////////////////////////////////////
// Računanje koordinat in členov matrike
// pošiljanje po serijskem kanalu

void calcXY(uint16_t* degrees, uint16_t* distances){


	int8_t i;
  int16_t x, y, z;
  int16_t theta;

  // Variance določene med evalvacijo
  int16_t var_phi = 0.563;
  int16_t var_theta = 0.563;
  int16_t var_r = 0.8842;

  // Števec prebranih paketov, željeno število se določi v void loop()
  dataCounter = dataCounter + 1;


  for(i=0;i<8;i++){

      // Sledeči "if" stavki, določijo varianco (var_r) in pristranskost (e) meritve razdalje, na podlagi linerane 
      // interpolacije med izmerjenimi vrednostmi. Enačbe premic so bile izračunane ročno

      // Meritvam manjšim od 525 se določi začetna pristraskost in varianca
      if(distances[i]<=525){
        int16_t e;
        e = -0.281;
        distances[i] = distances[i]-e; //Pristranskost odštejemo od meritve
        var_r = 0.8842;
      }


      if(distances[i]<=1010){
        int16_t e;
        e = -0.0014*distances[i] + 0.4720; //Enačba premice
        distances[i] = distances[i]-e; //Pristranskost odštejemo od meritve
        var_r = 0.0058*distances[i] - 2.1505; //Enačba premice
      }

      if(distances[i]<=1490){
        int16_t e;
        e = -0.0278*distances[i] + 27.0988;
        distances[i] = distances[i]-e;
        var_r = 0.4728 *distances[i] - 473.7924;
      }

      if(distances[i]<=1975){
        int16_t e;
        e = -0.0966*distances[i] + 129.5955;
        distances[i] = distances[i]-e;
        var_r = -0.2259 *distances[i] + 567.1712;
      }

      if(distances[i]<=2545){
        int16_t e;
        e = 0.0565*distances[i] - 172.8231;
        distances[i] = distances[i]-e;
        var_r = 0.5681*distances[i] - 1000.9;
      }

      if(distances[i]<=3145){
        int16_t e;
        e = -0.0277*distances[i] + 41.6056;
        distances[i] = distances[i]-e;
        var_r = 0.2749*distances[i] - 254.7709;
      }

      if(distances[i]<=3746){
        int16_t e;
        e = -0.0031*distances[i] - 35.9142;
        distances[i] = distances[i]-e;
        var_r = 0.5215*distances[i] - 1030.2;
      }

      if(distances[i]<=4345){
        int16_t e;
        e = 0.0490*distances[i] - 230.9502;
        distances[i] = distances[i]-e;
        var_r = -0.9832*distances[i] + 4606.2;
      }

      if(distances[i]<=4950){
        int16_t e;
        e = 0.0527*distances[i] -247.1994;
        distances[i] = distances[i]-e;
        var_r = 0.8636*distances[i] - 3418.2;
      }

      if(distances[i]<=5550){
        int16_t e;
        e = -0.0391*distances[i] +207.2590;
        distances[i] = distances[i]-e;
        var_r = 0.1112*distances[i] + 306.2055;
      }

       if(distances[i]<=6280){
        int16_t e;
        e = 0.0651*distances[i] -371.1415;
        distances[i] = distances[i]-e;
        var_r = 7.9983*distances[i] - 43467;
      }

      if(distances[i] > 6280){
        int16_t e;
        e = 37.9377;
        distances[i] = distances[i]-e;
        var_r = 6762.3;

      }

      // Kot nagiba theta
      // 15 pulzov je 10 stopinj 10/15 = 0,66
      theta = (int32_t)encoder.getCount() * 0.66; 
    
      // Izračun členov kovariančne matrike
      int E11 = var_phi*sq(distances[i])*sq(-sin((1.f * PI * degrees[i]) / 180)) + var_r*sq(cos((1.f * PI * degrees[i]) / 180));
      int E12 = - var_phi*sin((1.f * PI * degrees[i]) / 180)*cos((1.f * PI * degrees[i]) / 180) + var_r*sin((1.f * PI * degrees[i]) / 180)*cos((1.f * PI * degrees[i]) / 180);
      int E13 = var_r*sin((1.f * PI * theta) / 180)*cos((1.f * PI * degrees[i]) / 180);

      int E21 = - var_phi*sin((1.f * PI * degrees[i]) / 180)*cos((1.f * PI * degrees[i]) / 180)*sq(distances[i]) + var_r*sin((1.f * PI * degrees[i]) / 180)*cos((1.f * PI * degrees[i]) / 180);
      int E22 = var_phi*sq(distances[i])*sq(cos((1.f * PI * degrees[i]) / 180)) + var_r*sq(sin((1.f * PI * degrees[i]) / 180));
      int E23 = var_r*sin((1.f * PI * theta) / 180)*sin((1.f * PI * degrees[i]) / 180);

      int E31 = var_r*sin((1.f * PI * theta) / 180)*cos((1.f * PI * degrees[i]) / 180);
      int E32 = var_r*sin((1.f * PI * theta) / 180)*sin((1.f * PI * degrees[i]) / 180);
      int E33 = var_theta*sq(distances[i])*sq(cos((1.f * PI * theta) / 180)) + var_r*sq(sin((1.f * PI * theta) / 180));

      // Izračun x,y in z koordinat
      x = cos((1.f * PI * degrees[i]) / 180) * (distances[i]);
      y = sin((1.f * PI * degrees[i]) / 180) * (distances[i]);
      z = -sin((1.f * PI * theta) / 180) * (distances[i]);
     
      // Pošiljanje podatkov po serijskem kanalu
      Serial.print("x");
      Serial.print(x);
      Serial.print("\n");
      Serial.print("y");
      Serial.print(y);
      Serial.print("\n");
      Serial.print("z");
      Serial.print(z);  
      Serial.print("\n");

      // Nesmiselni znaki za označevanje podatkov se uporabljajo, ker je prišlo do error-jev pri uporabi nekaterih znakov
      // Pombno, da se ujemajo z znaki v "saveData.py"
      Serial.print("a");
      Serial.print(E11);
      Serial.print("\n");
      Serial.print("b");
      Serial.print(E12);
      Serial.print("\n");
      Serial.print("p");
      Serial.print(E13);
      Serial.print("\n");

      Serial.print("d");
      Serial.print(E21);
      Serial.print("\n");
      Serial.print("k");
      Serial.print(E22);
      Serial.print("\n");
      Serial.print("f");
      Serial.print(E23);
      Serial.print("\n");

      Serial.print("*");
      Serial.print(E31);
      Serial.print("\n");
      Serial.print("?");
      Serial.print(E32);
      Serial.print("\n");
      Serial.print("+");
      Serial.print(E33);
      Serial.print("\n");

  }
}

//////////////////////////////////////////////////////////////////////////////////
// Funkcija za razčljenjevanje LIDAR paketov
// Po razčlenjenju se kliče funkcija calcXY()
// Podrobnejši opis v delovnem poročilu

void lidar(){
  
  static State_t state;
  static uint32_t counter;
  static uint8_t payload[64];

if (HardwareLaser.available()) {
  
    uint8_t data = HardwareLaser.read();
    switch (state)
    {
      case STATE_WAIT_HEADER:
        if (data == header[0]) {
          counter++;
          payload[0] = data;
          state = STATE_READ_HEADER;
        } else {
          HardwareLaser.flush();
        }
        break;
      case STATE_READ_HEADER:
        if (data == header[counter]) {
          payload[counter] = data;
          counter++;
          if (counter == sizeof(header)) {
            state = STATE_READ_PAYLOAD;
          }
        } else {
          counter = 0;
          state = STATE_WAIT_HEADER;
        }
        break;
      case STATE_READ_PAYLOAD:
        payload[counter] = data;
        counter++;
        if (counter == sizeof(LidarPacket_t)) {
          state = STATE_READ_DONE;
        }
        break;
      case STATE_READ_DONE:
        LidarPacket_t* packet = (LidarPacket_t*)payload;
        {
          uint16_t degree_begin;
          uint16_t degree_end;
          degree_begin = convertDegree(packet->angle_begin);
          degree_end = convertDegree(packet->angle_end);
          if ((degree_begin < 360) && (degree_end < 360)) {
            uint16_t map[8];
            uint16_t distances[8];
            remapDegrees(degree_begin, degree_end, map);
            distances[0] = packet->distance_0;
            distances[1] = packet->distance_1;
            distances[2] = packet->distance_2;
            distances[3] = packet->distance_3;
            distances[4] = packet->distance_4;
            distances[5] = packet->distance_5;
            distances[6] = packet->distance_6;
            distances[7] = packet->distance_7;
            calcXY(map,distances);
        }

        counter = 0;
        state = STATE_WAIT_HEADER;
        break;
    }
  
  }

  }
}

// Funkcija za regulacijo motorja, ki pa se uporablja samo ko LIDAR miruje -> measureState = STATE_1
void motorRegulator(int refAngle){

  int dutyCycle = 60;

  while(true){

  angleCounter = (int32_t)encoder.getCount();
  int error = refAngle - angleCounter;

  if (error > 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
  }
  else if (error < 0){

    ledcWrite(pwmChannel, dutyCycle); 
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
  }
  else{
    ledcWrite(pwmChannel, dutyCycle);
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, HIGH);
    return;
  }
  }

}

//////////////////////////////////////////////////////////////////////////////////

void setup() {

  // Inicializacija serijskih povezav
  Serial.begin(115200);
  HardwareLaser.begin(115200, SERIAL_8N1, LIDAR_TX, -1);

  // Določitev pinov
  pinMode(PIN_BUTTON, INPUT_PULLDOWN);
  pinMode(button, INPUT_PULLDOWN);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(enable, OUTPUT);
  
  // Določitev PWM pina
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable, pwmChannel);

  // Enkoder šteje pulze na pinih 32 in 25
  ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder.attachHalfQuad(32, 25);
	encoder.setCount(0);

  // Lastnosti Timer Interupt funkcije
  // Narejen po primeru "https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/"
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

}

void loop(){

  static measureState_t measureState;

  int button_state = 1;
  dutyCycle = 100;

  button_state = digitalRead(button);


 // Ob pritisku na button, preidemo v stanje "measure"
 if(button_state == LOW){
      measure = true;
    }


  if(measure){
  
  delay(1000);

  //Definirana dva stanja
  switch (measureState)
    {
      // STATE_0 
      case STATE_0:
        // Inicializiramo števec pulzov
        encoder.setCount(0);

        // Določimo željeno število prebranih paketov -> dataCounter
        while(dataCounter < 1000){
          // LIDAR se začne premikati (Timer Iterupt funkcija)
          startMove = true;

          // Začne se razčlenjevanje paketov ter računanje koordinat in členov matrik, ter pošiljanje po serijskem kanalu
          lidar();
        }

        //Ustavimo premikanje
        startMove = false;
        measureState = STATE_1;
      
      // STATE_1
      case STATE_1:
      // Sistem miruje
      // dataCounter se ponastavi
      // čakamo na ponoven pritisk button
        motorRegulator(0);
        digitalWrite(MotorPin1, HIGH);
        digitalWrite(MotorPin2, HIGH);
        dataCounter = 0;
        measureState = STATE_0;
        break;
    }
    
  measure = false;

  }

}