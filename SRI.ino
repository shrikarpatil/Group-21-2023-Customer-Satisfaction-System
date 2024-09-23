#include <Wire.h>
#include <dht.h>
#include <TMRpcm.h>          
#include <SPI.h>
#include <SD.h> 
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal.h>
#include <IRLibSendBase.h>    
#include <IRLib_HashRaw.h>    

IRsendRaw mySender;
#define SD_ChipSelectPin 53
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true

dht DHT;

#define DHT11_PIN 16


LiquidCrystal lcd(48,46,44,42,40,38);
byte gammatable[256];


int ir =0 ;


float C, F; 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
TMRpcm tmrpcm; 

#define RAW_DATA_LEN 348
uint16_t rawDataOn[RAW_DATA_LEN] = {
  2990, 8986, 486, 498, 482, 1538, 482, 502, 
	482, 502, 482, 502, 482, 498, 482, 502, 
	482, 502, 482, 502, 482, 1538, 482, 498, 
	482, 502, 482, 1538, 482, 502, 482, 502, 
	482, 1534, 486, 1534, 482, 1538, 482, 1538, 
	482, 1538, 482, 498, 486, 502, 482, 498, 
	486, 498, 486, 498, 482, 502, 482, 502, 
	482, 502, 482, 498, 486, 498, 486, 498, 
	482, 502, 482, 502, 482, 502, 482, 502, 
	482, 498, 486, 498, 486, 498, 482, 502, 
	482, 498, 486, 498, 482, 502, 482, 502, 
	482, 502, 482, 498, 486, 498, 482, 502, 
	482, 502, 482, 502, 482, 498, 486, 498, 
	486, 498, 482, 1538, 482, 1534, 486, 1534, 
	482, 1538, 482, 2474, 2938, 9010, 482, 1534, 
	482, 502, 482, 502, 482, 502, 482, 498, 
	482, 502, 486, 498, 482, 502, 482, 502, 
	482, 1534, 482, 498, 486, 502, 482, 1534, 
	486, 498, 482, 1538, 482, 1534, 482, 1534, 
	486, 1534, 482, 1534, 486, 1534, 482, 502, 
	482, 498, 486, 498, 482, 502, 482, 502, 
	482, 502, 482, 498, 482, 502, 482, 502, 
	482, 502, 482, 498, 486, 498, 482, 502, 
	482, 502, 482, 498, 486, 498, 486, 498, 
	486, 498, 482, 502, 482, 498, 486, 498, 
	486, 498, 482, 502, 486, 498, 482, 502, 
	482, 498, 486, 498, 486, 498, 482, 502, 
	482, 498, 486, 498, 486, 498, 482, 502, 
	482, 502, 482, 498, 486, 498, 482, 2478, 
	2966, 8982, 482, 1538, 482, 502, 482, 498, 
	486, 498, 482, 502, 482, 502, 482, 498, 
	482, 502, 486, 498, 482, 1534, 482, 502, 
	482, 502, 482, 502, 482, 1534, 482, 1534, 
	490, 1530, 486, 498, 482, 1534, 486, 1534, 
	482, 1534, 486, 498, 486, 1534, 482, 502, 
	486, 1530, 486, 1534, 486, 498, 482, 498, 
	486, 498, 486, 1534, 486, 1534, 486, 1530, 
	486, 498, 486, 498, 482, 502, 482, 498, 
	486, 498, 490, 494, 486, 1534, 486, 1534, 
	482, 498, 490, 1530, 486, 494, 490, 1530, 
	490, 1530, 486, 498, 486, 498, 486, 498, 
	486, 498, 482, 498, 490, 494, 486, 498, 
	486, 498, 486, 1534, 486, 1530, 486, 1534, 
	486, 1534, 486, 1000};


#define RAW_DATA_LEN 348
uint16_t rawDataOff[RAW_DATA_LEN] = {  
 2994, 8986, 482, 502, 482, 1538, 482, 502, 
	478, 506, 478, 502, 482, 502, 482, 502, 
	478, 506, 462, 518, 482, 1538, 482, 502, 
	482, 502, 478, 1542, 478, 1538, 482, 502, 
	482, 1562, 454, 1538, 482, 1538, 482, 1534, 
	482, 1538, 482, 502, 482, 502, 482, 502, 
	478, 502, 482, 506, 478, 502, 478, 502, 
	482, 502, 482, 502, 482, 502, 482, 502, 
	482, 502, 482, 502, 478, 502, 458, 530, 
	454, 526, 482, 502, 482, 502, 482, 502, 
	482, 498, 482, 502, 482, 502, 482, 502, 
	482, 502, 482, 502, 482, 498, 482, 502, 
	482, 502, 482, 502, 482, 502, 482, 502, 
	482, 498, 486, 498, 482, 502, 482, 1538, 
	482, 1534, 482, 2478, 2938, 9010, 482, 1538, 
	482, 526, 454, 506, 478, 506, 478, 502, 
	482, 502, 482, 502, 486, 498, 482, 502, 
	478, 1566, 454, 530, 454, 502, 482, 1534, 
	486, 526, 454, 1538, 482, 1538, 478, 1538, 
	482, 1538, 454, 1566, 478, 1538, 482, 502, 
	482, 498, 482, 502, 482, 526, 454, 506, 
	478, 506, 478, 530, 454, 502, 482, 502, 
	482, 502, 478, 506, 478, 506, 478, 506, 
	478, 502, 482, 502, 482, 502, 478, 506, 
	478, 502, 482, 502, 482, 502, 482, 502, 
	478, 506, 478, 502, 482, 502, 482, 502, 
	478, 506, 478, 506, 478, 502, 482, 502, 
	482, 502, 478, 502, 482, 502, 482, 506, 
	478, 502, 482, 502, 482, 502, 482, 2474, 
	2966, 8982, 482, 1538, 482, 502, 482, 498, 
	482, 502, 482, 502, 482, 502, 478, 502, 
	482, 506, 478, 502, 482, 1538, 478, 502, 
	482, 502, 478, 506, 454, 530, 454, 530, 
	478, 506, 478, 1538, 482, 1534, 482, 1538, 
	482, 1538, 478, 506, 478, 1538, 482, 502, 
	482, 1534, 482, 1538, 482, 502, 454, 530, 
	482, 498, 482, 1538, 482, 1534, 482, 1538, 
	482, 530, 454, 502, 482, 498, 482, 502, 
	458, 526, 482, 502, 482, 1538, 482, 1534, 
	482, 502, 482, 1538, 482, 498, 458, 1562, 
	482, 1538, 478, 506, 478, 502, 482, 502, 
	482, 502, 482, 502, 454, 526, 482, 502, 
	482, 502, 482, 502, 482, 502, 482, 1538, 
	478, 1542, 482, 1000};

void setup() {
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // use these three pins to drive an LED
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
 
  if (!SD.begin(SD_ChipSelectPin)) {  // see if the card is present and can be initialized:
    Serial.println("SD fail");  
  }
  else{   
    Serial.println("SD ok");  
  tmrpcm.speakerPin = 11;
  tmrpcm.setVolume(-10);
  tmrpcm.play("song.wav");
}
 pinMode(13, OUTPUT);
 digitalWrite(13 , HIGH);

}


void loop() {
  float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

 
  

 ir = digitalRead(7); 
  if(ir==HIGH)
  {
    Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  
  Serial.print("\n");

    #if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif

tmrpcm.setVolume(5);

  }
int chk = DHT.read11(DHT11_PIN);
  C = DHT.temperature;    	
Serial.print(C);
 
 
   lcd.setCursor(0,0);
   lcd.print(C);
     lcd.setCursor(7,0);
    lcd.print("DEGREES C");
  delay(1000);     

  if (C>23.0) {
      mySender.send(rawDataOn,RAW_DATA_LEN,36);      
      Serial.println(F("AC Switched On"));
    }
    else if (C<19.0) {
      mySender.send(rawDataOff,RAW_DATA_LEN,36);
      Serial.println(F("AC Switched Off"));
    }
}

