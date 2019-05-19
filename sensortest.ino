#include <Wire.h>
#include "SHTSensor.h"
#include <Adafruit_MPL3115A2.h>
#include "String.h"
#include "LiquidCrystal.h"



#define Rl 47000
#define Vcc 3.3
#define Ro 33000
#define ButPIN 50 //Pullup resistor switch
#define LCD_PWRPIN 53
#define LCD_LIGHTPIN 48

String warning = "WARNING! BAD AIR QUALITY";


SHTSensor sht;
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
LiquidCrystal lcd(24,26,28,30,32,34);  //LCD PIN mapping for Mega 

bool init_sht = 0;
bool init_mpl = 0;
float humidity = 0;
float temperature = 0;
bool warningflag = 0;

void LCDplay(void) //LCD Display function for temperature
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RH: ");
  lcd.print(humidity);
  lcd.setCursor(0,1);
  lcd.print("T: ");
  lcd.print(temperature);

  if(warningflag == 1)
  {
  lcd.setCursor(0,0);
  lcd.print(warning);
  lcd.setCursor(0,1);
  lcd.print(warning);
  }
}

void setup() {

  Wire.begin();
  Serial.begin(9600);
  delay(1000); // let serial console settle
/*Initialize SHT sensor */
  if (sht.init()) {
    Serial.print("init SHT: success\n");
    init_sht = 1;
  }
  else {
    Serial.print("init SHT: failed\n");
    init_sht = 0;
  }
  /*Initialize MPL sensor*/
  if (! baro.begin()) 
  {
    Serial.println("Couldnt find MPL sensor");
    init_mpl = 0;
  }
  else{
    Serial.println(" init MPL: success ");
    init_mpl = 1;
    }
    pinMode(ButPIN,INPUT_PULLUP);
    pinMode(LCD_LIGHTPIN,OUTPUT);
    pinMode(LCD_PWRPIN,OUTPUT);
    digitalWrite(LCD_LIGHTPIN,LOW);
    digitalWrite(LCD_PWRPIN,HIGH);
}

void loop() {
  
  //SHT Code
  if(init_sht == 1){
  sht.readSample();
    humidity = sht.getHumidity();
  humidity = humidity;
    Serial.print("  RH: ");
    Serial.print(humidity, 2);
    Serial.print("\n");
  
    temperature = sht.getTemperature();
  temperature = temperature;
    Serial.print("  T:  ");
    Serial.print(temperature, 2);
    Serial.print("\n");
  }
  //MPL code
if (! baro.begin()) 
  {
    init_mpl = 0;
  }
  else{
    init_mpl = 1;
    }
  
  if(init_mpl == 1){
    float pascals = baro.getPressure();
    Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

    float altm = baro.getAltitude();
    Serial.print(altm); Serial.println(" meters");

    float tempC = baro.getTemperature();
    Serial.print(tempC); Serial.println("*C");
  }
  
  float sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  float voltage = sensorValue * (3.3 / 1023.0);
  // print out the value you read:
  float resistance = (Vcc/voltage - 1)*Rl;
  float output = resistance/Ro;
  
  if(output <= 0.8){
    warningflag = 1 ;
    Serial.print(warning);
  }
  else{
    warningflag = 0;
  }
  
  //Serial.print("Volt ");
  //Serial.print(voltage);
  //Serial.print("\n");
  Serial.print("OUT: ");
  //Serial.print(output);
  Serial.print(resistance);
  Serial.print("\n");
  //digitalWrite(LCD_PWRPIN,LOW);
  //Button state read
  int butstate = digitalRead(ButPIN);
  //LCDplay(); //debug
  if (butstate == LOW)
  {
    //digitalWrite(LCD_PWRPIN,HIGH);
    //delay(500);
    digitalWrite(LCD_LIGHTPIN,HIGH);
    LCDplay();
    Serial.print("UP");
  }
  else if(butstate == HIGH){
  //digitalWrite(LCD_PWRPIN,LOW); 
  digitalWrite(LCD_LIGHTPIN,LOW); 
  }
  
  delay(1000);
}
