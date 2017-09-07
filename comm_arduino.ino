//Author : Oumeima El Isbihani
//Title : Arduino side first sensor

//----------------------------------------------------------------------------------------------//
//                                      USED LIBRARIES                                          //
//----------------------------------------------------------------------------------------------//
#include<SPI.h>
#include<RF24.h>                                                            //radio transmitter
#include<Adafruit_Sensor.h>                                                 //humidity + temp sensor
#include<DHT.h>                                                             //humidity + temp sensor
#include<DHT_U.h>                                                           //humidity + temp sensor
#include "LowPower.h"                                                       //power consumption
#include <Battery.h>                                                        //battery monitoring
#include "MCP3421.h"                                                        //ADC converter SapFlowSensor
#include <SimpleTimer.h>                                                    //Library to do timed loops
#include  <Wire.h>
#include <math.h>                                                           //to use the log for heat velocity
//----------------------------------------------------------------------------------------------//
//                                      CONNECTION PINS                                         //
//----------------------------------------------------------------------------------------------//
#define POWER_SENSE A1                                                      //must be analog pin
#define POWER_ACTIV A2                                                      //either analog or digital
#define DHTPIN 7                                                            //pin for to the DHT sensor
#define DHTTYPE DHT11                                                       //DHT 11 type

const int sensor_light = A0;                                                //pin for the light sensor

const int transistorH_cap = 4;                                              //connected to the base of the first transistor for charging the capacitor
const int transistorB_cap = 5;                                              //connected to the base of the second transistor between the capacitor and the wire

const int transistorP_therm1 = 6;                                           //connected to the base of the third transistor for the VIN + of the first thermistor
const int transistorM_therm1 = 8;                                           //connected to the base of the forth transistor for the VIN + of the first thermistor

const int transistorP_therm2 = 2;                                           //connected to the base of the third transistor for the VIN + of the second thermistor
const int transistorM_therm2 = 3;                                           //connected to the base of the sixth transistor for the VIN - of the second thermistor

float result_1;                                                             //result of the upper thermistor
float result_2;                                                             //result of the lower thermistor
const float k = 2.5 * pow(10,-3);                                           //thermal diffusivity of green wood
const float c_w = 1200;                                                     //heat capacity of wood
const float c_s = 4185;                                                     //heat capacity of sap
const float rho_b = 0.45;                                                   //density of wood : Redwood, American 
const float rho_s = 1;                                                      //density of water
const float m_c = 25;                                                       //water content of sap wood
const float S_A = 115;                                                      //cross section of the tree in the heated probe level => 115 cm²
const float x = 0.06;                                                       //distance between heated probe and temperature probe (in mm)

uint32_t delayMS;                                                           //delay for the two data transmition for the DHT11

SimpleTimer timer;                                                          // the timer object
Battery battery(6000, 8400, POWER_SENSE, POWER_ACTIV);                      // Battery(uint16_t minVoltage, uint16_t maxVoltage, uint8_t sensePin, uint8_t activationPin = 0xFF);
DHT_Unified dht(DHTPIN, DHTTYPE);                                           //DHT definition
MCP3421 MCP = MCP3421();                                                    //ADC definition MCP3421
RF24 radio(9,10);                                                           //creation of the NRF24L01

float T0;
float V0;
float p1;
float p2;
float p3;
float p4;
float q1;
float q2;
float q3;
//float T_CJ = 23.0; // temperature jonction  froide
//float v_cj = 0.992;
char st1[20];

long l1;
double vin;
float f1;

//----------------------------------------------------------------------------------------------//
//                                     MESSAGES                                                 //
//----------------------------------------------------------------------------------------------//
String resultat;
String id = "1";                                                            //sensor id
String hum;
String temp;
String brightness;
String sapflow;
String battery_level;
String battery_voltage;
String minimum_batteryvoltage = "6.00";                                     //minimum battery voltage
String alerte = "CHARGING PROBLEM";                                         //ALERT message when low voltage
                      //----------------------------------------------------------------------------------------------//
                      //                                      SETUP NRF24L01                                          //
                      //----------------------------------------------------------------------------------------------//
void setup_NRF24(void){
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(0x50);
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  const uint64_t pipe = 0xE8E8F0F0E1LL;                                     //reading pipe hexadecimal address  
  radio.openReadingPipe(1, pipe);                                           //open reading pipe
  radio.enableDynamicPayloads();
  radio.powerUp();
}

void setup_DHT11(void){
// Initialize device.
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;;
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      GET DATA FROM DHT11                                     //
                      //----------------------------------------------------------------------------------------------//
void get_DHT11(void){
  delay(delayMS);                                                           // Delay between measurements.
  sensors_event_t event; 
  dht.humidity().getEvent(&event);                                          // Get humidity event and print its value.
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    //Serial.print("Humidity: ");
    //Serial.print(event.relative_humidity);
    //Serial.println("%");
    hum += event.relative_humidity;
    //Serial.println(hum);
  }
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    //Serial.print("Temperature: ");
    //Serial.print(event.temperature);
    //Serial.println(" *C");
    temp += event.temperature;
    //Serial.println(temp);
  }

}
                      //----------------------------------------------------------------------------------------------//
                      //                                      GET LIGHT                                               //
                      //----------------------------------------------------------------------------------------------//
void get_light(void) {
  int sensorValue = analogRead(sensor_light);                               //create a var to store the value of the sensor
  //Serial.println("the analog read data is ");                             //print on the serial monitor what's in the ""
  //Serial.println(sensorValue);                                            // print the value of the sensor on the serial monitor
  brightness += sensorValue;
  //Serial.println(brightness);
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      CLEANING FUNCTION                                       //
                      //----------------------------------------------------------------------------------------------//
void clean(void){
  resultat.remove(0);
  hum.remove(0);
  temp.remove(0);
  brightness.remove(0);
  sapflow.remove(0);
  battery_voltage.remove(0);
  battery_level.remove(0);
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      TRANSISTOR CONTROL                                      //
                      //----------------------------------------------------------------------------------------------//
void charging_cap(void){
  //for 4 min = charging the capacitor
  digitalWrite(transistorH_cap, HIGH);                                       //the current is going to the capacitor
  digitalWrite(transistorB_cap, LOW);                                        //the capacitor is charging
}
void discharging_cap(void){
  //for 1*10^-8 s = heating the wire
  digitalWrite(transistorH_cap, LOW);                                        //the current is not going to the capacitor
  digitalWrite(transistorB_cap, HIGH);                                       //the capacitor is discharging 
}

void transistor(void){
  timer.setInterval(0.00001, discharging_cap);  
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      GET TEMP COLD JUNC                                      //
                      //----------------------------------------------------------------------------------------------//
float get_tempcj(void){
  float temperature = temp.toFloat();
  return temperature;
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      COLD JUNCTION TEMP                                      //
                      //----------------------------------------------------------------------------------------------//
float get_coldjunc(){
  T0 = 2.5*pow(10,1);
  //Serial.println(T0); 
  V0 = 9.9198279*pow(10,-1);
  //Serial.println(V0);
  p1 = 4.0716564*pow(10,-2);
  //Serial.println(p1);
  p2 = 7.1170297*pow(10,-4);
  //Serial.println(p2);
  p3 = 6.8782631*pow(10,-7);
  //Serial.println(p3);
  p4 = 4.3295061*pow(10,-11);
  //Serial.println(p4);
  q1 = 1.6458102*pow(10,-2);
  //Serial.println(q1);
  q2 = 0.0;
  //Serial.println(q2);
  float T_CJ = get_tempcj();
  Serial.print("Temperature cold junction ");
  Serial.println(T_CJ);
  float v_CJ = V0 + (((T_CJ-T0)*(p1+(T_CJ-T0)*(p2+(T_CJ-T0)*p3+p4*(T_CJ-T0))))/(1+(T_CJ-T0)*(q1+q2*(T_CJ-T0))));
  Serial.print("Voltage cold junction : ");
  Serial.println(v_CJ);

  return v_CJ;
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      TEMPERATURE CALCULATION                                 //
                      //----------------------------------------------------------------------------------------------//
float get_temp(float v_ther){
  if((v_ther>-4.648)||(v_ther<0)){
    T0 = -6.0*pow(10,1);
    //Serial.println(T0);
    V0 = -2.1528350*pow(10,0);
    //Serial.println(V0);
    p1 = 3.0449332*pow(10,1);
    //Serial.println(p1);
    p2 =-1.294656*pow(10,0);
    //Serial.println(p2);
    p3 =-3.0500735*pow(10,0);
    //Serial.println(p3);
    p4 =-1.9226856*pow(10,-1);
    //Serial.println(p4);
    q1 =6.9877863*pow(10,-3);
    //Serial.println(q1);
    q2 =-1.0596207*pow(10,-1);
    //Serial.println(q2);
    q3 =-1.0774995*pow(10,-2);
    //Serial.println(q3);
  }else if ((v_ther>0)||(v_ther<9.288)){
    T0 =1.35*pow(10,2);
    //Serial.println(T0);
    V0 =5.9588600*pow(10,0);
    //Serial.println(V0);
    p1 =2.0325591*pow(10,1);
    //Serial.println(p1);
    p2 =3.3013079*pow(10,0);
    //Serial.println(p2);
    p3 =1.2638462*pow(10,-1);
    //Serial.println(p3);
    p4 =-8.2883695*pow(10,-4);
    //Serial.println(p4);
    q1 =1.7595577*pow(10,-1);
    //Serial.println(q1);
    q2 =7.9740521*pow(10,-3);
    //Serial.println(q2);
    q3 =0;
    //Serial.println(q3);
  }
  float v_cj = get_coldjunc();
  float v_therm = v_ther - v_cj;
  Serial.print("Voltage : ");
  Serial.println(v_therm);  
  float T = T0 + ((v_therm-V0)*(p1+(v_therm-V0)*(p2+(v_therm-V0)*(p3+p4*(v_therm-V0)))))/(1+(v_therm-V0)*(q1+(v_therm-V0)*(q2+q3*(v_therm-V0))));
  return T;
}

                      //----------------------------------------------------------------------------------------------//
                      //                                      MCP3421                                                 //
                      //----------------------------------------------------------------------------------------------//

  
float adc_convert_first(void){                                               //get the data from the first thermocouple by controling the transistors
  //Transistor pin 6 and pin 8 HIGH
  digitalWrite(transistorP_therm1, HIGH);                                    //the first thermocouple is connected to the MCP
  digitalWrite(transistorM_therm1, HIGH);                                       

  //Transistor pin 2 and pin 3 LOW 
  digitalWrite(transistorP_therm2, LOW);                                     //the second thermocouple is disconnected from the MCP
  digitalWrite(transistorM_therm2, LOW); 

  while(MCP.ready()==0);

  vin=MCP.getDouble();
  f1=vin;
  Serial.print("Voltage first probe (mv):");
  Serial.println(f1*1000);

  result_1 = get_temp(f1 * 1000);
  Serial.print("Temperature 1ere probe : ");
  Serial.println(result_1);
  return result_1;             
}
float adc_convert_second(void){                                              //get the data from the second thermocouple (heated) by controling the transistors
  //Transistor pin 3 and pin 2 HIGH
  digitalWrite(transistorP_therm2, HIGH);                                    //the second thermocouple is connected to the MCP
  digitalWrite(transistorM_therm2, HIGH); 
  
  //Transistor pin 6 and pin 8 LOW
  digitalWrite(transistorP_therm1, LOW);                                     //the first thermocouple is disconnected from the MCP
  digitalWrite(transistorM_therm1, LOW);    

  while(MCP.ready()==0);
  vin=MCP.getDouble();
  f1=vin;
  Serial.print("Voltage second probe (mV):");
  Serial.println(f1*1000);
  
  result_2 = get_temp(f1 * 1000);
  Serial.print("Temperature 2eme probe : ");
  Serial.println(result_2);
  return result_2;                
}

                      //----------------------------------------------------------------------------------------------//
                      //                                      SAPFLOW SENSOR                                          //
                      //----------------------------------------------------------------------------------------------//
void sapflowcalculation(void){
  Serial.println("BEFORE HEATING");
  //get the temperature from the first probe before heating
  float temp1_before = adc_convert_first();
  //get the temperature from the second probe before heating
  float temp2_before = adc_convert_second();

  //heat the middle probe
  //timer.run();                                                                //discharge the capacitor

  Serial.println("AFTER HEATING");
  //get the temperature from the first probe after heating
  float temp1_after = adc_convert_first();
  //get the temperature from the second probe after heating
  float temp2_after = adc_convert_second();

  //calculate the temperature difference for the first probe
  float delta_temp_1 = temp1_before - temp1_after;
  Serial.print("Delta temperature 1 ");
  Serial.println(delta_temp_1);
  
  //calculate the temperature difference for the second probe
  float delta_temp_2 = temp2_before - temp2_after;
  Serial.print("Delta temperature 2 ");
  Serial.println(delta_temp_2);
  
  //calculating the heat pulse velocity Vh
  float V_h = (k/x)*log(abs(delta_temp_1)/abs(delta_temp_2))*3600;
  Serial.print("Heat velocity ");
  Serial.println(V_h);
  
  //calculating the sap velocity Vs
  float V_s = ((V_h*rho_b*(c_w+m_c*c_s))/(rho_s*c_s));
  Serial.print("Sap velocity ");
  Serial.println(V_s);
  
  //calcultating the sap flow F
  float F = V_s * S_A;
  Serial.print("Sap Flow : ");
  Serial.println(F);
  
  //putting the values back to 0
  sapflow += F;
  result_1 = 0;
  result_2 = 0;
  
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      GET STRING RESULT                                       //
                      //----------------------------------------------------------------------------------------------//
void get_result(void) {
  resultat += id;
  resultat += ";"; 
  resultat += temp;
  resultat += ";";
  resultat += hum;
  resultat += ";";
  resultat += brightness;
  resultat += ";";
  resultat += sapflow;
  //Serial.println(resultat);
}

                      //----------------------------------------------------------------------------------------------//
                      //                                      SETUP FUNCTION                                          //
                      //----------------------------------------------------------------------------------------------//
void setup(void){
  Serial.begin(9600);                                                       //initialize the serial monitor at 9600 baud rate
  setup_NRF24();
  setup_DHT11();
  battery.begin();
  Wire.begin();
  delay(1000);
 // Serial.println("begin");
  pinMode(transistorH_cap, OUTPUT);                                          //set the transistor pin as output
  pinMode(transistorB_cap, OUTPUT);                                          //set the transistor pin as output
  pinMode(transistorP_therm1, OUTPUT);                                       //set the transistor pin as output
  pinMode(transistorM_therm1, OUTPUT);                                       //set the transistor pin as output
  pinMode(transistorM_therm2, OUTPUT);                                       //set the transistor pin as output
  pinMode(transistorP_therm2, OUTPUT);                                       //set the transistor pin as output 
  transistor();
  MCP.init(0x68,3,3);
}
                      //----------------------------------------------------------------------------------------------//
                      //                                     POWER SAVING FUNCTION                                    //
                      //----------------------------------------------------------------------------------------------//
void sleep(void){
  //sleep for 4 minutes
  for(int i=0; i <30; i++){       // (4*60)/8
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  }                  
}
                      //----------------------------------------------------------------------------------------------//
                      //                                     BATTERY MONITORING                                       //
                      //----------------------------------------------------------------------------------------------//
void battery_monit(void){
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);

  Serial.print("Battery voltage is ");
  battery_voltage += battery.voltage();
  Serial.println(battery_voltage);
  Serial.print("Battery level is");
  battery_level += battery.level();
  Serial.println(battery_level);
 
}
                      //----------------------------------------------------------------------------------------------//
                      //                                      LOOP FUNCTION                                           //
                      //----------------------------------------------------------------------------------------------//
void loop(void){
  radio.startListening();
  Serial.println("Starting loop.Radio on.");
  char receivedMessage[32] = {0} ; 
  //charging_cap();                                                             //charging the capacitor 
  sleep();                                                                    //sleeping for four minutes  
  if(radio.available()){                                                      //when a message is received  
    radio.read(receivedMessage, sizeof(receivedMessage));
    Serial.println(receivedMessage);
    Serial.println("Turning off the radio.");
    radio.stopListening();                                                    //stops listening
    String stringMessage(receivedMessage);
    if (stringMessage == "GETDATA"){                                          //if its the right message
        //battery_monit();                                                      //monitor the battery level
        get_DHT11();
        sapflowcalculation();                                                 //calculates the sapflow
        get_light();                                                          //get all the results
        get_result();                                                         //get all the results
        char result[resultat.length()+1];
        resultat.toCharArray(result,resultat.length()+1);
        Serial.println(resultat);
        if (battery_voltage > minimum_batteryvoltage) {
            radio.write(&result, sizeof(result));                             //if there is enough battery voltage message sent
        }else{
            char result[alerte.length()+1];
            alerte.toCharArray(result,alerte.length()+1);                     //if not alert message is sent
            radio.write(&result, sizeof(result));
        }
        clean();                                                              //clean the strings
      }
    }
  //delay(1000);
  
}

