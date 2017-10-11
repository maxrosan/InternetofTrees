
//Authors : Oumeima El Isbihani
//          Max Rosan

#define MOCK 1


#include <RF24.h>
#include <RF24_config.h>
//#include <printf.h>
#include <nRF24L01.h>
#include <SPI.h>

#include <SimpleTimer.h>                                                    //Library to do timed loops

#ifndef MOCK
#include <Adafruit_Sensor.h>                                                 //humidity + temp sensor
#include <DHT.h>                                                             //humidity + temp sensor
#include <DHT_U.h>                                                           //humidity + temp sensor
#include <Battery.h>                                                        //battery monitoring
#include "MCP3421.h"                                                        //ADC converter SapFlowSensor
#include <Wire.h>
#endif

#include "LowPower.h"                                                       //power consumption

#include <math.h>                                                           //to use the log for heat velocity

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

float sapflow;

float minimum_batteryvoltage = 6.00;
float battery_level;
float battery_voltage;

SimpleTimer timer;                                                          // the timer object

#ifndef MOCK
Battery battery(6000, 8400, POWER_SENSE, POWER_ACTIV);                      // Battery(uint16_t minVoltage, uint16_t maxVoltage, uint8_t sensePin, uint8_t activationPin = 0xFF);
DHT_Unified dht(DHTPIN, DHTTYPE);                                           //DHT definition
MCP3421 MCP = MCP3421();                                                    //ADC definition MCP3421
#endif

String temp;
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

// Structs

struct packet {
  uint8_t type;
};

struct packetConfigure {
  struct packet header;
  uint32_t interval;
  uint32_t duration;
};

struct packetData {
  struct packet header;
  float temperature;
  float humidity;
  float brightness;
  float flow;
};

struct {
  int intervalToCollectData;
  int durationOfDataCollection;
} sensorState;

// End [ Structs ]

enum { PACKET_CONFIGURE = 1, PACKET_DATA = 2 };

enum { STATE_WAITING_CONFIGURATION, STATE_SENDING_DATA };
enum { ACTION_SENSOR_CONFIGURED, ACTION_SENSOR_DATA_COLLECTION_DONE };

uint8_t receivedMessage[32] = {0} ;
int stateSensor;

RF24 radio(9, 10);

const uint64_t pipeWriting = 0xE6E6E6E6E6E6, pipeReading = 0xF6F6F6F6F6F6;

// Sensors

void setup_DHT11(void){

#ifndef MOCK  
  
// Initialize device.
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
//  delayMS = sensor.min_delay / 1000;;

#endif

}


inline static bool get_DHT11(struct packetData *data){

#ifndef MOCK
  
  sensors_event_t event;
  dht.humidity().getEvent(&event);

  if (isnan(event.relative_humidity)) {
    Serial.println("Failed to read humidity");
    return false;
  }

  if (isnan(event.temperature)) {
    Serial.println("Failed to read humidity");
    return false;
  }
  
  data->temperature = event.temperature;
  data->himidity = event.relative_humidity;

#else

  data->temperature = 15.;
  data->humidity = 30.;


#endif

  return true;
}

void get_light(struct packetData *data) {
  
#ifndef MOCK

  float sensorValue = (float) analogRead(sensor_light);                               //create a var to store the value of the sensor
  //Serial.println("the analog read data is ");                             //print on the serial monitor what's in the ""
  //Serial.println(sensorValue);                                            // print the value of the sensor on the serial monitor
  data->brightness = sensorValue;
  //Serial.println(brightness);

#else

  data->brightness = 50.;

#endif

}

//

void charging_cap(void){
#ifndef MOCK
  //for 4 min = charging the capacitor
  digitalWrite(transistorH_cap, HIGH);                                       //the current is going to the capacitor
  digitalWrite(transistorB_cap, LOW);                                        //the capacitor is charging
#endif
}

void discharging_cap(void){
#ifndef MOCK  
  //for 1*10^-8 s = heating the wire
  digitalWrite(transistorH_cap, LOW);                                        //the current is not going to the capacitor
  digitalWrite(transistorB_cap, HIGH);                                       //the capacitor is discharging 
#endif
}

void transistor(void){
#ifndef MOCK    
  timer.setInterval(0.00001, discharging_cap);  
#endif
}

float get_tempcj(void){
  float temperature = temp.toFloat();
  return temperature;
}

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


float adc_convert_first(void){    

#ifndef MOCK

  //get the data from the first thermocouple by controling the transistors
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
  
#else

  result_1 = 1.0;

  return 1.0;

#endif

}

float adc_convert_second(void){  //get the data from the second thermocouple (heated) by controling the transistors

#ifndef MOCK

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
 
#else

  result_2 = 1.0;

  return 1.0;
 
#endif
  
}

void sapflowcalculation(struct packetData *data){
  
#ifndef MOCK
    
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
  data->flow = F;
  result_1 = 0;
  result_2 = 0;
  
#else

  data->flow = 1.;
  result_1 = 0;
  result_2 = 0;

#endif
  
}

inline static void battery_monit(void){
  
#ifndef MOCK

  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);

  Serial.print("Battery voltage is ");
  battery_voltage = battery.voltage();
  Serial.println(battery_voltage);
  Serial.print("Battery level is");
  battery_level = battery.level();
  Serial.println(battery_level);
 
#endif

}

////

void fireAction(int action) {
 
 if (stateSensor == STATE_WAITING_CONFIGURATION && action == ACTION_SENSOR_CONFIGURED) {
   stateSensor = STATE_SENDING_DATA;
 } else if (stateSensor == STATE_SENDING_DATA && action == ACTION_SENSOR_DATA_COLLECTION_DONE) {
   stateSensor = STATE_WAITING_CONFIGURATION;
 }
 
}

void setup() {
  // put your setup code here, to run once:
  
  stateSensor = STATE_WAITING_CONFIGURATION;  
  Serial.begin(9600);  
  //Serial.setDebugOutput(true);
  
  setup_DHT11();
  
#ifndef MOCK

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

#endif    

  radio.begin();
  radio.setChannel(0x50);
  radio.openWritingPipe(pipeWriting);
  radio.openReadingPipe(1, pipeReading);
  radio.enableDynamicPayloads();
  radio.setPALevel(RF24_PA_MIN);
  
  radio.powerUp();

  delay(1000);
  
  radio.startListening();
  
  Serial.println("Waiting configuration");
}

void loopWaitingConfiguration() {

  if(radio.available()){
  
    struct packetConfigure *p;
    
    Serial.println("Listening");

    radio.read(receivedMessage, sizeof(receivedMessage));
    
    p = (struct packetConfigure*) receivedMessage;

    Serial.print("Type ");
    Serial.println(p->header.type);
    
    if (p->header.type == PACKET_CONFIGURE) {
      
      Serial.print("Interval ");
      Serial.print(p->interval);
      Serial.print(" ");
      Serial.println(p->duration);
      
      sensorState.intervalToCollectData    = p->interval;
      sensorState.durationOfDataCollection = p->duration;
      
      radio.stopListening();
      
      fireAction(ACTION_SENSOR_CONFIGURED);
      
    }
    
  } 
}

inline static void sleep( uint32_t time ){
  //sleep for 4 minutes
  for(int i=0; i < time; i += 8){       // (4*60)/8
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  }                  
}

void loopSendingData() {
  
  static struct packetData data;
  
  if (sensorState.durationOfDataCollection > 0) {
      
    data.header.type = PACKET_DATA;

    get_DHT11(&data);
    get_light(&data);
    sapflowcalculation(&data);    
    
    sensorState.durationOfDataCollection--;
    
    radio.write(&data, sizeof(data));
    
    Serial.print("Sending data ");
    Serial.println(sizeof(data));

    //delay(sensorState.intervalToCollectData * 1000);
    
    sleep(sensorState.intervalToCollectData);
    
  } else {
    
    fireAction(ACTION_SENSOR_DATA_COLLECTION_DONE);
    
  }
  
}

void loop() {
  
  if (stateSensor == STATE_WAITING_CONFIGURATION) {
    loopWaitingConfiguration();
  } else if (stateSensor == STATE_SENDING_DATA) {
    loopSendingData();
  } else {
    Serial.println("Unknown state"); 
    delay(1000);
  }

}
