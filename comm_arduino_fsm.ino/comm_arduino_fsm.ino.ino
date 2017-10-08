#include <RF24.h>
#include <RF24_config.h>
//#include <printf.h>
#include <nRF24L01.h>
//Include needed Libraries at beginning
#include "SPI.h"

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

enum { PACKET_CONFIGURE = 1, PACKET_DATA = 2 };

enum { STATE_WAITING_CONFIGURATION, STATE_SENDING_DATA };
enum { ACTION_SENSOR_CONFIGURED, ACTION_SENSOR_DATA_COLLECTION_DONE };

uint8_t receivedMessage[32] = {0} ;
int stateSensor;

int message[] = { 0 };

RF24 radio(9, 10);

const uint64_t pipeWriting = 0xE6E6E6E6E6E6, pipeReading = 0xF6F6F6F6F6F6;

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

void loopSendingData() {
  
  static struct packetData data;
  
  if (sensorState.durationOfDataCollection > 0) {
      
    data.header.type = PACKET_DATA;
    data.temperature = 15.2;
    data.humidity = 27.2;
    data.brightness = 21.;
    data.flow = 15;    
    
    sensorState.durationOfDataCollection--;
    
    radio.write(&data, sizeof(data));
    
    Serial.print("Sending data ");
    Serial.println(sizeof(data));
    
    delay(sensorState.intervalToCollectData * 1000);
    
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
