#include <DHT.h>
#include <printf.h>
#include <RF24.h>
#include <IrSenderNonMod.h>

#define PIN_DHT                  8              // PIN for DHT sensor communication.
#define PIN_RF24_CSN             10              // CSN PIN for RF24 module.
#define PIN_RF24_CE              9             // CE PIN for RF24 module.
#define NRF24_CHANNEL            113              // 0 ... 125
#define NRF24_CRC_LENGTH         RF24_CRC_16    // RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16 for 16-bit
#define NRF24_DATA_RATE          RF24_1MBPS   // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
//#define NRF24_DYNAMIC_PAYLOAD    1
#define NRF24_PAYLOAD_SIZE       16              // Max. 32 bytes.
#define NRF24_PA_LEVEL           RF24_PA_MIN    // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    
#define NRF24_RETRY_DELAY        5              // Delay bewteen retries, 1..15.  Multiples of 250µs.
#define NRF24_RETRY_COUNT        15              // Number of retries, 1..15.

#define PROTOCOL 0x01                           // 0x01 (byte), temperature (float), humidity (float)
                                                // Python 1: "<Bff"

#define DHT_TYPE              DHT11             // Type of DHT sensor:
                                                // DHT11, DHT12, DHT21, DHT22 (AM2302), AM2301                                     
RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);          // Cretate NRF24L01 radio.
DHT dht(PIN_DHT, DHT_TYPE);                     // Create DHT sensor.

byte rf24_tx[6] = "1Node"; 
byte rf24_rx[6] = "2Node";                     // Address used when transmitting data.

struct PayloadStruct {
  float temperature; 
  float humidity;                          // only using 6 characters for TX & RX payloads
  float people;
};
PayloadStruct payload;

unsigned long last_reading = 0;                 // Milliseconds since last measurement was read.
unsigned long ms_between_reads = 2000;         // 10000 ms = 10 seconds
//SR04 Setup
const int trigPin1 = 4;
const int echoPin1 = 5;
const int trigPin2 = 6;
const int echoPin2 = 7;

// Variables for storing distance and duration
long duration1,duration2,time1,time2;
float distance1,distance2;
float p=5;

//IR module simulating remote signals

static constexpr pin_t NON_MOD_PIN = 2U;
static const microseconds_t increaseData[] = {
    388, 1164, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 1164, 388, 388,
    1164, 1164, 388, 388, 1164, 1164, 388, 388, 1164, 1164, 388, 388, 1164,
    1164, 388, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 1164,
    388, 388, 1164, 1164, 388, 388, 1164, 388, 1164, 388, 11364
};
static const IrSequence increase(increaseData, sizeof(increaseData)/sizeof(microseconds_t));

static const microseconds_t decreaseData[] = {
    388, 1164, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 1164, 388, 388,
    1164, 1164, 388, 388, 1164, 1164, 388, 388, 1164, 1164, 388, 388, 1164,
    1164, 388, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 388, 1164, 1164,
    388, 388, 1164, 1164, 388, 388, 1164, 1164, 388, 388, 11364
};
static const IrSequence decrease(decreaseData, sizeof(decreaseData)/sizeof(microseconds_t));
static IrSenderNonMod sender(NON_MOD_PIN);

void setup() {
  // Initialize serial.
  Serial.begin(19200);
  //SR04
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
 // printf_begin();
  delay(100);

  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  // Show that program is starting.
  //Serial.println("\n\nNRF24L01 Arduino Simple Sender.");

  // Configure the NRF24 tranceiver.
 // Serial.println("Configure NRF24 ...");
  //Serial.println(radio.isChipConnected());
  nrf24_setup();
  radio.flush_rx();
  radio.flush_tx();
  
  // Show debug information for NRF24 tranceiver.
 // radio.printDetails();
  
  // Initialise the DHT sensor.
  dht.begin();
}

void loop() {

  if (millis() - last_reading > ms_between_reads) {
    // Read sensor values every "ms_between_read" milliseconds.
  
    // Read the humidity and temperature.
    float t, h;
    h = dht.readHumidity();
    t = dht.readTemperature();
    //p = p+people();
    //Serial.println(people());
    if(p<0){p = 0;}
    
    // Report the temperature and humidity.    
    //Serial.print("Sensor values: temperature="); Serial.print(t); 
    //Serial.print(", humidity="); Serial.println(h);
    
    // Stop listening on the radio (we can't both listen and send).
    radio.stopListening();
    delay(50);

    // Send the data ...
    //send_reading(t,h);

    send_reading(t,h,p);
    delay(10);

    // This block is for replicating remote control of the AC using IR sensor
    if(t>read_control_command()){
      sender.sendNonModulated(decrease, 2);
      delay(5);
    }
    if(t<read_control_command()){
      sender.sendNonModulated(increase, 2);
      delay(5);
    }


    // Register that we have read the temperature and humidity.
    last_reading = millis();
  }
}

void send_reading(float temperature, float humidity, float people)
{
  payload ={temperature, humidity, people};
  byte preparedData[sizeof(payload)];
  memcpy(preparedData, &payload, sizeof(payload));
  bool success =  radio.write(&preparedData, sizeof(preparedData));  // Perform the write operation

  Serial.print(payload.temperature);
  Serial.print("  ");
  Serial.print(payload.humidity);
  Serial.print("  ");
  Serial.print(payload.people);
  Serial.print("  ");
  
 if (success) {
    //Data sent successfully
    Serial.println("Data sent successfully");
  } else {
    //Failed to send data
    Serial.println("Failed to send data");
  }  
}

float read_control_command()
{
  radio.startListening();
  float receivedData={};
  if ( radio.available()) {
    while (radio.available()) {
          radio.read(&receivedData, sizeof(receivedData));
        }     
  }
  return receivedData;
}    

int people(){
  int plusCount=0;
  int minusCount =0;
  long time1Array[100]={};
  long time2Array[100]={};
  float sensor1Array[100] = {};
  float sensor2Array[100] = {};
  // Clear the trigPin
  for(int i=0;i<100;i++){
    digitalWrite(trigPin1, LOW);
    digitalWrite(trigPin2, LOW);

    delayMicroseconds(2);
    // Set the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);

    // Read the echoPin, returns the sound wave travel time in microseconds
    duration1 = pulseIn(echoPin1, HIGH);
    
    // Calculate the distance in centimeters
    sensor1Array[i] = duration1 * 0.035 / 2;
    time1Array[i]=millis();

    // Serial.print(sensor1Array[i]);
    // Serial.print(" ");

    // if(sensor1Array[i]<180){
    //   time1 = millis();
    //   digitalWrite(trigPin1, LOW);
    // }
    //digitalWrite(trigPin2, LOW);
    //delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);
    sensor2Array[i] = duration2 * 0.035 / 2;
    time2Array[i]=millis();
    //Serial.println(sensor2Array[i]);
  }
  int i=0;
  while(i<100){
    if(sensor1Array[i]<=70){
      time1=time1Array[i];
      delay(100);
      break;
    }
    i++;
  }
  int j=0;
  while(j<100){
    if(sensor2Array[j]<=40){
      time2=time2Array[j];
      delay(100);
      break;
    }
    j++;
  }
  
  if(time1<time2){
    return -1;
  }
  else if(time1>time2){
    return 1;
  } else {return 0;}
}

void nrf24_setup()
{
  radio.begin();
  radio.setAutoAck(false);                 
  radio.disableAckPayload();
  radio.disableDynamicPayloads();
  radio.setPALevel(NRF24_PA_LEVEL);
  radio.setRetries(NRF24_RETRY_DELAY, NRF24_RETRY_COUNT);              
  radio.setDataRate(NRF24_DATA_RATE);          
  radio.setChannel(NRF24_CHANNEL);
  radio.setCRCLength(NRF24_CRC_LENGTH);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);
  radio.openWritingPipe(rf24_tx);
  radio.openReadingPipe(1, rf24_rx);  
  }