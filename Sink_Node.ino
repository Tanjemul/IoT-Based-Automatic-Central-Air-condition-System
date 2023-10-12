#include <DHT.h>
#include <printf.h>
#include <RF24.h>
#include <IrSenderNonMod.h>

#define PIN_DHT                  8              // PIN for DHT sensor communication.
#define PIN_RF24_CSN             10              // CSN PIN for RF24 module.
#define PIN_RF24_CE              9              // CE PIN for RF24 module.
#define NRF24_CHANNEL            113              // 0 ... 125
#define NRF24_CRC_LENGTH         RF24_CRC_16    // RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16 for 16-bit
#define NRF24_DATA_RATE          RF24_1MBPS   // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define NRF24_PAYLOAD_SIZE       16              // Max. 32 bytes.
#define NRF24_PA_LEVEL           RF24_PA_MIN    // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    
#define DHT_TYPE                 DHT11             // Type of DHT sensor         

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);          // Cretate NRF24L01 radio.
DHT dht(PIN_DHT, DHT_TYPE);                     // Create DHT sensor.

byte rf24_rx[6] = "1Node";                      // Address used when listening data.   
byte rf24_tx[6] = "2Node";                      // Address used when transmitting data.   

struct PayloadStruct {
  float temperature; 
  float humidity;                          // only using 6 characters for TX & RX payloads
  float people;
};
PayloadStruct payload;                            // Payload bytes. Used both for transmitting and receiving
float controlCommand[2];
unsigned long last_reading = 0;                 // Milliseconds since last measurement was read.
unsigned long ms_between_reads = 3500;         // 10000 ms = 10 seconds

//SR04 Setup
const int trigPin1 = 4;
const int echoPin1 = 5;
const int trigPin2 = 6;
const int echoPin2 = 7;

// Variables for storing distance and duration
long duration1,duration2,time1,time2;
float distance1,distance2;
float p=10;

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
  //printf_begin();
  delay(100);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  //Serial.println("Configure NRF24 ...");
  //Serial.println(radio.isChipConnected());
  nrf24_setup();
  radio.flush_rx();
  radio.flush_tx();
  //radio.disableCRC();
  //Show debug information for NRF24 tranceiver.
  radio.printDetails();
  // Initialise the DHT sensor.
  dht.begin();

  //Initialise SR04s
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {

  if (millis() - last_reading > ms_between_reads) {
    // Read sensor values every "ms_between_read" milliseconds.
  
    float t, h;
    h = dht.readHumidity();
    t = dht.readTemperature();
    //people();
    //p = p+people();
    //Serial.println(people());
    if(p<0){p = 0;}
    
    // Report the temperature and humidity.    
    //Serial.print("Sensor values: temperature="); Serial.print(t); 
    //Serial.print(", humidity="); Serial.println(h);
    
    // Stop listening on the radio (we can't both listen and send).
    radio.stopListening();
    delay(10);

    radio.startListening();
    read_reading(t,h,p);

    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');

        // Split the received data into individual float values using ',' as the delimiter.
        int index = 0;
        char* dataChar = strdup(data.c_str());
        char* token = strtok(dataChar, ",");
        while (token != NULL && index < 2) {
            controlCommand[index] = atof(token);
            token = strtok(NULL, ",");
            index++;
        }
        free(dataChar);
        if(t>controlCommand[1]){
          sender.sendNonModulated(decrease, 2);
          delay(5);
        }
        if(t<controlCommand[1]){
          sender.sendNonModulated(increase, 2);
          delay(5);
        }
        radio.stopListening();
        delay(5);
        send_control_command(controlCommand[0]);
      } 
    // Register that we have read the temperature and humidity.
    last_reading = millis();
  }
}

void read_reading(float temperature2, float humidity2, float people2) 
{
  radio.startListening();
  PayloadStruct receivedData;

  byte receivedPayload[sizeof(receivedData)];

  if ( radio.available()) {
    while (radio.available()) {
          radio.read(&receivedPayload, sizeof(receivedPayload));
        }
    }
    //Serial.print(sizeof(receivedPayload));
    // Deserialize the received data
    memcpy(&receivedData, receivedPayload, sizeof(receivedData));

    float bundleArray[6] = {receivedData.temperature, receivedData.humidity, receivedData.people,
    temperature2, humidity2, people2};
    //radio.read(&received, sizeof(received));  // get incoming payload

    const int arraySize = sizeof(bundleArray) / sizeof(bundleArray[0]);

    String arrayString = "";
    for (int i = 0; i < arraySize; i++) {
      arrayString += String(bundleArray[i], 2); // 2 decimal places
      if (i < arraySize - 1) {
        arrayString += ","; // add comma and space between elements
      }
    }
    Serial.println(arrayString); // Prints the array as a string

    delay(10); // Delay for 1 second before repeating
    // //Serial.print(bytes);  // print the size of the payload
    // Serial.print(F(": "));
    // Serial.print(receivedData.temperature);
    // Serial.print(" ");  // print incoming message
    // Serial.print(receivedData.humidity);
    // Serial.print(" ");
    // Serial.print(receivedData.people);
}

void send_control_command(float temperature)
{
  float command_payload[1] = {temperature};
  bool success =  radio.write(&command_payload, sizeof(command_payload));  // Perform the write operation
  if (success) {
    // Data sent successfully
    //Serial.println("Data sent successfully");
  } else {
    // Failed to send data
    //Serial.println("Failed to send data");
  }   
}

int people(){
  long time1Array[50]={};
  long time2Array[50]={};
  float sensor1Array[50] = {};
  float sensor2Array[50] = {};
  // Clear the trigPin
  for(int i=0;i<50;i++){
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

    //Serial.print(sensor1Array[i]);
    //Serial.print(" ");

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
  while(i<50){
    if(sensor1Array[i]<=50){
      time1=time1Array[i];
      break;
    }
    i++;
  }
  int j=0;
  while(j<50){
    if(sensor2Array[j]<=45){
      time2=time2Array[j];
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
  radio.setPALevel(NRF24_PA_LEVEL);
  radio.setDataRate(NRF24_DATA_RATE);          
  radio.setChannel(NRF24_CHANNEL);
  radio.setCRCLength(NRF24_CRC_LENGTH);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);
  radio.openWritingPipe(rf24_tx); 
  radio.openReadingPipe(1,rf24_rx); 
  radio.stopListening();
}