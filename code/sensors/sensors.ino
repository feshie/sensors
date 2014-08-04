// Setup the pin muxing for the fake I2C
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7
#define I2C_TIMEOUT 10

#include <SoftI2CMaster.h>
#include <Wire.h>
#include <EEPROM.h>

#include "sensors.h"
#include "TAD.h"
#include "buffer.h"
#include "pb_encode.h"

#include <util/crc16.h>

#define OPCODE_ECHO 0x00
#define OPCODE_LISTSENSORS 0x01
#define OPCODE_GETDATA 0x02
#define OPCODE_SETGAIN 0x03
#define OPCODE_RESPONSE 0xFF

#define MASTER_ADDRESS 0x00

//Pin for toggling rx/tx
#define IO_PIN 2

//globals for sensor and id info
byte avrID;
byte attachedSensor;

uint8_t RESPONSE_HEADER[] = {
  MASTER_ADDRESS, OPCODE_RESPONSE};
uint16_t CRC_START = 0xFFFF;

struct Message {
  byte address;
  byte opcode;
  void * payload;
  uint16_t payloadLength;
};

//Respond to a GetData request
void getData(struct Message *rq) {
  Rs485 message;
  message.dst = 0;
  message.type = Rs485_Type_DATA;
  message.has_sensor = true;
  
  if (attachedSensor & SENSOR_TSPIDER) {
    //TODO
    message.sensor = Rs485_Sensor_OW;
    message.ow_count = 0;
  } else {
    message.ow_count = 0;
  }
  
  if (attachedSensor & SENSOR_WPRESSURE) {
    //TODO
    message.sensor = Rs485_Sensor_ADC0;
    message.ad_count = 0;
  } else {
    message.ad_count = 0;
  }
  
  if (attachedSensor & SENSOR_TACHAIN) {
    message.sensor = Rs485_Sensor_TA_CHAIN;
    message.tad_count = 4;
    readTAD(TMP_ADDR1, ACCEL_ADDR1, false, 1, &message.tad[0]);
    readTAD(TMP_ADDR2, ACCEL_ADDR2, false, 2, &message.tad[1]);
    readTAD(TMP_ADDR1, ACCEL_ADDR1, true, 3, &message.tad[2]);
    readTAD(TMP_ADDR2, ACCEL_ADDR2, true, 4, &message.tad[3]);
  } else {
     message.tad_count = 0; 
  }

  digitalWrite(IO_PIN, HIGH);
  pb_ostream_t stream = {
    &serialWriter,(void*)CRC_START,512,0,0  };
  pb_write(&stream, RESPONSE_HEADER, 2);
  bool status = pb_encode(&stream, Rs485_fields, &message);
  uint16_t crc = (uint16_t)stream.state;
  Serial.write(crc & 0xFF);
  Serial.write((crc >> 8) & 0xFF);
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}

void writeMessage(Message *m) {
  digitalWrite(IO_PIN, HIGH);
  pb_ostream_t stream = {&serialWriter,(void*)CRC_START,512,0,0  };
  pb_write(&stream, &(m->address), 1);
  pb_write(&stream, &(m->opcode), 1);
  pb_write(&stream, (uint8_t*)m->payload, m->payloadLength);
  uint16_t crc = (uint16_t)stream.state;
  Serial.write(crc & 0xFF);
  Serial.write((crc >> 8) & 0xFF);
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}

//Respond to a ListSensors request
void listSensors(struct Message *rq) {
  rq->address = MASTER_ADDRESS;
  rq->opcode = OPCODE_RESPONSE;
  
  if (attachedSensor & SENSOR_TACHAIN) {
      rq->payload = (void*)"TACHAIN";
  } else if (attachedSensor & SENSOR_TSPIDER) {
    rq->payload = (void*)"TSPIDER";
  } else if (attachedSensor & SENSOR_WPRESSURE) {
    rq->payload = (void*)"TWPRESSURE";
  }
  rq->payloadLength = strlen((char*)rq->payload);
  
  writeMessage(rq);
}

//Respond to an Echo request
void echo(struct Message *rq) {
  rq->address = MASTER_ADDRESS;
  rq->opcode = OPCODE_RESPONSE;
  
  writeMessage(rq);
}

bool serialWriter(pb_ostream_t *stream, const uint8_t *buf, size_t count) {  
  while (count--) {
    uint8_t b = *buf++;
    Serial.write(b);
    stream->state = (void*) _crc16_update((uint16_t) stream->state, b);
  }

  return true;
}

uint16_t computeCRC(unsigned char * data, int length) {
  uint16_t crc = CRC_START;
  while (length--)
    crc = _crc16_update(crc, *data++);
  return crc;
}

void setup(void) {
  Serial.begin(9600);
  Serial.setTimeout(100);
  Wire.begin();
  i2c_init();

  avrID = EEPROM.read(0);
  attachedSensor = EEPROM.read(1);

  pinMode(IO_PIN, OUTPUT);
}

void handleMessage(struct Message *request) {
  switch (request->opcode) {
  case OPCODE_ECHO:
    echo(request);
    break; 
  case OPCODE_GETDATA:
    getData(request);
    break;
  case OPCODE_LISTSENSORS:
    listSensors(request);
    break;
 }
}

void serialDebug(char * msg) {
  digitalWrite(IO_PIN, HIGH);
  Serial.println(msg);
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}

unsigned char readbuffer[512];

void loop(void) {
  int bytesread = Serial.readBytes((char*)readbuffer, 512);

  if (bytesread>0) {
    //check that this message is for us
    if (readbuffer[0] == avrID) {
      uint16_t crc = computeCRC(readbuffer, bytesread - 2);
      uint8_t crclow = crc & 0xFF;
      uint8_t crchigh = (crc >> 8) & 0xFF;

      //now check the CRC to ensure the message is valid
      if (crclow==readbuffer[bytesread-2] && crchigh==readbuffer[bytesread-1]) {
        struct Message request;
        request.address = readbuffer[0];
        request.opcode = readbuffer[1];
        request.payload = &(readbuffer[2]);
        request.payloadLength = bytesread - 4;

        handleMessage(&request);
      }
    }
  }
}


