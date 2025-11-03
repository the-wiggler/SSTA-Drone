#define RXD1 44
#define TXD1 43 

HardwareSerial mySerial(1);

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial.println("Serial passthrough started!\n");
}

void loop() {
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
