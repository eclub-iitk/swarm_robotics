/*

UDPSendReceive.pde:

This sketch receives UDP message strings, prints them to the serial port

and sends an "acknowledge" string back to the sender.

*/

int x1 =5;

const int ProxSensor=12; //D6

int inputVal = 0;

#include <ESP8266WiFi.h>

#include <WiFiUdp.h>

#ifndef STASSID

#define STASSID "Electronics Club"

#define STAPSK "RandomShit"

#endif

unsigned int localPort = 5007; // local port to listen on

// buffers for receiving and sending data

char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

char ReplyBuffer[] = "acknowledged\r\n"; // a string to send back

WiFiUDP Udp;

const int EnA = 4; //D2

const int EnB = 14; //D5

const int In1 = 16; //D0

const int In2 = 5; //D1

const int In3 = 0; //D3

const int In4 = 2; //D4

void setup() {

pinMode(EnA, OUTPUT);

pinMode(EnB, OUTPUT);

pinMode(In1, OUTPUT);

pinMode(In2, OUTPUT);

pinMode(In3, OUTPUT);

pinMode(In4, OUTPUT);

pinMode(ProxSensor,INPUT); //Pin 2 is connected to the output of proximity sensor

Serial.begin(115200);

WiFi.mode(WIFI_STA);

WiFi.begin(STASSID, STAPSK);

while (WiFi.status() != WL_CONNECTED) {

Serial.print('.');

delay(500);

}

Serial.print("Connected! IP address: ");

Serial.println(WiFi.localIP());

Serial.printf("UDP server on port %d\n", localPort);

Udp.begin(localPort);

}

void wait(){

digitalWrite(In1, LOW);

digitalWrite(In2,LOW);

digitalWrite(In3, LOW);

digitalWrite(In4, LOW);

}

void back() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1, LOW);

digitalWrite(In2,HIGH);

// set speed to 150 out 255

analogWrite(EnA, 400);

// turn on motor B

digitalWrite(In3, LOW);

digitalWrite(In4, HIGH);

// set speed to 150 out 255

analogWrite(EnB, 400);

}

void right() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1, HIGH);

digitalWrite(In2, LOW);

// set speed to 150 out 255

analogWrite(EnA, 250);

// turn on motor B

digitalWrite(In3, LOW);

digitalWrite(In4, HIGH);

// set speed to 150 out 255

analogWrite(EnB, 250);

}

void left() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1,LOW);

digitalWrite(In2,HIGH);

// set speed to 150 out 255

analogWrite(EnA, 250);

// turn on motor B

digitalWrite(In3,HIGH);

digitalWrite(In4, LOW);

// set speed to 150 out 255

analogWrite(EnB, 250);

}

void forward() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In2, LOW);

digitalWrite(In1,HIGH);

// set speed to 150 out 255

analogWrite(EnA, 250);

// turn on motor B

digitalWrite(In4, LOW);

digitalWrite(In3, HIGH);

// set speed to 150 out 255

analogWrite(EnB, 250);

}

void impulse() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1,HIGH);

digitalWrite(In2,LOW);

// set speed to 150 out 255

analogWrite(EnA, 350);

// turn on motor B

digitalWrite(In3,HIGH);

digitalWrite(In4, LOW);

// set speed to 150 out 255

analogWrite(EnB, 350);

}

void lefti() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1,LOW);

digitalWrite(In2,HIGH);

// set speed to 150 out 255

analogWrite(EnA, 350);

// turn on motor B

digitalWrite(In3,HIGH);

digitalWrite(In4, LOW);

// set speed to 150 out 255

analogWrite(EnB, 350);

}

void righti() //run both motors in the same direction

{

// turn on motor A

digitalWrite(In1, HIGH);

digitalWrite(In2, LOW);

// set speed to 150 out 255

analogWrite(EnA, 350);

// turn on motor B

digitalWrite(In3, LOW);

digitalWrite(In4, HIGH);

// set speed to 150 out 255

analogWrite(EnB, 350);

}

char x= '0';

void loop() {

inputVal = digitalRead(ProxSensor);

//Serial.println(inputVal);

//delay(1000); // wait for a second

// if there's data available, read a packet

int packetSize = Udp.parsePacket();

if (packetSize) {

Serial.printf("Received packet of size %d from %s:%d\n (to %s:%d, free heap = %d B)\n",

packetSize,

Udp.remoteIP().toString().c_str(), Udp.remotePort(),

Udp.destinationIP().toString().c_str(), Udp.localPort(),

ESP.getFreeHeap());

// read the packet into packetBufffer

int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

Serial.println("Contents:");

Serial.println(packetBuffer[0]);

Serial.print("x:");

x = packetBuffer[0];

Serial.print(x);

packetBuffer[n] = 0;

if(x != x1){

wait();

if(inputVal == 1){

righti();

delay(1000);

impulse();

}

else if (x=='0'){

forward();

}

else if(x=='4'){

wait();

}

else if(x=='2'){

right();

}

else if(x=='3'){

left();

}

else if(x=='5'){

impulse();

}

else if(x=='6'){

righti();

}

else if(x=='7'){

lefti();

}

}

x1 = x;

}

}
