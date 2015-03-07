#include <Servo.h>
#include <ax12.h>

const int PORT_SERVO = 18;
const int STBD_SERVO = 15;

const int PACKET_SIZE = 10;
byte packet[PACKET_SIZE];

const float POWER_CONVERSION = 9.0/127.0; // 90.0/127.0

Servo aft_thruster;
Servo port_thruster;
Servo stbd_thruster;

const float ANGLE_CONVERSION = 4096.0/360.0;

int heartbeat = 0;

void setup()
{
  Serial.begin(9600);

  SetPosition(PORT_SERVO, 90 * ANGLE_CONVERSION);
  SetPosition(STBD_SERVO, 90 * ANGLE_CONVERSION);

  aft_thruster.attach(12);
  port_thruster.attach(14);
  stbd_thruster.attach(13);

  aft_thruster.write(90);
  port_thruster.write(90);
  stbd_thruster.write(90);
  
  pinMode(0, OUTPUT);
  digitalWrite(0, heartbeat);
}

void loop()
{
  if(Serial.available() > PACKET_SIZE && Serial.read() == '-')
  {
    heartbeat = !heartbeat;
    digitalWrite(0, heartbeat);

    for(int i = 0; i < PACKET_SIZE; i++)
    {
      packet[i] = Serial.read();
    }
    int port_angle = (packet[0] << 8) | packet[1];
    int stbd_angle = (packet[2] << 8) | packet[3];
    int aft_power = (packet[4] << 8) | packet[5];
    int port_power = (packet[6] << 8) | packet[7];
    int stbd_power = (packet[8] << 8) | packet[9];

    SetPosition(PORT_SERVO, port_angle * ANGLE_CONVERSION);
    SetPosition(STBD_SERVO, stbd_angle * ANGLE_CONVERSION);

    aft_thruster.write(aft_power * POWER_CONVERSION + 90);
    port_thruster.write(port_power * POWER_CONVERSION + 90);
    stbd_thruster.write(stbd_power * POWER_CONVERSION + 90);
  }
}

