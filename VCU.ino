#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

#define APPS1 A0
#define APPS2 A1
#define brake A3
#define RTDS 10
#define P1 13
#define P2 12
#define SDC_IN 9
#define START_BTN 8
#define Brake_Out 7
#define SC_Relay 4 // relay out
#define AIR1 22
#define AIR2 23
#define MG1 24
#define MG2 25
#define MotorCurrent1 A10
#define MotorCurrent2 A8

#define ADDR1 0x60 // motorout1 DAC address
#define ADDR2 0x61 // motorout2 DAC address

int RTDS_TRUE = 0;
unsigned long t = 0;
int sum;

float HV_Current;
float LV_Current;
float HV_Voltage;
float LV_Voltage;
float HV_SOC;

struct can_frame canMsg;
struct can_frame IMDselfTestMsg;

#define IMD_info_ADDR 0x37
#define LVTempADDR1 0x01
#define LVTempADDR2 0x02
#define LVVoltADDR 0x03

#define Cell_Broadcast_ADDR 0x36
#define BMS_ADDR1 0x6B0
#define BMS_ADDR2 0x6B1
#define BMS_ADDR3 0x6B2

#define Sensors_ADDR 0x10 // CAN

// #define AIR1 1
// #define AIR2 2

int val1 = 0;
int val2 = 0;
int var1 = 0;
int var2 = 0;

int brakein = 0;
int motorout = 0;

MCP2515 mcp2515(53);

void DAC1(int output)
{
  Serial.print("POWER : ");
  Serial.print("output");
  Wire.beginTransmission(ADDR1);
  Wire.write(64);                 // cmd to update the DAC
  Wire.write(output >> 4);        // the 8 most significant bits…
  Wire.write((output & 15) << 4); // the 4 least significant bits…
  Wire.endTransmission();
}

void DAC2(int output)
{
  Wire.beginTransmission(ADDR2);
  Wire.write(64);                 // cmd to update the DAC
  Wire.write(output >> 4);        // the 8 most significant bits…
  Wire.write((output & 15) << 4); // the 4 least significant bits…
  Wire.endTransmission();
}

int tmap(int inp)
{
  int tmapout;
  if ((digitalRead(P1) == HIGH) && (digitalRead(P1) == HIGH))
  {
    Serial.println("5:1");
    tmapout = inp;
  }
  else if ((digitalRead(P1) == LOW) && (digitalRead(P1) == LOW))
  {
    Serial.println("5:2");
    tmapout = map(inp, 0, 4095, 0, 2047);
  }
  else
  {
    Serial.println("5:3");
    tmapout = map(inp, 0, 4095, 0, 3072);
  }
  return tmapout;
}
void control()
{
  val1 = analogRead(APPS1);
  val2 = analogRead(APPS2);
  brakein = analogRead(brake);
  // Serial.print(val2);

  //
  if (brakein > 205)
  {
    DAC1(0);
    DAC2(0);
    digitalWrite(Brake_Out, HIGH);

    Serial.print("brake:");
    Serial.println(brake);
  }

  // sensor failure (outside acceptable range)
  else if (val1 < 103 || val1 > 920 || val2 < 103 || val2 > 920)
  {
    // Serial.println("sensor failure (outside acceptable range)");
    DAC1(0);
    DAC2(0);
  }

  else if (abs(val1 - val2) > 103)
  {

    t = millis();
    while ((unsigned long)(millis() - t) < 100)
    {
      var1 = analogRead(APPS1);
      var2 = analogRead(APPS2);
      if (abs(var1 - var2) < 103)
      {
        break;
      }
    }

    if (abs(var1 - var2) > 103)
    {
      RTDS_TRUE = 0;
      Serial.println("7:APPS implausibility failure");
      DAC1(0);
      DAC2(0);

      // Serial.print("APPS implausibility failure :");
      // Serial.println(var1-var2);
    }
  }

  else
  {
    int motorpre;
    motorpre = (val1 + val2) * 2;
    motorout = tmap(motorpre);
    DAC1(motorout);
    DAC2(motorout);
    // Serial.print("ssss");
    Serial.println(motorout);
    Serial.println(val1);
    Serial.println(val2);
  }
}

void RTDS_check()
{
  if (digitalRead(START_BTN) == HIGH)
    Serial.print("Button pressed");
  {
    if (digitalRead(AIR1) == HIGH && digitalRead(AIR2) == HIGH && digitalRead(SDC_IN) == HIGH && analogRead(brake) > 512 &&
        digitalRead(MG1) == HIGH && digitalRead(MG2) == HIGH)
    {
      mcp2515.sendMessage(&IMDselfTestMsg); // IMD self test
      digitalWrite(RTDS, HIGH);
      RTDS_TRUE = 1;
    }
  }
}

void CAN()
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // polling for incoming signals
  {
    LV_BMS_CAN();
    Sensors_CAN();
    IMD_CAN();
  }
}

void LV_BMS_CAN()
{
  if (canMsg.can_id == LVTempADDR1)
  {
    Serial.print("f1:");
    for (int i = 0; i < canMsg.can_dlc; i++) // print the LV pack temp2 data
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();
  }
  if (canMsg.can_id == LVTempADDR2)
  {
    Serial.print("f2:");
    for (int i = 0; i < canMsg.can_dlc; i++) // print the LV pack temp2 data
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();
  }

  if (canMsg.can_id == LVVoltADDR)
  {
    Serial.print("c:");
    for (int i = 0; i < canMsg.can_dlc; i++) // print the LV voltage then current data
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();
  }
}

void Sensors_CAN()
{
  if (canMsg.can_id == Sensors_ADDR)
  {
    Serial.println("3:"); // speed
    Serial.println(canMsg.data[0]);

    Serial.print("a:"); // wheel speeds
    for (int i = 1; i < 5; i++)
    {
      Serial.print(canMsg.data[i]);
    }
    Serial.println();

    Serial.print("b:"); // accelaration x y z
    for (int i = 5; i < 8; i++)
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();

    Serial.print("j:"); // Motor temperature
    for (int i = 8; i < 10; i++)
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();
    Serial.print("3:"); // Eror code motor controller
    Serial.print(" ");
    Serial.println(canMsg.data[10]);
  }
}

void IMD_CAN()
{
  if (canMsg.can_id == IMD_info_ADDR)
  {
    if (canMsg.data[2] & B00000111)
    {
      Serial.println("7:IMD Failure");
    }
    Serial.print("i:");                      // sending message
    for (int i = 0; i < canMsg.can_dlc; i++) // print the data
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println();
  }
}
void BMS_CAN()
{

  if (canMsg.can_id == Cell_Broadcast_ADDR)
  {
    Serial.print("d:");
    for (int i = 0; i < canMsg.can_dlc; i++) // print Cell broadcast
    {
      Serial.print(canMsg.data[i]);
      Serial.print(" ");
    }
    Serial.println(Checksum());
  }
  if (canMsg.can_id == BMS_ADDR1)
  {
    Serial.print("K1:");
    for (int i = 0; i < canMsg.can_dlc; i++) // BMS status 1
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println(Checksum());

    HV_Current = ((canMsg.data[0] << 8) + canMsg.data[1]) / 10;

    Serial.print("m:");
    Serial.println(HV_Current);

    HV_Voltage = ((canMsg.data[2] << 8) + canMsg.data[3]) / 10;

    Serial.print("l:");
    Serial.println(HV_Voltage);

    HV_SOC = canMsg.data[4] / 2;

    Serial.print("4:");
    Serial.println(HV_Voltage);
  }

  if (canMsg.can_id == BMS_ADDR2)
  {
    Serial.print("K2:");
    for (int i = 0; i < canMsg.can_dlc; i++) // BMS status 2
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println(Checksum());
  }

  if (canMsg.can_id == BMS_ADDR3)
  {
    Serial.print("K3:");
    for (int i = 0; i < canMsg.can_dlc; i++) // BMS status 3
    {
      Serial.print(" ");
      Serial.print(canMsg.data[i]);
    }
    Serial.println(Checksum());

    if (canMsg.data[1] || canMsg.data[2] || canMsg.data[3])
    {
      Serial.println("7:BMS fault");
    }
  }
}

void setup()
{
  Serial.begin(9600); // Serial communication Intialisation
  Wire.begin();       // I2C Intialisation

  // CAN module  Intialisation
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  IMDselfTestMsg.can_id = 0x22;
  IMDselfTestMsg.can_dlc = 5;
  IMDselfTestMsg.data[0] = 0x21;
  IMDselfTestMsg.data[1] = 1;
  IMDselfTestMsg.data[2] = 0;
  IMDselfTestMsg.data[3] = 0;
  IMDselfTestMsg.data[4] = 0;

  // throttle of both motors to 0
  DAC1(0);
  DAC2(0);
  delay(10);
  digitalWrite(SC_Relay, HIGH);

  // Timer Intialistion
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 12500; // 200ms
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12  for 256  prescaler
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  pinMode(APPS1, INPUT);
  pinMode(APPS2, INPUT);
  pinMode(brake, INPUT);
  pinMode(RTDS, INPUT);
  pinMode(P1, INPUT);
  pinMode(P2, INPUT);
  pinMode(SDC_IN, INPUT);
  pinMode(START_BTN, INPUT);
  pinMode(Brake_Out, OUTPUT);
  pinMode(SC_Relay, OUTPUT); // relay out
  pinMode(AIR1, INPUT);
  pinMode(AIR2, INPUT);
  pinMode(MG1, INPUT);
  pinMode(MG2, INPUT);
  pinMode(MotorCurrent1, INPUT);
  pinMode(MotorCurrent2, INPUT);
}

void loop()
{
  TCNT1 = 0;
  if (RTDS_TRUE)
  {
    Serial.print("in main loop");

    control();
    t = millis();
    while ((unsigned long)(millis() - t) < 50)
    {
      CAN();
    }
  }

  else
  {
    RTDS_check();
    Serial.println("checking rtds");
    t = millis();
    while ((unsigned long)(millis() - t) < 50)
    {
      CAN();
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(SC_Relay, LOW);
  Serial.println("7:VCU Failure");
}

bool Checksum()
{
  sum = canMsg.can_id + canMsg.can_dlc;
  for (int i = 0; i < canMsg.can_dlc - 1; i++)
  {
    sum = sum + canMsg.data[i];
  }

  sum = sum >> 8;

  return sum == canMsg.data[canMsg.can_dlc - 1];
}
