#include "SerialCommand.h"
#include "EEPROMex.h"
#include "median3.h"
#include "PID.h"

#define SOFTWARE_VERSION          "v0.1"
#define SOFTWARE_NAME             "FAN_CONTROL"

#define ADDR_EEPROM_P             0
#define ADDR_EEPROM_I             4
#define ADDR_EEPROM_D             8
#define ADDR_EEPROM_DZONE         12
#define ADDR_EEPROM_CONTROL_MAX   16
#define ADDR_EEPROM_CONTROL_MIN   20
#define ADDR_EEPROM_TARGET        24
#define ADDR_EEPROM_SPEED         28

#define CONTROL_PIN               3
#define FEEDBACK_PIN              A4

#define PIN_DIRECTION_A           12
#define PIN_SPEED_A               CONTROL_PIN
#define PIN_BRAKE_A               9
#define PIN_CURRENT_SENSING_A     A0

#define PIN_DIRECTION_B           13
#define PIN_SPEED_B               11
#define PIN_BRAKE_B               8
#define PIN_CURRENT_SENSING_B     A1

#define PIN_INFO_LED_GREEN        5
#define PIN_INFO_LED_RED          6

SerialCommand sCmd;
struct PIDdata pid;
GMedian3<int> sensorFilter;

unsigned long previousMillis = 0;
const long interval = 5;

bool run_control = false;
bool loging = false;

void setup()
{
  pinMode(PIN_DIRECTION_A, OUTPUT);     // настройка канала A
  digitalWrite(PIN_DIRECTION_A, LOW);   //
                                        //
  pinMode(PIN_BRAKE_A, OUTPUT);         //
  digitalWrite(PIN_BRAKE_A, LOW);       //
                                        //
  pinMode(PIN_SPEED_A, OUTPUT);         //
  analogWrite(PIN_SPEED_A, 0);          //

                                        // канал B выключен
  pinMode(PIN_DIRECTION_B, OUTPUT);     //
  digitalWrite(PIN_DIRECTION_B, LOW);   //
                                        //
  pinMode(PIN_BRAKE_B, OUTPUT);         //
  digitalWrite(PIN_BRAKE_B, HIGH);      //
                                        //
  pinMode(PIN_SPEED_B, OUTPUT);         //
  analogWrite(PIN_SPEED_B, 0);          //
                              
 // pinMode(CONTROL_PIN, OUTPUT);
 // analogWrite(CONTROL_PIN, 0);

  pinMode(PIN_INFO_LED_RED, OUTPUT);
  pinMode(PIN_INFO_LED_GREEN, OUTPUT);
  
  Serial.begin(115200);

  sCmd.addCommand("set_p",    set_P);
  sCmd.addCommand("set_i",   set_I);
  sCmd.addCommand("set_d", set_D);

  sCmd.addCommand("get_p",    get_P);
  sCmd.addCommand("get_i",   get_I);
  sCmd.addCommand("get_d", get_D);

  sCmd.addCommand("set_target",   set_Target);
  sCmd.addCommand("get_target", get_Target);

  sCmd.addCommand("set_dzone",   set_dzone);
  sCmd.addCommand("get_dzone", get_dzone);

  sCmd.addCommand("set_min_out",   set_min_out);
  sCmd.addCommand("get_min_out", get_min_out);

  sCmd.addCommand("set_max_out",   set_max_out);
  sCmd.addCommand("get_max_out", get_max_out);

  sCmd.addCommand("on_control", on_Control);
  sCmd.addCommand("off_control", off_Control);

  sCmd.addCommand("on_log", on_Log);
  sCmd.addCommand("off_log", off_Log);

  sCmd.addCommand("reset", reset);
  sCmd.addCommand("set_speed", set_speed);
  sCmd.addCommand("info", info);
  sCmd.addCommand("who", who);

  sCmd.setDefaultHandler(unrecognized);

  initPID(EEPROM.readFloat(ADDR_EEPROM_P), EEPROM.readFloat(ADDR_EEPROM_I), EEPROM.readFloat(ADDR_EEPROM_D), EEPROM.readFloat(ADDR_EEPROM_DZONE), EEPROM.readFloat(ADDR_EEPROM_CONTROL_MAX), EEPROM.readFloat(ADDR_EEPROM_CONTROL_MIN), EEPROM.readFloat(ADDR_EEPROM_TARGET), &pid);

}

void loop()
{
  float sensorValue;
  float control;

  sCmd.readSerial();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    sensorValue = sensorFilter.filtered(map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255));
    //sensorValue = map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255);

    if (run_control)
    {
      control = updatePID(sensorValue, &pid);

      digitalWrite(PIN_INFO_LED_RED, LOW);
      digitalWrite(PIN_INFO_LED_GREEN, LOW);

      if ((pid.target - sensorValue) > pid.dZone)
      {
        digitalWrite(PIN_INFO_LED_RED, HIGH);
        digitalWrite(PIN_INFO_LED_GREEN, LOW);
      }

      if ((pid.target - sensorValue) < -pid.dZone)
      {
        digitalWrite(PIN_INFO_LED_RED, LOW);
        digitalWrite(PIN_INFO_LED_GREEN, HIGH);
      }

    }
    else
    {
      digitalWrite(PIN_INFO_LED_RED, LOW);
      digitalWrite(PIN_INFO_LED_GREEN, LOW);
      control = 0;
    }

    analogWrite(CONTROL_PIN, control);

    if (loging)
    {
      Serial.print(pid.target);
      Serial.print(",");
      Serial.print(sensorValue);
      Serial.print(",");
      Serial.print(pid.target - sensorValue);
      Serial.print(",");
      Serial.print(control);
      Serial.println();
    }
  }
}

void set_P()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.P_Factor = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_P, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_I()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.I_Factor = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_I, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_D()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.D_Factor = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_D, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_dzone()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.dZone = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_DZONE, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_min_out()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.controlMin = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_CONTROL_MIN, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_max_out()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    pid.controlMax = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_CONTROL_MAX, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void set_Target()
{
  float aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    aNumber = atof(arg);
    aNumber = constrain(aNumber, 0.0, 255.0);
    pid.target = aNumber;
    resetIntegralError(&pid);
    EEPROM.writeFloat(ADDR_EEPROM_TARGET, aNumber);
  }
  else
  {
    Serial.println("No arguments");
    return;
  }
}

void get_P()
{
  Serial.println(pid.P_Factor);
}

void get_I()
{
  Serial.println(pid.I_Factor);
}

void get_D()
{
  Serial.println(pid.D_Factor);
}

void get_dzone()
{
  Serial.println(pid.dZone);
}

void get_min_out()
{
  Serial.println(pid.controlMin);
}

void get_max_out()
{
  Serial.println(pid.controlMax);
}

void get_Target()
{
  Serial.println(pid.target);
}

void on_Control()
{
  run_control = true;
  resetIntegralError(&pid);
}

void off_Control()
{
  run_control = false;
  resetIntegralError(&pid);
}

void on_Log()
{
  loging = true;
}

void off_Log()
{
  loging = false;
}


void reset()
{
  asm volatile ("  jmp 0");
}

void set_speed() {
  int aNumber;
  long speed_port;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
  }
  else {
    Serial.println("No arguments");
    return;
  }
  // 0 - 110
  // 1 - 300
  // 2 - 600
  // 3 - 1200
  // 4 - 2400
  // 5 - 4800
  // 6 - 9600
  // 7 - 14400
  // 8 - 19200
  // 9 - 28800
  // 10 - 38400
  // 11 - 57600
  // 12 - 115200
  switch ( aNumber ) {
    case 0:
      speed_port = 110;
      break;
    case 1:
      speed_port = 300;
      break;
    case 2:
      speed_port = 600;
      break;
    case 3:
      speed_port = 1200;
      break;
    case 4:
      speed_port = 2400;
      break;
    case 5:
      speed_port = 4800;
      break;
    case 6:
      speed_port = 9600;
      break;
    case 7:
      speed_port = 14400;
      break;
    case 8:
      speed_port = 19200;
      break;
    case 9:
      speed_port = 28800;
      break;
    case 10:
      speed_port = 38400;
      break;
    case 11:
      speed_port = 57600;
      break;
    case 12:
      speed_port = 115200;
      break;
    default:
      Serial.println("Bad arguments");
      return;
      break;
  }

  EEPROM.writeLong(ADDR_EEPROM_SPEED, speed_port);
  Serial.begin(speed_port);
}

void info()
{
  Serial.println();
  Serial.print(SOFTWARE_NAME);
  Serial.print(" ");
  Serial.println(SOFTWARE_VERSION);
  Serial.println();
  Serial.println("PID parametrs :");
  Serial.print("Kp : ");
  Serial.println(pid.P_Factor);
  Serial.print("Ki : ");
  Serial.println(pid.I_Factor);
  Serial.print("Kd : ");
  Serial.println(pid.D_Factor);
  Serial.print("DZone : ");
  Serial.println(pid.dZone);
  Serial.print("ControlMax : ");
  Serial.println(pid.controlMax);
  Serial.print("ControlMin : ");
  Serial.println(pid.controlMin);
  Serial.print("Target : ");
  Serial.println(pid.target);
  Serial.println();
}

void who()
{
  Serial.println(SOFTWARE_NAME);
}
void unrecognized(const char *command)
{
  Serial.println("Not found command");
}
