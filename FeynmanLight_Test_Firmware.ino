#include <SPI.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

//Feynman Pin Map
#define UV_nOE        0 //PA0 
#define CAB_LED       1 //PA1 
#define RESIN_SW      2 //PA2
#define UV_SDA        3 //PA3 
#define UV_SCL        4 //PA4 
#define STEP_EN       5 //PA5 
#define LED_DAT       6 //PA6 
//#define XIN32       7 //PA7 
//#define XOUT32      8 //PA8 
#define S2_DIAG       9 //PA9
#define S0_DIAG      10 //PA10 
#define BOOT_nCS     11 //PA11 
//#define MISO       12 //PA12 
//#define MOSI       13 //PA13 
//#define SCK        14 //PA14 
#define S_FAN_12V    15 //PA15 
#define SERVO_OUT    16 //PA16 
#define RESIN_LEVEL  17 //PA17 
#define RESIN_THERM  18 //PA18 
#define RES_B_THERM  19 //PA19 
#define AIR_THERM    20 //PA20 
//#define DM         21 //PA21 
//#define DP         22 //PA22 
#define S0_STEP      23 //PA23 
#define S1_DIR       24 //PA24
#define ENDSTOP0     25 //PA25
#define ENDSTOP3     26 //PA26
#define ENDSTOP1     27 //PA27
#define R_PUMP_12V   28 //PA28 
#define S1_STEP      29 //PA29 
#define S2_nCS       30 //PA30 
#define H20_P_12V    31 //PA31 
#define ARRAY_THERM  32 //PB0 
#define ENDSTOP2     33 //PB1 
//#define ADC6       34 //PB2 
#define S1_DIAG      35 //PB3 
#define DOUT         36 //PB4
#define TRACESWO     37 //PB5
//#define SWDIO      38 //PB6
//#define SWCLK      39 //PB7
#define FCOM4_TX     40 //PB8
#define FCOM4_RX     41 //PB9
#define S2_DIR       42 //PB10 
#define S2_STEP      43 //PB11 
//#define ERASE      44 //PB12 
#define S0_nCS       45 //PB13
#define S0_DIR       46 //PB14 
#define S1_nCS       47 //PB15 

#define S0_STEP_PIN 23 //PA23
#define S1_STEP_PIN 29 //PA29
#define S2_STEP_PIN 43 //PB11
#define STEP_EN_PIN 5 //PA23

#define S0_NCS_PIN 45 //PB13
#define S1_NCS_PIN 47 //PB15
#define S2_NCS_PIN 30 //PA30

#define H20_P_12V 31 //PA31
#define R_PUMP_12V 28 //PA28
#define S_FAN_12V 15 //PA15
#define SERVO_OUT 16 //PA16

#define UV_nOE 0 //PA0

#define CAB_LED 1 //PA1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

void setup() {
analogWrite(S_FAN_12V,0);
analogWrite(CAB_LED,0);
analogWrite(UV_nOE,0);


  //Disable LED PWM Driver PCA9685
  pinMode(UV_nOE, INPUT_PULLUP);

  //Initialize MOSFETS pins
  pinMode(H20_P_12V,OUTPUT);
  digitalWrite(H20_P_12V,LOW);
  pinMode(R_PUMP_12V,OUTPUT);
  digitalWrite(R_PUMP_12V,LOW);
  pinMode(S_FAN_12V,OUTPUT);
  digitalWrite(S_FAN_12V,LOW);
  pinMode(CAB_LED,OUTPUT);
  digitalWrite(CAB_LED,LOW);

  //Initialize SERVO signal output pins
  pinMode(SERVO_OUT,OUTPUT);
  digitalWrite(SERVO_OUT,LOW);

  //Initialize TMC2130 chip select pins
  pinMode(S0_NCS_PIN,OUTPUT);
  digitalWrite(S0_NCS_PIN,HIGH);
  pinMode(S1_NCS_PIN,OUTPUT);
  digitalWrite(S1_NCS_PIN,HIGH);
  pinMode(S2_NCS_PIN,OUTPUT);
  digitalWrite(S2_NCS_PIN,HIGH);

  //Initialize communications peripherals
  SerialUSB.begin(115200);
  SPI.begin();
  tmc2130_init(S0_NCS_PIN);
  tmc2130_init(S1_NCS_PIN);
  tmc2130_init(S2_NCS_PIN);

  pinMode(STEP_EN_PIN,OUTPUT);
  digitalWrite(STEP_EN_PIN,LOW);

  pinMode(S0_STEP_PIN,OUTPUT);
  pinMode(S1_STEP_PIN,OUTPUT);
  pinMode(S2_STEP_PIN,OUTPUT);

  pinMode(S0_DIR,OUTPUT);
  pinMode(S1_DIR,OUTPUT);
  pinMode(S2_DIR,OUTPUT);
  digitalWrite(S0_DIR,LOW);
  digitalWrite(S1_DIR,LOW);
  digitalWrite(S2_DIR,LOW);

  PIO_Configure(
      g_APinDescription[PIN_WIRE_SDA].pPort,
      g_APinDescription[PIN_WIRE_SDA].ulPinType,
      g_APinDescription[PIN_WIRE_SDA].ulPin,
      g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_WIRE_SCL].pPort,
      g_APinDescription[PIN_WIRE_SCL].ulPinType,
      g_APinDescription[PIN_WIRE_SCL].ulPin,
      g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);
}

void loop() {
  byte c;

  if (SerialUSB.available() > 0) 
  {
    c = SerialUSB.read();

    //Check thermistors
    if( c == 'a' )   { 
      SerialUSB.print("RESIN_THERM: ");
      SerialUSB.println( analogRead(RESIN_THERM) );

      SerialUSB.print("ARRAY_THERM: ");
      SerialUSB.println( analogRead(ARRAY_THERM) );

      SerialUSB.print("AIR_THERM: ");
      SerialUSB.println( analogRead(AIR_THERM) );

      SerialUSB.print("RES_B_THERM: ");
      SerialUSB.println( analogRead(RES_B_THERM) );

      SerialUSB.print("RESIN_LEVEL: ");
      SerialUSB.println( analogRead(RESIN_LEVEL) );
    }

    //Hi
    if( c == 'h' )   { 
      SerialUSB.println("Hi");
    }

    // Manually init SPI peripheral
    if( c == 's' )   { 
      SerialUSB.println("spi begin");
      SPI.begin();
    }

    //Manually init tmc2130 drivers
    if( c == 'i' )   {
      tmc2130_init(S0_NCS_PIN);
      tmc2130_init(S1_NCS_PIN);
      tmc2130_init(S2_NCS_PIN);
    }

    // Toggle i2c lines to check on scope.
    if( c == 'b' ) {
      pinMode(SDA,OUTPUT);
      pinMode(SCL,OUTPUT);
      digitalWrite(SDA,HIGH);
      digitalWrite(SCL,HIGH);
      delay(1);
      digitalWrite(SDA,LOW);
      digitalWrite(SCL,LOW);
      delay(1);
      digitalWrite(SDA,HIGH);
      digitalWrite(SCL,HIGH);
      delay(1);
      digitalWrite(SDA,LOW);
      digitalWrite(SCL,LOW);
    }

    //Scan i2c for devices
    if( c == 'c' ) i2c_scan();

    //Set PWM frequency of LED DRIVER
    if( c == 'l' ) pwm.setPWMFreq(24);  // 24 to 1526
    if( c == 'm' ) pwm.setPWMFreq(1526);  // 24 to 1526

    // Turn on a channel for testing
    if( c == 'n' ) pwm.setPin(8, 500, 0);
    if( c == 'o' ) pwm.setPin(8, 1000, 0);
    if( c == 'p' ) pwm.setPin(8, 4000, 0);

    //Turn on all channels on lowest duty cycle for testing
    if( c == 'q' ) {
      leddriver_init();
      leddriver_enable();
      pwm.setPin(1, 1, 0);
      pwm.setPin(2, 1, 0);
      pwm.setPin(3, 1, 0);
      pwm.setPin(4, 1, 0);
      pwm.setPin(5, 1, 0);
      pwm.setPin(6, 1, 0);
      pwm.setPin(7, 1, 0);
      pwm.setPin(8, 1, 0);
      pwm.setPin(9, 1, 0);
      pwm.setPin(10, 1, 0);
      pwm.setPin(11, 1, 0);

      //Unused PWM channels - Test Points Only - Used to check PWM on scope
      //pwm.setPin(0, 500, 0); //TP47
      //pwm.setPin(12, 500, 0); //TP48
      pwm.setPin(13, 500, 0); //TP49
      //pwm.setPin(14, 500, 0); //TP50
      //pwm.setPin(15, 500, 0); //TP51
    }

    //Turn off all channels
    if( c == 'r' ) {
      leddriver_init();
      pwm.setPin(1, 0, 0);
      pwm.setPin(2, 0, 0);
      pwm.setPin(3, 0, 0);
      pwm.setPin(4, 0, 0);
      pwm.setPin(5, 0, 0);
      pwm.setPin(6, 0, 0);
      pwm.setPin(7, 0, 0);
      pwm.setPin(8, 0, 0);
      pwm.setPin(9, 0, 0);
      pwm.setPin(10, 0, 0);
      pwm.setPin(11, 0, 0);
      leddriver_enable();
    }

    //Individually turn on channels
    if( c == 'A' ) pwm.setPin(1, 1, 0);
    if( c == 'B' ) pwm.setPin(2, 1, 0);
    if( c == 'C' ) pwm.setPin(3, 1, 0);
    if( c == 'D' ) pwm.setPin(4, 1, 0);
    if( c == 'E' ) pwm.setPin(5, 1, 0);
    if( c == 'F' ) pwm.setPin(6, 1, 0);
    if( c == 'G' ) pwm.setPin(7, 1, 0);
    if( c == 'H' ) pwm.setPin(8, 1, 0);
    if( c == 'I' ) pwm.setPin(9, 1, 0);
    if( c == 'J' ) pwm.setPin(10, 1, 0);
    if( c == 'K' ) pwm.setPin(11, 1, 0);
  }

  
  spin_motors();

}

void spin_motors() {
  //Change direction of motors after a count of steps
  static unsigned int count=0;
  static bool direction=0;
  count++;
  if( count > (200*16) ) { //A full rotation on a 1.8deg stepper motor at 16 microstepping.
    count=0;
    direction = !direction;
    digitalWrite(S0_DIR,direction);
    digitalWrite(S1_DIR,direction);
    digitalWrite(S2_DIR,direction);
  }


  //Step the motors
  digitalWrite(S0_STEP_PIN,LOW);
  digitalWrite(S1_STEP_PIN,LOW);
  digitalWrite(S2_STEP_PIN,LOW);
  delay(1);
  digitalWrite(S0_STEP_PIN,HIGH);
  digitalWrite(S1_STEP_PIN,HIGH);
  digitalWrite(S2_STEP_PIN,HIGH);
  delay(1);
}

void tmc2130_write(uint8_t chipselect, uint8_t address,uint8_t wval1,uint8_t wval2,uint8_t wval3,uint8_t wval4)
{
  uint32_t val32;
  uint8_t val0;
  uint8_t val1;
  uint8_t val2;
  uint8_t val3;
  uint8_t val4;

  //datagram1 - write
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  digitalWrite(chipselect,LOW);
  SPI.transfer(address+0x80);
  SPI.transfer(wval1);
  SPI.transfer(wval2);
  SPI.transfer(wval3);
  SPI.transfer(wval4);
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();

  //datagram2 - response
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  digitalWrite(chipselect,LOW);
  val0 = SPI.transfer(0);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  val3 = SPI.transfer(0);
  val4 = SPI.transfer(0);
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();

  SerialUSB.print("WriteRead 0x");
  SerialUSB.print(address,HEX);
  SerialUSB.print(" Status:");
  SerialUSB.print(val0 & 0b00000111,BIN);
  SerialUSB.print("  ");
  SerialUSB.print(val1,BIN);
  SerialUSB.print("  ");
  SerialUSB.print(val2,BIN);
  SerialUSB.print("  ");
  SerialUSB.print(val3,BIN);
  SerialUSB.print("  ");
  SerialUSB.print(val4,BIN);

  val32 = (uint32_t)val1<<24 | (uint32_t)val2<<16 | (uint32_t)val3<<8 | (uint32_t)val4;
  SerialUSB.print(" 0x");
  SerialUSB.println(val32,HEX);
}

void tmc2130_chopconf(uint8_t cs, bool extrapolate256 = 1, uint16_t microstep_resolution = 16)
{
  uint8_t mres=0b0100;
  if(microstep_resolution == 256) mres = 0b0000;
  if(microstep_resolution == 128) mres = 0b0001;
  if(microstep_resolution == 64)  mres = 0b0010;
  if(microstep_resolution == 32)  mres = 0b0011;
  if(microstep_resolution == 16)  mres = 0b0100;
  if(microstep_resolution == 8)   mres = 0b0101;
  if(microstep_resolution == 4)   mres = 0b0110;
  if(microstep_resolution == 2)   mres = 0b0111;
  if(microstep_resolution == 1)   mres = 0b1000;

  mres |= extrapolate256 << 4; //bit28 intpol

  tmc2130_write(cs,0x6C,mres,1,00,0xC5);
}

void tmc2130_init(uint8_t cspin)
{
  /*
  uint8_t cs[4] = { X_TMC2130_CS, Y_TMC2130_CS, Z_TMC2130_CS, E0_TMC2130_CS };
  uint8_t current[4] = { 23, 23, 23, 23 };

  digitalWrite(X_TMC2130_CS, HIGH);
  digitalWrite(Y_TMC2130_CS, HIGH);
  digitalWrite(Z_TMC2130_CS, HIGH);
  digitalWrite(E0_TMC2130_CS, HIGH);
  pinMode(X_TMC2130_CS,OUTPUT);
  pinMode(Y_TMC2130_CS,OUTPUT);
  pinMode(Z_TMC2130_CS,OUTPUT);
  pinMode(E0_TMC2130_CS,OUTPUT);
  */

  uint8_t CURRENT0 = 17;

  pinMode(cspin,OUTPUT);
  digitalWrite(cspin,HIGH);
  tmc2130_chopconf(cspin,1,16); // 16 Microstepping to 256 microstepping
  tmc2130_write(cspin,0x10,0,15,CURRENT0,CURRENT0); //0x10 IHOLD_IRUN
  tmc2130_write(cspin,0x0,0,0,0,0b101); //address=0x0 GCONF EXT VREF - STEALTH CHOP
  tmc2130_write(cspin,0x70,0,0b111,0x01,0xC8); //address=0x70 PWM_CONF //reset default=0x00050480



/*
  for(int i=0;i<4;i++)
  {
    //tmc2130_write(cs[i],0x6C,0b10100,01,00,0xC5);
    tmc2130_chopconf(cs[i],1,16); // 16 Microstepping to 256 microstepping
    tmc2130_write(cs[i],0x10,0,15,current[i],current[i]); //0x10 IHOLD_IRUN
    tmc2130_write(cs[i],0x0,0,0,0,0b101); //address=0x0 GCONF EXT VREF - STEALTH CHOP
    //tmc2130_write(cs[i],0x11,0,0,0,0xA);
    tmc2130_write(cs[i],0x70,0,0b111,0x01,0xC8); //address=0x70 PWM_CONF //reset default=0x00050480
  }
  */
}

void i2c_scan()
{
  byte error, address;
  int nDevices;
 
  SerialUSB.println("Scanning...");
  Wire.begin();
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      SerialUSB.print("I2C device found at address 0x");
      if (address<16)
        SerialUSB.print("0");
      SerialUSB.print(address,HEX);
      SerialUSB.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      SerialUSB.print("Unknown error at address 0x");
      if (address<16)
        SerialUSB.print("0");
      SerialUSB.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    SerialUSB.println("No I2C devices found\n");
  else
    SerialUSB.println("done\n");
}

void leddriver_init() {
  pwm.begin();
  pwm.setPWMFreq(1526);  // This is the maximum PWM frequency
}

void leddriver_enable() {
  //Enable LED PWM Driver PCA9685
  pinMode(UV_nOE, OUTPUT);
  digitalWrite(UV_nOE, LOW);
}

