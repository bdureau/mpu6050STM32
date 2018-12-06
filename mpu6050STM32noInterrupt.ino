/*
 * Author: Boris du Reau
 * Date: october 2018
 * Description: This is juste a demo program to use an accelerometer in a model rocket.
 * Code as been optimized to use an STM32F103C running the Arduino environment but can be modified to use any other
 * Arduino board.
 * You will need to install the MPU6050 library. 
 * The gyroscope/ accelerometer board I am using is GY-521 but any other MPU6050 based board should do
 * I am also using a bluetooth module to comunicate with an Android device such as a phone or a tablet
 * This version does not use an interrupt and can be calibrated by sending the c; command
 * I am using Serial1 with the STM32 board but you can use any other serial port
 */

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   does not depends on the MPU-6050's INT pin 
   ========================================================================= */

#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//calibration stuff
//Change those 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 200;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize Serial1 communication
  // (38400 chosen because it is required for the bluetooth 
  //Serial1.begin(115200);
  Serial1.begin(38400);
  while (!Serial1); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial1.println(F("Initializing I2C devices..."));
  mpu.initialize();
 
  // verify connection
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial1.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial1.available() && Serial1.read()); // empty buffer
  while (!Serial1.available());                 // wait for data
  while (Serial1.available() && Serial1.read()); // empty buffer again

  // load and configure the DMP
  Serial1.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // you can recalibrate it later
  mpu.setXAccelOffset(1094);
  mpu.setYAccelOffset(328);
  mpu.setZAccelOffset(1272);
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(-25);
 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial1.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial1.print(F("DMP Initialization failed (code "));
    Serial1.print(devStatus);
    Serial1.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop(void)
{
  MainMenu();
}
void myloop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ( fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial1.println(F("FIFO overflow!"));

    // otherwise we are good to go
  } else {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    float q1[4];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    q1[0] = q.w;
    q1[1] = q.x;
    q1[2] = q.y;
    q1[3] = q.z;
    serialPrintFloatArr(q1, 4);

    Serial1.println("");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

// ================================================================
// === Those 2 functions will format the data                   ===
// ================================================================
void serialPrintFloatArr(float * arr, int length) {
  for (int i = 0; i < length; i++) {
    serialFloatPrint(arr[i]);
    Serial1.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial1.print(c1);
    Serial1.print(c2);
  }
}




/*

   calibration routines those will be executed each time the board is powered up.
   we might want to calibrate it for good on a flat table and save it to the microcontroler eeprom 

*/
void calibrate() {
  // start message
  Serial1.println("\nMPU6050 Calibration Sketch");
  Serial1.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");

  // verify connection
  Serial1.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
      Serial1.println("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(100);
    }

    if (state == 1) {
      Serial1.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(100);
    }

    if (state == 2) {
      meansensors();
      Serial1.println("\nFINISHED!");
      Serial1.print("\nSensor readings with offsets:\t");
      Serial1.print(mean_ax);
      Serial1.print("\t");
      Serial1.print(mean_ay);
      Serial1.print("\t");
      Serial1.print(mean_az);
      Serial1.print("\t");
      Serial1.print(mean_gx);
      Serial1.print("\t");
      Serial1.print(mean_gy);
      Serial1.print("\t");
      Serial1.println(mean_gz);
      Serial1.print("Your offsets:\t");
      Serial1.print(ax_offset);
      Serial1.print("\t");
      Serial1.print(ay_offset);
      Serial1.print("\t");
      Serial1.print(az_offset);
      Serial1.print("\t");
      Serial1.print(gx_offset);
      Serial1.print("\t");
      Serial1.print(gy_offset);
      Serial1.print("\t");
      Serial1.println(gz_offset);
      Serial1.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial1.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial1.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      break;
    }
  }
}


void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    //Serial1.println("...");
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}


//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[200];

  

    while (Serial1.available())
    {
      readVal = Serial1.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  
  if (commandbuffer[0]!='\0'){
    interpretCommandBuffer(commandbuffer);
    commandbuffer[0]='\0';
  }
  myloop();
}

void interpretCommandBuffer(char *commandbuffer) {
  //Serial1.println((char*)commandbuffer);
  //this will erase all flight
  if (commandbuffer[0] == 'c')
  {
    Serial1.println(F("calibration\n"));
    // Do calibration suff
   state=0;
  
   calibrate();
   mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  
  }
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
  }
}
