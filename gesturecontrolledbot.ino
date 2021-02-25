#include <Wire.h>
  // Connect MPU6050 pins SCL & SDA to A5 & A4 respectively.
  const int trigPin = 6;                      // Connects to trig pin of SR04.
  const int echoPin = 7;                      // Connects to the echo pin of SR04.
  int duration;                               // Defining variable to record echo time duration.
  int distance;                               // Defining variable to record frontal distance.
  const int leftForward = 2;                  // Connect to pin 2 of L293D.
  const int leftBackward = 3;                 // Connect to pin 7 of L293D.
  const int rightForward = 4;                 // Connect to pin 15 of L293D.
  const int rightBackward = 5;                // Connect to pin 10 of L293D.

  long accelX, accelY, accelZ;                // Defining variables to read
  float gForceX, gForceY, gForceZ;            // the accelerometer data.

void setup() {
  pinMode(trigPin, OUTPUT);                   // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);                    // Sets the echoPin as an Input
  pinMode(leftForward,OUTPUT);                // Sets these pins as output 
  pinMode(leftBackward,OUTPUT);               // to control left and right
  pinMode(rightForward,OUTPUT);               // motors.
  pinMode(rightBackward,OUTPUT);
  Serial.begin(9600);                         // Begins serial transmission with 9600 baud rate.
  Wire.begin();                               // To initiate i2c comms b/w MPU6050 and Arduino.
  setupMPU();
}


void loop() {
  digitalWrite(trigPin, LOW);                 // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);                // Creating trigger pulse for
  delayMicroseconds(10);                      // 10us as required by SR04.
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);          // Time for returing wave in us.
  float distance = duration*0.034/2;          // Calculating Distance in cm.
  Serial.print("Distance = ");
  Serial.println(distance);
  recordAccelRegisters();                     // Calling function to record Accelerometer Data
  digitalWrite(leftForward,LOW);              // Resetting control pins to clear any previous
  digitalWrite(leftBackward,LOW);             // motor commands.
  digitalWrite(rightForward,LOW);
  digitalWrite(rightBackward,LOW); 
  if((distance > 7) || (distance < 0))        // Safety measure to prevent bot crash.
  {
    commandProcess();                         // To send apropriate control commands.
    Serial.println(gForceX);                  // For Debugging.
    Serial.println(gForceY);                  //
    Serial.println(gForceZ);                  //
  }
  else 
  {
    if((gForceX < -0.27) && (gForceX < gForceY) && (gForceX < -gForceY)) 
    {
      // To enable backward movement when forward distance is less.
      Serial.println("Backward");
      digitalWrite(leftForward,LOW);
      digitalWrite(leftBackward,HIGH);
      digitalWrite(rightForward,LOW);
      digitalWrite(rightBackward,HIGH);
    }
    if((gForceX < 0.27) && (gForceX > -0.27) && (gForceY < 0.27) && (gForceY > -0.27)) 
    {
      // Stall commands.
      digitalWrite(leftForward,LOW);
      digitalWrite(leftBackward,LOW);
      digitalWrite(rightForward,LOW);
      digitalWrite(rightBackward,LOW); 
    }
  }
}

void setupMPU(){
  Wire.beginTransmission(0b1101000);          //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                           //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);                     //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000);          //I2C address of the MPU
  Wire.write(0x1B);                           //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000);                     //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000);          //I2C address of the MPU
  Wire.write(0x1C);                           //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000);                     //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);          //I2C address of the MPU
  Wire.write(0x3B);                           //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);              //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();        //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read();        //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read();        //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){ 
  gForceX = accelX / 16384.0;                 // Conditioning of Accelerometer 
  gForceY = accelY / 16384.0;                 // data to get values from -1 to 1.
  gForceZ = accelZ / 16384.0;
}

void commandProcess() {
  if((gForceX > 0.27) && (gForceX > gForceY) && (gForceX > -gForceY)) 
  {
    // Forward motion commands.
    Serial.println("Forward"); // For debugging.
    digitalWrite(leftForward,HIGH);
    digitalWrite(leftBackward,LOW);
    digitalWrite(rightForward,HIGH);
    digitalWrite(rightBackward,LOW);
  }
  if((gForceX < -0.27) && (gForceX < gForceY) && (gForceX < -gForceY)) 
  {
    // Backward motion commands.
    Serial.println("Backward"); // For debugging.
    digitalWrite(leftForward,LOW);
    digitalWrite(leftBackward,HIGH);
    digitalWrite(rightForward,LOW);
    digitalWrite(rightBackward,HIGH);
  }
  if((gForceY > 0.27) && (gForceY > gForceX) && (gForceY > -gForceX)) 
  {
    // Left motion command.
    Serial.println("Left"); // For debugging.
    digitalWrite(leftForward,LOW);
    digitalWrite(leftBackward,HIGH);
    digitalWrite(rightForward,HIGH);
    digitalWrite(rightBackward,LOW);
  }
  if((gForceY < -0.27) && (gForceY < gForceX) && (gForceY < -gForceX)) 
  {
    // Right motion command.
    Serial.println("Right"); // For debugging.
    digitalWrite(leftForward,HIGH);
    digitalWrite(leftBackward,LOW);
    digitalWrite(rightForward,LOW);
    digitalWrite(rightBackward,HIGH);
  }
  if((gForceX < 0.27) && (gForceX > -0.27) && (gForceY < 0.27) && (gForceY > -0.27)) 
  {
    // Resetting motor commands.
    digitalWrite(leftForward,LOW);
    digitalWrite(leftBackward,LOW);
    digitalWrite(rightForward,LOW);
    digitalWrite(rightBackward,LOW); 
  }
  Serial.println();
  delay(350);
}
