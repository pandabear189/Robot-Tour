#include <Encoder.h>
#include <PID_v1_bc.h>
#include <SparkFun_TB6612.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <gyro.h>
#include <HCSR04.h>

// HCSR04 leftDist(37, 35);
// HCSR04 rightDist(41, 39);



// #define encAYellow 1
// #define encAGreen 1
// #define encBYellow 1
// #define encBGreen 1
// Encoder knobLeft(encAYellow, encAGreen);
// Encoder knobRight(encBYellow, encBGreen);


// #define AIN1 1
// #define AIN2 1
// #define BIN1 1
// #define BIN2 1
// #define PWMA 1
// #define PWMB 1
// #define STBY 1

#define enc1A 19
#define enc1B 18
#define enc2A 3
#define enc2B 2

//motor1 = left

#define button 49

#define AIN1 15
#define BIN1 40
#define AIN2 14
#define BIN2 38
#define PWMA 6
#define PWMB 7
#define STBY 34

Encoder knobLeft(enc1A, enc1B);
Encoder knobRight(enc2A, enc2B);

const int offsetA = -1;
const int offsetB = -1;

Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

Adafruit_ICM20948 icm;
double currAngle;
int sum;
gyro angle = gyro(&icm, &sum, &currAngle);

//Eric add in the path variable definitions here
//R: Right Turn
//L: Left Turn
//S: start move
//E: end move

//F: 1 forward moves
//2: 2 forward moves
//3: 3 forward moves

//B: 1 reverse move
//Y: 2 reverse move, n
//Z: 3 reverse moves m

//SRFL3RRFRFRRFRFRFRRFRFR3R3RR2LFL2RE mit path
// SL2RR3L2LFLFRR2L2RFRFRRFL2RRFR2LFRFRFRRFLFL3RFLFRFRE
//SRFLFLFRFRFLFLFL2E
//S2LFL2RR3RFRRFLFL3LFLFRRFR3RRFLE state
//S2LFL2RR3RFRRFLFL3LFLFRRFR3RRFLE
//SFLLFLLFLLFLLFLLFLLF
//SR2RR3RFLFLFRRFR2L2RRFLFRRFRFLFE palatine 24
//prac SLFRRFLFLFRFLFRRFRF
// SL2RR3L2LFLFRR2L2RFRFLLFL2RRFR2LFRFRFLLFLFL3RFLFRFRE
String path = "333";
// SR2L3L3L2LFL2R2RFRFRR2LFRRFRFL2LFLFRRFRFR3L3LFE penn
//SLFRFR2RFLFRRFRFLFR3LFL2L3RFRRFL2RFRFLFRFRFRRFLFLFRFLFE solon main
// SLFRFR2RFLFRRFRFLFRFR2RFRRFL2R2LFRRFRFLFLFRFRFRRFLFLFRFLFE solon alternative
// SR2L3L3L2LFL2R2RFRFRR2LFRRFRFL2LFLFRRFRFR3L3LFE idk
// SR2LFRFRFRR2L3RFRFLFLFRRFRFLFLFRFR3RE
int targetTime = 80;
// SR2L3LFRFLFLFRFLFRRFRFR2LFRFLFLE
int mySpeed = 130; //set to 0 to auto calculate speed

double xOffset = 0; //left = -, right = +
double yOffset = 0; //ba4.5ck = -, forward = +

double timeOff = 0; //adjusts start/end move speed to offset small offset

// double currDelta = 0;
// double nextDelta = 0;


class timeControl
{
private:
    double targetNoTurns = targetTime;
    int requiredSpeed = 0;

    int num1f = 0;
    int num2f = 0;
    int num3f = 0;

    int num1b = 0;
    int num2b = 0;
    int num3b = 0;

    unsigned long startTime;

    void pathParser()
    {
      pathParser(0);
    }
    void pathParser(int instruction)
    {
      for (int i = instruction; i < path.length(); i++)
      {
        switch (path[i])
        {
          case 'R':
            targetNoTurns -= 0.81;
            break;
          case 'L':
            targetNoTurns -= 0.83;
            break;
          case 'S':
            targetNoTurns -= 1.7075;
            break;
          case 'E':
            targetNoTurns -= 1.88625;
            break;
          case 'F':
            num1f++;
            break;
          case '2':
            num2f++;
            break;
          case '3':
            num3f++;
            break;
          case 'B':
            // num1b++;
            num1f++;
            break;
          case 'Y':
            num2f++;
            // num2b++;
            break;
          case 'Z':
            num3f++;
            // num3b++;
            break;
        }
      }
    }

    void bruteForceTime()
    {
      for (int i = 1; i <= 225; i++)
      {
        double thisSpeedTime =
                forwardTime(1, i, num1f) +
                forwardTime(2, i, num2f) +
                forwardTime(3, i, num3f) +
                backwardTime(1, i, num1b) +
                backwardTime(2, i, num2b) +
                backwardTime(3, i, num3b);

        if ((thisSpeedTime - targetNoTurns) > 0 && abs(thisSpeedTime - targetNoTurns) < abs(requiredSpeed - targetNoTurns))
        {
          requiredSpeed = i;
        }
      }

      if (requiredSpeed == 0)
      {
        requiredSpeed = 225;
        Serial.println("ran out of speed");
      }
    }

    double forwardTime(int distance, int speed, int amount)
    {
      if (distance == 1)
      {
        return (1908.65735* pow(speed, -1.77453) + 1.60341 ) * amount;
        // return ((88.497 * pow(speed, -0.971662)) + 1.21408) * amount;
      }
      else if (distance == 2)
      {
        return (6660.91837 * pow(speed, -1.85821) + 2.09068) * amount;
        // return ((685.32 * pow(speed, -1.24345)) + 1.41097) * amount;
      }
      else if (distance == 3)
      {
        return (5462.21835 * pow(speed, -1.65756) + 2.34208) * amount;
        // return ((1051.57 * pow(speed, -1.21325)) + 1.27255) * amount;
      }
      return 0;
    }

    double backwardTime(int distance, int speed, int amount)
    {
      double constDT = 0.2;

      if(distance == 1)
      {
        return ((88.497 * pow(speed, -0.971662)) + 1.21408) * amount + constDT; //single baclwards regression, time(distance)
      }
      else if(distance == 2)
      {
        return ((685.32 * pow(speed, -1.24345)) + 1.41097) * amount + constDT; //double backwards regression, time(distance)
      }
      else if(distance == 3)
      {
        return ((1051.57 * pow(speed, -1.21325)) + 1.27255) * amount + constDT; //triple backwards regression, time(distance)
      }
    }

public:
    timeControl()
    {
      pathParser();
      bruteForceTime();
      startTime = millis(); // Start timer
      Serial.println(targetNoTurns);
      Serial.print("AlgoSpeed: ");
      Serial.println(requiredSpeed);

      if (mySpeed == 0)
      {
        mySpeed = requiredSpeed;
      }
    }

    void begin() {
      startTime = millis();
    }

    void updateSpeedAfterMove(int rem1f, int rem2f, int rem3f, int rem1b, int rem2b, int rem3b, int right, int left)
    {
      double elapsedTime = (millis() - startTime) / 1000.0;
      Serial.println(elapsedTime);
      double newTarget = targetTime - elapsedTime - 0.765 * right - 0.85875 * left - 1.88625;

      if (newTarget <= 0) return;

      int low = 1;
      int high = 225;
      int bestSpeed = 225;
      double bestDiff = 1e9;

      while (low <= high)
      {
        int mid = (low + high) / 2;

        double t =
                forwardTime(1, mid, rem1f) +
                forwardTime(2, mid, rem2f) +
                forwardTime(3, mid, rem3f) +
                backwardTime(1, mid, rem1b) +
                backwardTime(2, mid, rem2b) +
                backwardTime(3, mid, rem3b);

        double diff = abs(t - newTarget);

        if (t >= newTarget)
        {
          if (diff < bestDiff)
          {
            bestDiff = diff;
            bestSpeed = mid;
          }
          high = mid - 1; // Try lower speeds
        }
        else
        {
          low = mid + 1; // Try higher speeds
        }
      }

      mySpeed = bestSpeed;
      Serial.print("Updated speed: ");
      Serial.println(mySpeed);
    }

};

timeControl timeSetter;

unsigned long previousMillis = 0;  // stores the last time the task was performed
const long interval = 200;         // interval (in milliseconds) for 0.5 second delay

// Function to simulate non-blocking delay
bool nonBlockingDelay(unsigned long interval) {
  unsigned long currentMillis = millis(); // Get current time
  
  // Check if enough time has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Reset the timer
    return true;  // Indicate that the delay interval has passed
  }
  return false;  // Return false if the interval hasn't passed yet
} 

// void calcXYError() {
//   double left = leftDist.dist();
//   double right = rightDist.dist();
//   if (left > 6 && left < 18) {       
//     // use left side
//     double dL = left;
    
//     nextDelta = max(nextDelta, 12.1 - dL); // if too far to left then is positive, if too far right then itll be -
//   }
//   else if (right > 6 && right < 18) {
//     double dR = right;
    
//     nextDelta = max(nextDelta, dR - 12.1); // if too far to left then is positive, if too far right then itll be -
//   }
//   else {
//     nextDelta = 0;
//   }
// }


class Robot {
private:
    int setSPEED = mySpeed;
    double targetAngle = 0;
    int currSPEED = setSPEED;
    const int CONSTANT_SPEED = 110;

    int INSTRUCTION = 0;

    double Setpoint = 0, Input = 0, Output = 0;
    double Input2 = 0, Output2 = 0;

    bool rStart = false;
    bool correctingPhase = false;

    double UNIT_CM_CALIBRATION = 38.9041;
    // 39.41756
    //17.1221

    PID motorPID = PID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);
    PID motorPID2 = PID(&Input2, &Output2, &Setpoint, 0, 0, 0, DIRECT);

    //PID tuning arrays

    //forwards
    const int pidK[1] = {4.5};
    const int pidI[1] = {2.5};

    const int pidK2[1] = {4.5};
    const int pidI2[1] = {2.5};

    //backwards
    const int BpidK[1] = {3};
    const int BpidI[1] = {2};

    const int BpidK2[1] = {3};
    const int BpidI2[1] = {2};

    boolean firstMove = true;

    //other variables
    int correctingSpeed = 55;
    int turningSpeed = 65;

    //Used to get current outputs from the PWM
    void calError() {
      Input = currAngle - targetAngle;
      Input2 = Input * -1;

      motorPID.Compute();
      motorPID2.Compute();

      if(Output > 30)
      {
        Output = 30;
      }

      if(Output2 > 30)
      {
        Output2 = 30;
      }
    }

    //weird code which sets the pwm to the right tuning
    void getPWMForwardTuning(int PWMMotorSpeed)
    {
      int pidTuningArraySize = sizeof(pidK)/sizeof(pidK[0]);
      int pidTuningIndex = round((((PWMMotorSpeed - 40)/215.0) * pidTuningArraySize)) - 1;

      if(pidTuningIndex >= pidTuningArraySize)
      {
        pidTuningIndex = pidTuningArraySize-1;
      }
      if(pidTuningIndex < 0)
      {
        pidTuningIndex = 0;
      }

      motorPID.SetTunings(pidK[pidTuningIndex], pidI[pidTuningIndex], 0);
      motorPID2.SetTunings(pidK2[pidTuningIndex], pidK2[pidTuningIndex], 0);
    }

    //same as other PWM function but uses backwards values
    void getBackPWMTuning(int PWMMotorSpeed)
    {
      int pidTuningArraySize = sizeof(pidK)/sizeof(pidK[0]);
      int pidTuningIndex = round((((PWMMotorSpeed - 40)/215.0) * pidTuningArraySize)) - 1;

      if(pidTuningIndex >= pidTuningArraySize)
      {
        pidTuningIndex = pidTuningArraySize-1;
      }
      if(pidTuningIndex < 0)
      {
        pidTuningIndex = 0;
      }

      motorPID.SetTunings(BpidK[pidTuningIndex], BpidI[pidTuningIndex], 0);
      motorPID2.SetTunings(BpidK2[pidTuningIndex], BpidK2[pidTuningIndex], 0);
    }

    bool turnCorrection() {
      if(currAngle - targetAngle < 0.2 && currAngle - targetAngle > 0)
      {
        motor1.brake();
        motor2.brake();
        motor1.drive(correctingSpeed*0.965);
        motor2.drive(correctingSpeed);
        return true;
      }
      else if(currAngle - targetAngle > -0.2 && currAngle - targetAngle < 0)
      {
        motor1.brake();
        motor2.brake();
        motor1.drive(correctingSpeed*0.965);
        motor2.drive(correctingSpeed);
        return true;
      }

      if(currAngle > targetAngle)
      {
        motor1.drive(correctingSpeed*0.965);
        motor2.drive(correctingSpeed * -1);
        return false;
      }

      if(currAngle < targetAngle)
      {
        motor1.drive((correctingSpeed*0.965) * -1);
        motor2.drive(correctingSpeed*1.01);
        return false;
      }
    }

    bool firstEndCorrection = true;
    double angelOffset = 0;
    double afterMovement = 0;
    bool doneTurnCorrecting = false;

    bool endCorrection()
    {
      if(xOffset == 0 && yOffset == 0)
      {
        return true;
      }

      if(firstEndCorrection)
      {
        yOffset = yOffset * -1;
        angelOffset = atan((abs(xOffset)/(yOffset + 9.748)) * (180/3.14159265));

        if(xOffset < 0)
        {
          angelOffset = angelOffset * -1;
        }

        afterMovement = sqrt(pow((yOffset+9.748),2) + pow(xOffset,2)) - 9.748;

        targetAngle = targetAngle + angelOffset;
        firstEndCorrection = false;

        knobLeft.write(0);
        knobRight.write(0);
      }

      if(doneTurnCorrecting)
      {
        if(afterMovement > 0)
        {
          if(!validateF(UNIT_CM_CALIBRATION * afterMovement))
          {
            const int speed = 80;
            getPWMForwardTuning(speed);
            calError();

            motor1.drive(speed);
            motor2.drive(speed);

            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          if(!validateB(UNIT_CM_CALIBRATION * afterMovement))
          {
            const int speed = 80;
            getBackPWMTuning(speed);
            calError();

            motor1.drive(-1*(speed));
            motor2.drive(-1*(speed));

            return false;
          }
          else
          {
            return true;
          }
        }
      }
      else
      {
        if(turnCorrection())
        {
          doneTurnCorrecting = true;
        }
        else
        {
          return false;
        }
      }
    }

    bool forwardDoneTurnCorrecing = false;

    void forward(double length, bool ending, int SPEED) {

      double cal = (length)*UNIT_CM_CALIBRATION;
      //cal = 650;

      calError();

      if(firstMove)
      {
        currSPEED = 0;
        firstMove = false;

        motorPID.SetMode(AUTOMATIC);
        motorPID2.SetMode(AUTOMATIC);
      }


      if (ending) {
        SPEED = CONSTANT_SPEED;
      }

      //sets it initially
      getPWMForwardTuning(SPEED);
      //motorPID.SetTunings(6,0, 0);
      //motorPID2.SetTunings(6,0, 0);

      if (!validateF(cal)) {
        if (knobLeft.read()>cal-900 || knobRight.read()>cal-900) {

          //1000 seems to work well, determins when to start decel
          currSPEED -= round(currSPEED/8); //change the number divied by to change rate
          if(currSPEED < 60)
          {
            currSPEED = 60;
          }

          //updates pwm as slowing down :)
          // getPWMForwardTuning(currSPEED);
          calError();

          motor1.drive(currSPEED+Output2);
          motor2.drive(currSPEED+Output);
        }
        else if(currSPEED < SPEED)
        {
          const int interval = 1.5; //used to control how fast accel happens
          if(SPEED - currSPEED > interval)
          {
            currSPEED += interval;
            // getPWMForwardTuning(currSPEED);
            calError();

            motor1.drive(currSPEED + Output2);
            motor2.drive(currSPEED + Output);
          }
          else
          {
            getPWMForwardTuning(SPEED);
            calError();
            currSPEED = SPEED;
          }
        }
        else {
          calError();
          motor1.drive(SPEED+Output2);
          motor2.drive(SPEED+Output);
        }
      }
      else {
        correctingPhase = true;
        motor1.brake();
        motor2.brake();
      }

      if(correctingPhase)
      {
        if (turnCorrection())
        {
          forwardDoneTurnCorrecing = true;
        }

        if(forwardDoneTurnCorrecing)
        {
          if(!ending)
          {
            motor1.brake();
            motor2.brake();
            INSTRUCTION++;
            knobLeft.write(0);
            knobRight.write(0);
            firstMove = true;
            correctingPhase = false;
            forwardDoneTurnCorrecing = false;
            // recalculateSpeed();
          }
          else
          {
            if(endCorrection())
            {
              motor1.brake();
              motor2.brake();
              INSTRUCTION++;
              knobLeft.write(0);
              knobRight.write(0);
              firstMove = true;
              correctingPhase = false;
              firstEndCorrection = true;
              forwardDoneTurnCorrecing = false;
              // recalculateSpeed();
            }
          }
        }
      }
    }

    void reverse(int length, int SPEED) {

      double cal = -length * UNIT_CM_CALIBRATION;

      if(firstMove)
      {
        currSPEED = 0;
        firstMove = false;

        motorPID.SetMode(AUTOMATIC);
        motorPID2.SetMode(AUTOMATIC);
      }

      calError();
      getBackPWMTuning(SPEED);

      if (!validateB(cal)) {

        if (knobLeft.read()<cal+450 || knobRight.read()<cal+450) {
          //1000 seems to work well, determins when to start decel
          currSPEED -= round(currSPEED/41); //change the number divied by to change rate
          if(currSPEED < 40)
          {
            currSPEED = 40;
          }

          //updates pwm as slowing down :)
          getPWMForwardTuning(currSPEED);
          calError();

          motor1.drive(-1*(currSPEED+Output));
          motor2.drive(-1*(currSPEED+Output2));
        }
        else if(currSPEED < SPEED)
        {
          const int interval = 6; //used to control how fast accel happens
          if(SPEED - currSPEED > interval)
          {
            currSPEED += interval;
            getPWMForwardTuning(currSPEED);
            calError();

            motor1.drive(-1*(currSPEED+Output));
            motor2.drive(-1*(currSPEED+Output2));
          }
          else
          {
            getPWMForwardTuning(SPEED);
            calError();
            currSPEED = SPEED;
          }
        }
        else {
          motor1.drive(-1*(SPEED+Output));
          motor2.drive(-1*(SPEED+Output2));
        }
      }
      else {
        correctingPhase = true;
      }

      if(correctingPhase)
      {
        if (turnCorrection()) {
          motor1.brake();
          motor2.brake();
          INSTRUCTION++;
          knobLeft.write(0);
          knobRight.write(0);
          firstMove = true;
          correctingPhase = false;
          motor1.brake();
          motor2.brake();
          // recalculateSpeed();
        }
      }
    }

    void right() {
      // const int turnCountRightM1 = 188;
      // const int turnCountRightM2 = -188;

      const int turnCountRightM1 = 188;
      const int turnCountRightM2 = -188;

      if (!rStart) {
        targetAngle = targetAngle - 90;
        rStart = true;

        motor1.drive((int)(turningSpeed));
        motor2.drive((turningSpeed*2) * -1);
      }
      else {
        if(!correctingPhase)
        {
          //Serial.println("doing shit");
          //Serial.println(knobLeft.read());
          //Serial.println(knobRight.read());

          if(knobRight.read() >= turnCountRightM1)
          {
            motor1.brake();
          }
          else
          {
            motor1.drive(turningSpeed);
          }

          if(knobLeft.read() <= turnCountRightM2)
          {
            motor2.brake();
          }
          else
          {
            motor2.drive((turningSpeed) * -1);
          }

          if((knobRight.read() >= turnCountRightM1) && (knobLeft.read() <= turnCountRightM2))
          {
            correctingPhase = true;
            motor1.brake();
            motor2.brake();
          }
        }
        else if (turnCorrection()) {
          motor1.brake();
          motor2.brake();
          INSTRUCTION++;
          knobLeft.write(0);
          knobRight.write(0);
          rStart = false;
          correctingPhase = false;
          // recalculateSpeed();
        }
      }
    }
    void left() {
      // const int turnCountRightM1 = -188;
      // const int turnCountRightM2 = 188;

      const int turnCountRightM1 = -180;
      const int turnCountRightM2 = 180;

      if (!rStart) {
        targetAngle = targetAngle + 90;
        rStart = true;

        motor1.drive((turningSpeed) * -1);
        motor2.drive((int)turningSpeed*.9);
      }
      else {
        if(!correctingPhase)
        {
          //Serial.println("doing shit");
          //Serial.println(knobLeft.read());
          //Serial.println(knobRight.read());

          if(knobRight.read() <= turnCountRightM1)
          {
            motor1.brake();
          }
          else
          {
            motor1.drive((turningSpeed) * -1);
          }

          if(knobLeft.read() >= turnCountRightM2)
          {
            motor2.brake();
          }
          else
          {
            motor2.drive(turningSpeed);
          }

          if((knobRight.read() <= turnCountRightM1) && (knobLeft.read() >= turnCountRightM2))
          {
            correctingPhase = true;
            motor1.brake();
            motor2.brake();
          }
        }
        else if (turnCorrection()) {
          INSTRUCTION++;
          motor1.brake();
          motor2.brake();
          knobLeft.write(0);
          knobRight.write(0);
          rStart = false;
          correctingPhase = false;
          // recalculateSpeed();
        }
      }
    }

    bool validateF(double calibration) {
      return (knobLeft.read() >= calibration || knobRight.read() >= calibration);
    }

    bool validateB(double calibration) {
      return (knobLeft.read() <= calibration || knobRight.read() <= calibration);
    }

    void recalculateSpeed() {
      int rem1f = 0, rem2f = 0, rem3f = 0;
      int rem1b = 0, rem2b = 0, rem3b = 0;
      int right = 0, left = 0;

      for (int i = INSTRUCTION; i < path.length(); i++) {
        switch (path[i]) {
          case 'F': rem1f++; break;
          case '2': rem2f++; break;
          case '3': rem3f++; break;
          case 'B': rem1b++; break;
          case 'Y': rem2b++; break;
          case 'Z': rem3b++; break;
          case 'R': right++; break;
          case 'L': left++;  break;
        }
      }

      timeSetter.updateSpeedAfterMove(rem1f, rem2f, rem3f, rem1b, rem2b, rem3b, right, left);
    }

public:
    int singleForwardSpeed;
    int doubleForwardSpeed;
    int tripleForwardSpeed;

    int singleBackwardsSpeed;
    int doubleBackwardsSpeed;
    int tripleBackwardsSpeed;

    Robot() //default constructor
    {
      singleForwardSpeed = setSPEED;
      doubleForwardSpeed = setSPEED;
      tripleForwardSpeed = setSPEED;

      singleBackwardsSpeed = setSPEED;
      doubleBackwardsSpeed = setSPEED;
      tripleBackwardsSpeed = setSPEED;
    }

    Robot(int SFS, int DFS, int TFS, int SBS, int DBS, int TBS)
    {
      singleForwardSpeed = SFS;
      doubleForwardSpeed = DFS;
      tripleForwardSpeed = TFS;

      singleBackwardsSpeed = SBS;
      doubleBackwardsSpeed = DBS;
      tripleBackwardsSpeed = TBS;
    }

    void run() {
      
      if (INSTRUCTION > path.length()-1) {
        delay(50000);
      }
      else if (path[INSTRUCTION] == 'S') {
        forward(34.748, false, CONSTANT_SPEED);
      }
      else if (path[INSTRUCTION] == 'E') {
        forward(35.252, true, CONSTANT_SPEED); //This should be lower
      }
      else if (path[INSTRUCTION] == 'F') {
        forward(50, false, singleForwardSpeed);
      }
      else if (path[INSTRUCTION] == '2') {
        forward(100, false, doubleForwardSpeed);
      }
      else if (path[INSTRUCTION] == '3') {
        forward(150, false, tripleForwardSpeed);
      }
      else if (path[INSTRUCTION] == 'C') {
        forward(20000, false, 225);
      }
      else if (path[INSTRUCTION] == 'R') {
        right();
      }
      else if (path[INSTRUCTION] == 'L') {
        left();
      }
      else if (path[INSTRUCTION] == 'B') {
        reverse(50, singleBackwardsSpeed);
      }
      else if (path[INSTRUCTION] == 'Y') {
        reverse(100, doubleBackwardsSpeed);
      }
      else if (path[INSTRUCTION] == 'Z') {
        reverse(150, tripleBackwardsSpeed);
      }
    }
}
;


Robot robotRunner;
boolean started = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("setup function");

  pinMode(button, INPUT_PULLUP);

  //calculateNeededSpeed();

  angle.mpuSetUp();
}

void loop()
{
  //debug();
  // angle.angleUpdate();


  // Serial.println(knobLeft.read());
  // Serial.println("left:");
  // Serial.println(leftDist.dist());
  // Serial.println("right:");
  // Serial.println(rightDist.dist());

  if(started)
  {
    //put robot moving code in this block!!!
    //motor1.drive(40);
    // if (nonBlockingDelay(interval)) {
    // // Call the function to run every 0.5 seconds
    //   calcXYError();
    // }


    robotRunner.run();
     Serial.println(knobLeft.read());
          // Serial.println(knobRight.read());
    
  }
  else
  {
    if(digitalRead(button) == LOW)
    {
      Serial.println("button");
      currAngle = 0;
      started = true;

      knobRight.write(0);
      knobLeft.write(0);

      delay(200);
      angle.calculateError();
      currAngle = 0;
      delay(200);
      timeSetter.begin();
    }

  }
}
