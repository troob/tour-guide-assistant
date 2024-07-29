/* TourGuideAssistant0.ino
 * Follow user if instructions given.
 * Search for target if not at target destination.
 */

//======Advisor======
//===arbitration struct===
typedef struct layer LAYER; // C struct for subsumption task output

struct layer
{
  int cmd, // assertion command
    arg, // assertion argument
    flag; // subsumption flag (instead of layer state?)
};

// the robot has x subsumption behaviors, so define 2 LAYERS:
LAYER follow,
  search,
  rotate,
  moveStraight,
  halt; // the default layer

const int job1Size = 5;

LAYER *job1[job1Size] = { &follow, &search, &rotate, &moveStraight, &halt };

LAYER *thisLayer = &halt; // output, layer chosen by arbitrator; global pointer to highest priority task asserting its flag, which is maintained by the "arbitration winner" signal

LAYER **job; // pointer to job priority list

//typedef struct target TARGET;
//
//struct target
//{
//  int x, y, heading
//};
//
//TARGET setTarget;

int jobSize, // number of tasks in priority list
  arbitrate; // global flag to enable subsumption

volatile int rotateEnable,
  followEnable,
  searchEnable,
  setPointActive,
  haltBot; // global flag to halt robot

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPins[] = 
{
  3, // encoder signal 1 pin
  7 // encoder signal 2 pin
};

const byte numEncoders = 2,
  pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

// values change in callback methods:
volatile int velPulseCounts[numEncoders],
  rotVels[numEncoders],
  prevRotVels[numEncoders],
  setVels[numEncoders],
  pulseCounts[numEncoders];

//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 5, 6 };

//======Mobile Platform (DFRobot Turtle)======
int wheelDiam = 64, // [mm]
  botDiam = 138; // [mm] (i.e. wheel base or distance between wheel centers)
//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
const int numMtrs = 2;

int reverse, // note: could be bool
  sensorsTmrCtr,
  maxOutVal,
  pubVelRate = 10,
  minSpeedAtTarget, // [pulses/(1/pubVelRate)s]
  topVel, // [mm/s]
  topPulseRate, // [pulses/(1/pubVelRate)s]
  navDeadZone, // [deg]
  dppr,
  tppr,
  setRadius, // [cm]
  turnLeft, // [pulses/(1/pubVelRate)s]
  turnRight; // [pulses/(1/pubVelRate)s]

double minLinearRes,
  downRamp, // [cm]
  dppcm = 0.0,
  tppcm = 0.0;

volatile float lCms, rCms, cms;

volatile double kp, ki, kd,
  //======Robot Pose======
  measX, // x position in cm
  measY, // y position in cm
  measHeading, // heading
  relHeading,
  totCm, // total cm traveled
  
  dSamplePosition,
  tSamplePosition,

  dLastPosition,
  tLastPosition,

  dPulses,
  tPulses,

  dCm, 
  tCm,

  setDistance, // value returned from virtual sensor

  dx, dy; //

volatile int setTanVel, // 0-100%
  setRotVel, // 0-100%
  setX,
  setY,
  setHeading,
  botVel, // global, current requested robot velocity
  headingError, // value returned from virtual sensor
  lSamp, rSamp, lPulses, rPulses, lPrev, rPrev;

volatile int pubMtrCmds[numMtrs],
  signs[numMtrs],
  mtrOutAccums[numMtrs];
  
void setup() 
{
  initSystem();

  initBehaviors();

  initSensorsTimer(); 
}

int initSystem()
{
  initNode("TourGuideAssistant0");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channel 1 */
  pinMode(mEnablePins[0], OUTPUT);
  pinMode(mSigPins[0], OUTPUT);

  /* Start Motor Channel 2 */
  pinMode(mEnablePins[1], OUTPUT);
  pinMode(mSigPins[1], OUTPUT);

  // ADD: publish rotational (and later translational) rotVel
}

void initBehaviors()
{
  initVars();

  setParams();
  
  // set job1 as active job at startup time
  initJob1();
}

void initVars()
{
  velPulseCounts[0] = 0;
  velPulseCounts[1] = 0;

  pulseCounts[0] = 0; // left distance
  pulseCounts[1] = 0; // right distance

  measX = 0; // [cm]
  measY = 0; // [cm]
  measHeading = piApprox / 2.0; // [rad]
  relHeading = 0;
  
  setTanVel = 0; // [mm/s]
  setRotVel = 0; // [deg/s]

  setX = 0; // [mm] // CHANGE to accept [ft]
  setY = 0; // [mm]
  setHeading = 0; // [deg]

  pubMtrCmds[0] = 0;
  pubMtrCmds[1] = 0;

  signs[0] = 1;
  signs[1] = 1;

  mtrOutAccums[0] = 0;
  mtrOutAccums[1] = 0;

  rotVels[0] = 0;
  rotVels[1] = 0;

  prevRotVels[0] = 0;
  prevRotVels[1] = 0;
}

void setParams()
{
  setPIDGains(1.0, 0.0, 0.0);

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  computeMinAngularRes();

  computeMinLinearRes();
  
  followEnable = 0;

  searchEnable = 0;

  setPointActive = 0; // no targets stored in memory yet

  reverse = 0;

  arbitrate = 1;

  haltBot = 1;

  downRamp = 10; // [cm], CHANGE: tune value properly

  minSpeedAtTarget = 0; // [pulses/(1/pubVelRate)s], CHANGE: tune value properly

  topVel = 1500; // [mm/s]

  topPulseRate = 2; //(int) round( convertMMToPulses(topVel) / (double) pubVelRate );
  Serial.print("topPulseRate: ");
  Serial.println(topPulseRate);

  navDeadZone = 10; // [deg], recommended b/t 5&10

  setRadius = 3; // [cm], CHANGE: tune value properly

  turnLeft = -1; // [pulses/(1/pubVelRate)s], CHANGE: tune value properly

  turnRight = 1; // [pulses/(1/pubVelRate)s], CHANGE: tune value properly

  Serial.println();
}

void setPIDGains(double pg, double ig, double dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;
  
  kp = pg;

  ki = ig;

  kd = dg;

  Serial.print("Controller got kp:");
  Serial.print(kp, 3);
  Serial.print(", ki:");
  Serial.print(ki, 3);
  Serial.print(", kd:");
  Serial.println(kd, 3);
}

void computeMinAngularRes()
{
  minAngularRes = 360.0 / pulsesPerRev;

  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
}

void computeMinLinearRes()
{
  float wheelCircumf;
  
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  wheelCircumf = piApprox * wheelDiam; // [mm]
  //Serial.print("wheelCircumf = ");
  //Serial.print(wheelCircumf);
  //Serial.println(" mm");
  
  minLinearRes = wheelCircumf / pulsesPerRev; 
  Serial.print("Min. Lin. Res. (mm): ");
  Serial.println(minLinearRes);
}

int initJob1() // make job1 the active job
{
  job = &job1[0]; // global job priority list pointer

  jobSize = job1Size; // no. tasks in job1 list

  return 0;
}

void initSensorsTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = sensorsTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void loop() {}

//======Interrupt Service Routines======
void encoder1Callback()
{
  velPulseCounts[0]++;

  determinePulseCounts(1);

  Serial.print("Vel. Pulse Count 1: ");
  Serial.print(velPulseCounts[0]);
  Serial.println("\n");
}

void encoder2Callback()
{
  velPulseCounts[1]++;

  determinePulseCounts(2);

  Serial.print("Vel. Pulse Count 2: ");
  Serial.print(velPulseCounts[1]);
  Serial.println("\n");
}

void determinePulseCounts(int id)
{
  if(signs[id - 1] == 1)
    pulseCounts[id - 1]++;
  else
    pulseCounts[id - 1]--;

  Serial.print("Pulse Count ");
  Serial.print(id);
  Serial.print(": ");
  Serial.print(pulseCounts[id-1]);
  Serial.println("\n");
}

/* Run Sensor Loop at x (maybe 10-20) Hz, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 */
ISR(TIMER1_OVF_vect) // sensors loop!
{
  TCNT1 = sensorsTmrCtr; // set timer

  readUserInput(); // read: get requested tan and rot velocities; get requested coordinates. in this case, received by uio, which is a separate task(?)

  searchTask(); // default task, for now
  
  followTask(); // accelerate forward and maintain given speed

  rotateTask(); // pivot to given heading, where bot starts at heading 90 deg, parallel to vertical (y) axis
  
  arbitrator(); // send highest priority to motors
}

void readUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "g")
    {
      Serial.println("go");

      searchEnable = 1;
      
      setPointActive = 1;
    
      followEnable = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      followEnable = 0;

      if(setPointActive)
        haltBot = 1;
    }
    else if(inputString.substring(0,1) == "t") // given in m/s, convert to mm/s, so operations can be done with integer
    { 
      setTanVel = (int) round( inputString.substring(1, inputString.length()).toFloat() * 1000 ); // get string after 't'
    
      Serial.print("v_t = ");
      Serial.print(setTanVel / 1000.0);
      Serial.println(" m/s\n");

      followEnable = 1;
    }
    else if(inputString.substring(0,1) == "r") // given in deg/s
    {
      setRotVel = inputString.substring(1, inputString.length()).toInt(); // get string after 'r'
      
      Serial.print("v_r = ");
      Serial.print(setRotVel);
      Serial.println(" deg/s\n");

      followEnable = 1;
    }
    else if(inputString.substring(0,1) == "x") // given in m
    {
      setX = (int) round( inputString.substring(1, inputString.length()).toFloat() * 100 ); // get string after 'x'
    
      Serial.print("x = ");
      Serial.print(setX);
      Serial.println(" cm\n");
    }
    else if(inputString.substring(0,1) == "y") // given in m
    {
      setY = (int) round( inputString.substring(1, inputString.length()).toFloat() * 100 ); // get string after 'y'; inputString.substring(1, inputString.length()).toInt();
    
      Serial.print("y = ");
      Serial.print(setY);
      Serial.println(" cm\n");
    }
    else if(inputString.substring(0,1) == "h") // given in deg
    {
      setHeading = inputString.substring(1, inputString.length()).toInt(); // get string after 'h'
    
      Serial.print("heading = ");
      Serial.print(setHeading);
      Serial.println(" deg\n");

      rotateEnable = 1;

      followEnable = 0;

      searchEnable = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "kp")
      kp = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kp'
    else if(inputString.substring(0,2) == "ki")
      ki = inputString.substring(2, inputString.length()).toFloat(); // get string after 'ki'
    else if(inputString.substring(0,2) == "kd")
      kd = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kd'

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

int searchTask() // call this from x Hz (~20 Hz) sensor loop
{
  extern LAYER search; // C structure for output
  
  extern int setPointActive; // global, reset when target acquired

  readOdometer(); // get current position, so robot knows where to return even if interrupted by user follow velocities
  
  //if(searchEnable) search.flag = true;
  //else search.flag = false;

  if(setPointActive == 0) // do halt behavior
  {
    search.cmd = 0; // yes, stop

    search.arg = 0; // stop turning
  }
  else // do navigate behavior
  {
    locateTarget(); // read the "sensor"

    if(setDistance < setRadius) // arrived at set point?
    {
      //search.cmd = 0; // yes, stop

      //search.arg = 0; // stop turning

      haltBot = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned

      setPointActive = 0; // signal that target is acquired
    }
    else // still looking for target
    {
      if(setDistance < downRamp) // slow down when getting close
      {
        search.cmd = (int) round( ( setDistance * topPulseRate ) / downRamp );

        if(search.cmd < minSpeedAtTarget)
          search.cmd = minSpeedAtTarget;
      }
      else
        search.cmd = topPulseRate; // distance > 10 cm, request top speed
      
      if(abs(headingError) > navDeadZone) // heading error large? (recommended navDeadzone: 5 or 10 deg)
      {
        if(headingError > 0) // yes, steer toward target
        {
          if(setDistance < downRamp)
            search.arg = (int) round( ( setDistance * turnLeft ) / downRamp );
          else
            search.arg = turnLeft;
        }
        else
        {
          if(setDistance < downRamp)
            search.arg = (int) round( ( setDistance * turnRight ) / downRamp );
          else
            search.arg = turnRight;
        }
      }
      else 
        search.arg = 0; // else target is in dead-zone, straight ahead

      if(search.flag == false)
      {
        if(search.cmd + search.arg > 0)
          mtrOutAccums[0] = 6000;
        else if(search.cmd + search.arg < 0)
          mtrOutAccums[0] = -6000;
  
        if(search.cmd - search.arg > 0)
          mtrOutAccums[1] = 6000;
        else if(search.cmd - search.arg < 0)
          mtrOutAccums[1] = -6000;
      }
    }
  }

  search.flag = true;
}

/* Like common cruise behavior,
 * but allows user to request tan and rot vels,
 * rather than assuming top speed or 0.
 */
int followTask()
{
  extern LAYER follow; // C structure for task output

  if(followEnable)
  {
    if(reverse) 
    {
      follow.cmd = (int) round( -convertMMToPulses(setTanVel) / (double) pubVelRate ); // request -drive
      follow.arg = (int) round( -convertDegToPulses(setRotVel) / (double) pubVelRate ); // request -turn
    }
    else
    {
      follow.cmd = (int) round( convertMMToPulses(setTanVel) / (double) pubVelRate ); // request drive
      follow.arg = (int) round( convertDegToPulses(setRotVel) / (double) pubVelRate ); // request turn
    }
  
    Serial.print("cmd = ");
    Serial.print(follow.cmd); // maybe change to user->cmd
    Serial.print(" pulses/");
    Serial.print(1.0 / pubVelRate);
    Serial.println("s");
  
    Serial.print("arg = ");
    Serial.print(follow.arg); // maybe change to user->cmd
    Serial.print(" pulses/");
    Serial.print(1.0 / pubVelRate);
    Serial.println("s");

    follow.flag = true; // signal arbitrator we want control, 
  }
  else // disabled
    follow.flag = false;
}

void readOdometer()
{
  // sample left and right encoder counts as close together in time as possible
  lSamp = pulseCounts[0]; // s_j^{(l)}
  rSamp = pulseCounts[1]; // s_j^{(r)}
  Serial.print("lSamp (p): ");
  Serial.println(lSamp);
  Serial.print("rSamp (p): ");
  Serial.println(rSamp);

  // determine no. pulses since previous sampling
  lPulses = lSamp - lPrev; // p_j^{(l)}
  rPulses = rSamp - rPrev; // p_j^{(r)}
  Serial.print("lPulses (p): ");
  Serial.println(lPulses);
  Serial.print("rPulses (p): ");
  Serial.println(rPulses);

  // and update previous sampling for next iteration
  lPrev = lSamp; // p_{j-1}^{(l)}
  rPrev = rSamp; // p_{j-1}^{(r)}
  Serial.print("lPrev (p): ");
  Serial.println(lPrev);
  Serial.print("rPrev (p): ");
  Serial.println(rPrev);

  // convert longs to floats and pulses to cms
  lCms = (float) lPulses / ( 10 / minLinearRes ); // d_j^{(l)}
  rCms = (float) rPulses / ( 10 / minLinearRes ); // d_j^{(r)}
  Serial.print("lCms (cm): ");
  Serial.println(lCms);
  Serial.print("rCms (cm): ");
  Serial.println(rCms);

  // calculate distance traveled since last sampling
  cms = ( lCms + rCms ) / 2.0; // d_j
  Serial.print("cms (cm): ");
  Serial.println(cms);

  // accumulate total rotation around our center
  relHeading = asin( ( rCms - lCms ) / ( botDiam / 10.0 ) );
  Serial.print("relHeading (rad): ");
  Serial.println(relHeading);
  measHeading += relHeading; //( lCms - rCms ) / ( wheelBase / 10.0 ); // [rad]
  Serial.print("measHeading (rad): ");
  Serial.println(measHeading);
  measHeading *= degsPerRad;
  Serial.print("measHeading (deg): ");
  Serial.println(measHeading);
  
  // and clip the rotation to +/-360 deg
  //measHeading += 360.0; // make sure measHeading > 360 before getting remainder of division by 360
  measHeading = (float) ( (int) ( (int) round(measHeading) % 360 ) ); // note: consider changing -= to =
  Serial.print("measHeading clipped (deg): ");
  Serial.println(measHeading);
  
  measHeading /= degsPerRad;
  Serial.print("measHeading clipped (rad): ");
  Serial.println(measHeading);
  // now calculate and accumulate our position in cms
  measY += cms * cos(relHeading); // y_j
  measX -= cms * sin(relHeading); // x_j
  
  displayPosition();
}

void displayPosition()
{
//  Serial.print("dSamplePosition (p): ");
//  Serial.println(dSamplePosition);
//  Serial.print("tSamplePosition (p): ");
//  Serial.println(tSamplePosition);
//  Serial.println();
//  Serial.print("dPulses (p): ");
//  Serial.println(dPulses);
//  Serial.print("tPulses (p): ");
//  Serial.println(tPulses);
//  Serial.println();
//  Serial.print("dLastPosition (p): ");
//  Serial.println(dLastPosition);
//  Serial.print("tLastPosition (p): ");
//  Serial.println(tLastPosition);
//  Serial.println();
//  Serial.print("dM (m): ");
//  Serial.println(dCm / 100.0);
//  Serial.print("tM (m): ");
//  Serial.println(tCm / 100.0);
//  Serial.println();
//  Serial.print("totM (m): ");
//  Serial.println(totCm / 100.0);
//  Serial.println();
  Serial.println("===Current Pose===");
  Serial.print("measX (cm): ");
  Serial.println(measX);
  Serial.print("measY (cm): ");
  Serial.println(measY);
  Serial.print("measHeading (rad): ");
  Serial.println(measHeading);
  Serial.println();
}

//void readOdometer()
//{
//  // sample the drive and turn positions, measured in pulses, as close together in time as possible
//  dSamplePosition = getDrivePosition();
//  tSamplePosition = getTurnPosition();
//
//  // determine how many pulses since our last sampling
//  dPulses = dSamplePosition - dLastPosition;
//  tPulses = tSamplePosition - tLastPosition;
//  
//  // and update the last sampling for next time
//  dLastPosition = dSamplePosition;
//  tLastPosition = tSamplePosition;
//
//  // convert pulses to cm
//  dCm = dPulses / dppcm; // total distance we have traveled since last sampling
//  tCm = tPulses / tppcm;
//
//  // accumulate total cm traveled
//  totCm += dCm;
//
//  // accumulate total rotation around our center
//  measHeading = tCm / (piApprox * ( wheelBase / 10.0 ) ); // ratio of turning circle
//
//  // and clip the rotation to +/-360 deg
//  measHeading *= 360.0;
//  measHeading = (double) ((int) measHeading % 360); // [deg]
//
//  // now calculate and accumulate our position in cm
//  measY += dCm * cos(measHeading / degsPerRad);
//  measX += dCm * sin(measHeading / degsPerRad);
//
//  displayPosition();
//}

//double getDrivePosition()
//{
//  return ( pulseCounts[0] + pulseCounts[1] ) / 2.0;
//}
//
//double getTurnPosition()
//{
//  return (double) pulseCounts[0] - pulseCounts[1];
//}

void locateTarget()
{
  dx = (float) setX - measX; // [cm], d_x
  dy = (float) setY - measY; // [cm], d_y

  setDistance = sqrt( ( dx * dx ) + ( dy * dy ) ); // [cm]

  /* no divide-by-zero allowed! */
  if (dx > 0.00001)
    setHeading = atan2(dy, dx) * degsPerRad;//90 - atan(dy/dx) * degsPerRad; //
    
  else if (dx < -0.00001)
    setHeading = -atan2(dy, dx) * degsPerRad;//90 - atan(dy/dx) * degsPerRad; //
    
  else
    setHeading = 90.0;

  headingError = setHeading - measHeading * degsPerRad; // [deg]
  //headingError += 360;
  headingError %= 360;
  
  if (headingError > 180.0)
    headingError -= 360.0;
  else if (headingError < -180.0)
    headingError += 360.0;

  displayTarget(dx, dy);
}

void displayTarget(double x, double y)
{
  Serial.println("===Set Pose===");
  Serial.print("setX (cm): ");
  Serial.println(setX);
  Serial.print("setY (cm): ");
  Serial.println(setY);
  Serial.print("setHeading (deg): ");
  Serial.println(setHeading);
  Serial.println("===Travel===");
  Serial.print("x (cm): ");
  Serial.println(x);
  Serial.print("y (cm): ");
  Serial.println(y);
  Serial.print("setDistance (cm): ");
  Serial.println(setDistance);
  Serial.print("headingError (deg): ");
  Serial.println(headingError);
  Serial.println();
}

/* Rotate to and stop at given heading.
 */
void rotateTask()
{
  extern LAYER rotate;

  readAngle();
  
  if(rotateEnable)
  {
    if(abs(dx) < 0.00001 && abs(dy) < 0.00001)
      rotate.flag = 1;
  }
}

void readAngle()
{
  
}

int convertMMToPulses(int mm) 
{
  return (int) round( mm / minLinearRes );
}

int convertDegToPulses(int deg) 
{
  return (int) round( deg / minAngularRes );
}

/* "winnerId" feedback line 
 * from Arbitrate back to tasks:
 * Essentially global var containing ID of task 
 * that "won" this round of arbitration.
 * It can be used by the individual tasks
 * to determine if they have been subsumed.
 */
void arbitrator()
{
  int i = 0;

  if(arbitrate)
  {
    for(i=0; i < jobSize - 1; i++) // step through tasks
    {
      if(job[i]->flag) break; // subsume
    }

    thisLayer = job[i]; // global output winner
  }

  mtrCmd(thisLayer); // send command to motors; execute, given pointer to winning layer
}

void mtrCmd(LAYER *l)
{
  botVel = l->cmd; // [pulses/(1/pubVelRate)s], current requested velocity

  // ADD: convert mm to pulses and deg to pulses before computing setVels!
  // Compute control signals:
  setVels[0] = botVel + l->arg; // // left motor = velocity + rotation, (int) round( 1000 * setRotVel / ( minAngularRes * pubVelRate ) ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  setVels[0] = clip(setVels[0], 100, -100); // don't overflow +/- 100% full speed
  
  setVels[1] = botVel - l->arg; // right motor = velocity - rotation
  setVels[1] = clip(setVels[1], 100, -100); // don't overflow +/- 100% full speed

  Serial.print("Set Vels (pulses/(1/pubVelRate)s): ");
  for(int i=0; i < numMtrs; i++)
  {
    Serial.print(setVels[i]);
    Serial.print(" ");
  }
  Serial.println("\n");

  if(haltBot)
    stopMoving();
  else
    controlVel(); // PID
}

void stopMoving()
{
  for(int i=0; i < numMtrs; i++)
    digitalWrite(mEnablePins[i], LOW);
}

void controlVel()
{
  int aOutputs[numMtrs];

  // Read analog input (i.e. calc rot vel):
  speedometer();
  
  // Compute control signals:
  computeControlSignals(); 

  // Set analog outputs:
  for(int i=0; i < 2; i++)
    aOutputs[i] = (int) round( mtrOutAccums[i] / 256.0 );
  
  modulatePulseWidths(aOutputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
}

/* Read and zero velPulseCounts.
 * Copy and accumulate counts from velPulseCounts
 * to the rot. vel. variable and
 * then reset velPulseCounts to zero.
 */
void speedometer()
{
  for(int i=0; i < numMtrs; i++)
  {
    rotVels[i] = velPulseCounts[i] * signs[i]; // copy and accumulate counts from velPulseCount to rotVel
    velPulseCounts[i] = 0; // reset velPulseCount to zero
  }
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void computeControlSignals()
{
  int errs[numMtrs],
    P[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight
  
  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);

  for(int i=0; i < numMtrs; i++)
  {
    Serial.print("setVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(setVels[i]);
    Serial.print(", rotVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(rotVels[i]);
  
    errs[i] = (int) round( ( b * setVels[i] - rotVels[i] ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    Serial.print(", errs[");
    Serial.print(i);
    Serial.print("] (pulses per 25.6 sec): ");
    Serial.println(errs[i]);
    
    P[i] = (int) round( errs[i] / kp ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))
    Serial.print("P[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(P[i]);

    D[i] = (int) round( ( ( rotVels[i] - prevRotVels[i] ) * 256 ) / kd ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
    Serial.print("D[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(D[i]);

    mtrOutAccums[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
    Serial.print("mtrOutAccums[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(mtrOutAccums[i]);

    prevRotVels[i] = rotVels[i]; // maintain history of previous measured rotVel

    mtrOutAccums[i] = clip(mtrOutAccums[i], maxOutVal, -maxOutVal); // accumulator
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < numMtrs; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }
  
  for(i=0; i < numMtrs; i++)
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    signs[mid] = -1;
  else if(signedVal >= 0)
    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");

  Serial.print("M");
  Serial.print(mid + 1);
  Serial.print(" speedometer sign: ");
  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  return (int) round( magnitude * 255.0 / 100 ); // cruise outputs
}
