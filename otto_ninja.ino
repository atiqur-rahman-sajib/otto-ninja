// ================================================================
// OTTO NINJA — DUAL MODE
//
// Push Button (D7)  → EXPLORE: enter ROLL position, roll forward,
//                     avoid obstacles (<10 cm) by spinning in place
//                     Press again to stop and return to stand.
// RemoteXY button_Y → Switch to Walk mode
// RemoteXY button_X → Switch to Roll mode
// RemoteXY button_A → WAVE: stand on left leg, wave right leg 2x
// RemoteXY button_B → CIRCLE: stand on left leg, left foot spins circle
//
// All servos are permanently attached — no mid-code attach/detach.
//
// FIX: 360° continuous rotation servos rarely stop exactly at 90.
//      FOOT_STOP_L and FOOT_STOP_R are the true neutral (no-creep)
//      values for each foot servo. Tune them by writing a fixed angle
//      until both foot servos go completely still at power-on.
//      Try values 85–95 one at a time until creep stops.
// ================================================================


// ================================================================
// LIBRARIES
// ================================================================
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>
#include <RemoteXY.h>
#include <Servo.h>

#define REMOTEXY_WIFI_SSID      "OTTO NINJA"
#define REMOTEXY_WIFI_PASSWORD  "12345678"
#define REMOTEXY_SERVER_PORT    6377

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,6,0,0,0,66,0,13,8,0,
  5,32,3,12,41,41,1,26,31,1,
  3,79,16,16,12,1,31,82,240,159,
  166,190,0,1,3,56,39,18,12,1,
  31,240,159,146,191,0,1,3,79,39,
  17,12,1,31,240,159,166,191,0,1,
  3,56,16,17,12,1,31,76,240,159,
  166,190,0 };

struct {
  int8_t  J_x;
  int8_t  J_y;
  uint8_t button_B;
  uint8_t button_X;
  uint8_t button_Y;
  uint8_t button_A;
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)


// ================================================================
// PIN DEFINITIONS
// ================================================================
#define SERVO_LEFT_FOOT_PIN   5    // D1
#define SERVO_LEFT_LEG_PIN    4    // D2
#define SERVO_RIGHT_FOOT_PIN  0    // D3
#define SERVO_RIGHT_LEG_PIN   2    // D4

#define PUSH_BUTTON_PIN      13    // D7 — HIGH = pressed
#define TRIG_PIN             14    // D5 — ultrasonic trigger
#define ECHO_PIN             12    // D6 — ultrasonic echo


// ================================================================
// SERVO OBJECTS
// ================================================================
Servo leftFoot;
Servo leftLeg;
Servo rightFoot;
Servo rightLeg;


// ================================================================
// CALIBRATION
// Adjust LA0 and RA0 to make the robot stand perfectly upright.
// All other values are derived from these two.
// ================================================================
const int LA0 = 65;    // Left  leg neutral angle
const int RA0 = 110;   // Right leg neutral angle

const int RI  = 90;    // Roll mode leg tilt amount
const int WI  = 40;    // Walk mode weight-shift amount
const int WSI = 50;    // Walk mode swing leg raise amount

// ----------------------------------------------------------------
// 360° FOOT SERVO STOP CALIBRATION
// These are the angles at which each foot servo is truly still.
// Default is 90 but most units creep at that value.
// Tune FOOT_STOP_L and FOOT_STOP_R independently:
//   - Upload, power on, watch each foot servo.
//   - If left foot creeps forward → lower FOOT_STOP_L by 1–2.
//   - If left foot creeps backward → raise FOOT_STOP_L by 1–2.
//   - Repeat for right foot with FOOT_STOP_R.
// ----------------------------------------------------------------
const int FOOT_STOP_L = 90;   // ← tune this (try 88–93)
const int FOOT_STOP_R = 90;   // ← tune this (try 88–93)

// Derived leg positions (calculated once in setup)
int LA1;    // Left  leg roll position
int RA1;    // Right leg roll position
int LATL;   // Left  leg tilt-left  position
int RATL;   // Right leg tilt-left  position
int LATR;   // Left  leg tilt-right position
int RATR;   // Right leg tilt-right position

// Foot step angles — how far the foot rotates per step.
const int LFFWRS = 20;   // Left  foot forward step
const int RFFWRS = 20;   // Right foot forward step
const int LFBWRS = 20;   // Left  foot backward step
const int RFBWRS = 20;   // Right foot backward step


// ================================================================
// SEQUENCE TIMING  (all times in milliseconds)
// ================================================================

// EXPLORE (push button) — ROLL mode
#define OBSTACLE_DISTANCE_CM   10     // Turn if obstacle is closer than this
#define EXPLORE_ROLL_ENTRY_MS  600    // Time to settle into roll position before moving
#define EXPLORE_TURN_MS        550    // How long to spin-turn for ~180° rotation

// Roll forward speed
#define ROLL_FWD_SPEED         40    // Degrees offset from stop for forward rolling

// Roll turn: BOTH feet set to the SAME angle = true pivot in place.
#define ROLL_TURN_ANGLE  135    // Both feet to this angle for pivot spin

// After a turn, ignore obstacles for this many ms so robot clears the wall
#define EXPLORE_CLEAR_MS      800

// WAVE (button_A)
#define WAVE_TILT_MS          500
#define WAVE_UP_MS            500
#define WAVE_DOWN_MS          500
#define WAVE_REPEATS            2
#define WAVE_LEG_LIFT          15

// CIRCLE (button_B)
#define CIRCLE_DURATION_MS   4000
#define CIRCLE_FOOT_SPEED      20


// ================================================================
// STATE VARIABLES
// ================================================================
int  ModeCounter = 0;   // 0 = Walk mode,  1 = Roll mode

unsigned long walkCycleStart = 0;

bool          seqActive         = false;
int           seqPhase          = 0;
unsigned long seqPhaseStart     = 0;
bool          clearingAfterTurn = false;
unsigned long clearStart        = 0;

bool prevButtonState  = false;
bool buttonEdge       = false;

bool          rxActionActive  = false;
int           rxActionType    = 0;
int           rxPhase         = 0;
unsigned long rxPhaseStart    = 0;
int           rxWaveCount     = 0;

uint8_t prevBtnA = 0;
uint8_t prevBtnB = 0;


// ================================================================
// HELPER — ULTRASONIC DISTANCE
// ================================================================
long readDistanceCM()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}


// ================================================================
// HELPER — STAND NEUTRAL
// Uses calibrated FOOT_STOP values so 360° servos don't creep.
// ================================================================
void standNeutral()
{
  leftLeg.write(LA0);
  rightLeg.write(RA0);
  leftFoot.write(FOOT_STOP_L);
  rightFoot.write(FOOT_STOP_R);
}


// ================================================================
// HELPER — ROLL NEUTRAL
// ================================================================
void rollNeutral()
{
  leftLeg.write(LA1);
  rightLeg.write(RA1);
  leftFoot.write(FOOT_STOP_L);
  rightFoot.write(FOOT_STOP_R);
}


// ================================================================
// WALK FORWARD GAIT
// ================================================================
void walkForward()
{
  const int Interval     = 250;
  const int Overlap      = 150;
  const int BaseInterval = 500;
  const int FootHold     = 188;

  const int FullCycle = (Interval * 2) + FootHold + BaseInterval +
                        (Interval * 2) + FootHold + BaseInterval;

  if (millis() > walkCycleStart + FullCycle)
    walkCycleStart = millis();

  long e = millis() - walkCycleStart;

  const long P1 = Interval;
  const long P2 = P1 + Interval;
  const long P3 = P2 + FootHold;
  const long P4 = P3 + BaseInterval;
  const long P5 = P4 + Interval;
  const long P6 = P5 + Interval;
  const long P7 = P6 + FootHold;

  if (e <= P1) {
    leftLeg.write(LATR);
    rightLeg.write(RA0);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }
  if (e >= P1 - Overlap && e <= P2) {
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
    leftLeg.write(LATR);
  }
  if (e > P2 && e <= P3) {
    rightFoot.write(FOOT_STOP_R - RFFWRS);
  }
  if (e > P3 && e <= P4) {
    rightFoot.write(FOOT_STOP_R);
    leftLeg.write(LA0);
    rightLeg.write(RA0);
  }

  if (e > P4 && e <= P5) {
    rightLeg.write(RATL);
    leftLeg.write(LA0);
    leftFoot.write(FOOT_STOP_L);
  }
  if (e >= P5 - Overlap && e <= P6) {
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
    rightLeg.write(RATL);
  }
  if (e > P6 && e <= P7) {
    leftFoot.write(FOOT_STOP_L + LFFWRS);
  }
  if (e > P7 && e <= FullCycle) {
    leftFoot.write(FOOT_STOP_L);
    leftLeg.write(LA0);
    rightLeg.write(RA0);
  }
}


// ================================================================
// SETUP
// ================================================================
void setup()
{
  // Attach servos first
  leftFoot.attach(SERVO_LEFT_FOOT_PIN,  544, 2400);
  rightFoot.attach(SERVO_RIGHT_FOOT_PIN,544, 2400);
  leftLeg.attach(SERVO_LEFT_LEG_PIN,   544, 2400);
  rightLeg.attach(SERVO_RIGHT_LEG_PIN, 544, 2400);

  // Calculate derived positions
  LA1  = LA0 + RI;
  RA1  = RA0 - RI;
  LATL = LA0 + WI;
  RATL = RA0 + WSI;
  LATR = LA0 - WSI;
  RATR = RA0 - WI - 8;

  pinMode(PUSH_BUTTON_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(250000);

  // Init RemoteXY BEFORE writing servo positions.
  // This closes the boot window where servos could receive
  // PWM with no logical owner and creep unpredictably.
  RemoteXY_Init();

  // Now write calibrated stop angles — feet go truly still.
  standNeutral();
  delay(500);

  Serial.println("OTTO NINJA ready.");
  Serial.println("D7=EXPLORE(roll) | A=WAVE | B=CIRCLE | X=Roll | Y=Walk");
  Serial.print("Foot stop angles: L=");
  Serial.print(FOOT_STOP_L);
  Serial.print("  R=");
  Serial.println(FOOT_STOP_R);
  Serial.println("If feet still creep, tune FOOT_STOP_L / FOOT_STOP_R.");
}


// ================================================================
// MAIN LOOP
// ================================================================
void loop()
{
  RemoteXY_Handler();

  // ── Push button edge detection ────────────────────────────────
  bool currBtn    = (digitalRead(PUSH_BUTTON_PIN) == HIGH);
  buttonEdge      = (currBtn && !prevButtonState);
  prevButtonState = currBtn;

  // ── RemoteXY button edge detection ───────────────────────────
  bool btnAPressed = (RemoteXY.button_A && !prevBtnA);
  bool btnBPressed = (RemoteXY.button_B && !prevBtnB);
  prevBtnA = RemoteXY.button_A;
  prevBtnB = RemoteXY.button_B;


  // ================================================================
  // PUSH BUTTON — START EXPLORE
  // ================================================================
  if (buttonEdge && !seqActive && !rxActionActive)
  {
    seqActive     = true;
    seqPhase      = 1;
    seqPhaseStart = millis();
    Serial.println(">>> EXPLORE started — entering roll position");

    leftLeg.write(LA1);
    rightLeg.write(RA1);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }


  // ================================================================
  // EXPLORE STATE MACHINE
  // ================================================================
  if (seqActive)
  {
    unsigned long elapsed = millis() - seqPhaseStart;

    // ── Phase 1: Settle into roll position ───────────────────────
    if (seqPhase == 1)
    {
      leftLeg.write(LA1);
      rightLeg.write(RA1);
      leftFoot.write(FOOT_STOP_L);
      rightFoot.write(FOOT_STOP_R);

      if (elapsed >= EXPLORE_ROLL_ENTRY_MS)
      {
        seqPhase      = 2;
        seqPhaseStart = millis();
        Serial.println(">>> Roll position reached — rolling forward");
      }
    }

    // ── Phase 2: Roll forward + obstacle detection ────────────────
    else if (seqPhase == 2)
    {
      leftLeg.write(LA1);
      rightLeg.write(RA1);

      leftFoot.write(FOOT_STOP_L + ROLL_FWD_SPEED);
      rightFoot.write(FOOT_STOP_R - ROLL_FWD_SPEED);

      if (clearingAfterTurn && (millis() - clearStart >= EXPLORE_CLEAR_MS))
      {
        clearingAfterTurn = false;
        Serial.println(">>> Obstacle sensor re-enabled");
      }

      if (!clearingAfterTurn)
      {
        long dist = readDistanceCM();
        if (dist > 0 && dist < OBSTACLE_DISTANCE_CM)
        {
          Serial.print(">>> Obstacle at ");
          Serial.print(dist);
          Serial.println(" cm — spinning 180");

          leftFoot.write(FOOT_STOP_L);
          rightFoot.write(FOOT_STOP_R);
          delay(150);

          seqPhase      = 3;
          seqPhaseStart = millis();
        }
      }

      if (buttonEdge)
      {
        rollNeutral();
        delay(300);
        standNeutral();
        seqActive         = false;
        seqPhase          = 0;
        clearingAfterTurn = false;
        Serial.println(">>> EXPLORE stopped by button");
        return;
      }
    }

    // ── Phase 3: Spin 180° ───────────────────────────────────────
    else if (seqPhase == 3)
    {
      leftLeg.write(LA1);
      rightLeg.write(RA1);

      leftFoot.write(ROLL_TURN_ANGLE);
      rightFoot.write(ROLL_TURN_ANGLE);

      if (elapsed >= EXPLORE_TURN_MS)
      {
        leftFoot.write(FOOT_STOP_L);
        rightFoot.write(FOOT_STOP_R);
        delay(150);

        clearingAfterTurn = true;
        clearStart        = millis();

        seqPhase      = 2;
        seqPhaseStart = millis();
        Serial.println(">>> 180 turn complete — rolling forward");
      }

      if (buttonEdge)
      {
        rollNeutral();
        delay(300);
        standNeutral();
        seqActive         = false;
        seqPhase          = 0;
        clearingAfterTurn = false;
        Serial.println(">>> EXPLORE stopped by button (during turn)");
        return;
      }
    }

    return;
  }


  // ================================================================
  // REMOTEXY ACTIONS — WAVE (A) and CIRCLE (B)
  // ================================================================
  if (!rxActionActive && ModeCounter == 0)
  {
    if (btnAPressed)
    {
      rxActionActive = true;
      rxActionType   = 1;
      rxPhase        = 1;
      rxPhaseStart   = millis();
      rxWaveCount    = 0;
      Serial.println(">>> WAVE started");
    }
    else if (btnBPressed)
    {
      rxActionActive = true;
      rxActionType   = 2;
      rxPhase        = 1;
      rxPhaseStart   = millis();
      Serial.println(">>> CIRCLE started");
    }
  }

  if (rxActionActive)
  {
    unsigned long elapsed = millis() - rxPhaseStart;

    // ── WAVE ──────────────────────────────────────────────────────
    if (rxActionType == 1)
    {
      if (rxPhase == 1)
      {
        leftLeg.write(LA0 + WI);
        rightLeg.write(RA0 + WSI);
        leftFoot.write(FOOT_STOP_L);
        rightFoot.write(FOOT_STOP_R);

        if (elapsed >= WAVE_TILT_MS)
        {
          rxPhase      = 2;
          rxPhaseStart = millis();
        }
      }
      else if (rxPhase == 2)
      {
        rightLeg.write(RA0 + WSI + WAVE_LEG_LIFT);

        if (elapsed >= WAVE_UP_MS)
        {
          rxPhase      = 3;
          rxPhaseStart = millis();
        }
      }
      else if (rxPhase == 3)
      {
        rightLeg.write(RA0 + WSI);

        if (elapsed >= WAVE_DOWN_MS)
        {
          rxWaveCount++;
          if (rxWaveCount < WAVE_REPEATS)
          {
            rxPhase      = 2;
            rxPhaseStart = millis();
          }
          else
          {
            rxPhase      = 4;
            rxPhaseStart = millis();
            Serial.println(">>> WAVE: returning to stand");
          }
        }
      }
      else if (rxPhase == 4)
      {
        standNeutral();

        if (elapsed >= 600)
        {
          rxActionActive = false;
          rxPhase        = 0;
          rxWaveCount    = 0;
          Serial.println(">>> WAVE complete");
        }
      }
    }

    // ── CIRCLE ────────────────────────────────────────────────────
    else if (rxActionType == 2)
    {
      if (rxPhase == 1)
      {
        rightLeg.write(RA0 + WSI);
        delay(40);
        leftLeg.write(LA0 + WI);
        leftFoot.write(FOOT_STOP_L + CIRCLE_FOOT_SPEED);
        rightFoot.write(FOOT_STOP_R);

        if (elapsed >= CIRCLE_DURATION_MS)
        {
          rxPhase      = 2;
          rxPhaseStart = millis();
          Serial.println(">>> CIRCLE: returning to stand");
        }
      }
      else if (rxPhase == 2)
      {
        standNeutral();

        if (elapsed >= 600)
        {
          rxActionActive = false;
          rxPhase        = 0;
          Serial.println(">>> CIRCLE complete");
        }
      }
    }

    return;
  }


  // ================================================================
  // REMOTEXY — MODE SWITCH BUTTONS
  // ================================================================
  if (RemoteXY.button_X) { ModeCounter = 1; leftLeg.write(LA1); rightLeg.write(RA1); }
  if (RemoteXY.button_Y) { ModeCounter = 0; standNeutral(); }


  // ================================================================
  // REMOTEXY — JOYSTICK CONTROL
  // ================================================================
  bool joystickIdle = (RemoteXY.J_x >= -10 && RemoteXY.J_x <= 10 &&
                       RemoteXY.J_y >= -10 && RemoteXY.J_y <= 10);

  // ── Walk mode ──────────────────────────────────────────────────
  if (ModeCounter == 0)
  {
    if (joystickIdle)
    {
      standNeutral();
      return;
    }

    if (RemoteXY.J_y > 0)
    {
      walkForward();
    }

    if (RemoteXY.J_y < 0)
    {
      const int lt = map(RemoteXY.J_x, 100, -100, 200, 700);
      const int rt = map(RemoteXY.J_x, 100, -100, 700, 200);
      const int I1 = 250;
      const int I2 = I1 + rt;
      const int I3 = I2 + 250;
      const int I4 = I3 + lt;
      const int I5 = I4 + 50;

      if (millis() > walkCycleStart + I5) walkCycleStart = millis();
      long e = millis() - walkCycleStart;

      if (e <= I1) {
        leftLeg.write(LATR);
        rightLeg.write(RATR);
      }
      if (e >= I1 && e <= I2) rightFoot.write(FOOT_STOP_R + RFBWRS);
      if (e >= I2 && e <= I3) {
        rightFoot.write(FOOT_STOP_R);
        leftLeg.write(LATL);
        rightLeg.write(RATL);
      }
      if (e >= I3 && e <= I4) leftFoot.write(FOOT_STOP_L - LFBWRS);
      if (e >= I4 && e <= I5) leftFoot.write(FOOT_STOP_L);
    }
  }

  // ── Roll mode ──────────────────────────────────────────────────
  if (ModeCounter == 1)
  {
    if (joystickIdle)
    {
      leftFoot.write(FOOT_STOP_L);
      rightFoot.write(FOOT_STOP_R);
      return;
    }

    int LWS = map(RemoteXY.J_y, 100, -100, FOOT_STOP_L + 45, FOOT_STOP_L - 45);
    int RWS = map(RemoteXY.J_y, 100, -100, FOOT_STOP_R - 45, FOOT_STOP_R + 45);
    int LWD = map(RemoteXY.J_x, 100, -100,  45,   0);
    int RWD = map(RemoteXY.J_x, 100, -100,   0, -45);
    leftFoot.write(LWS + LWD);
    rightFoot.write(RWS + RWD);
  }
}
