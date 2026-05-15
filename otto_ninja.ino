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
// ================================================================


// ================================================================
// LIBRARIES
// ================================================================
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>
#include <RemoteXY.h>
#include <Servo.h>

#define REMOTEXY_WIFI_SSID      "OTTO NINJA2"
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
const int LA0 = 70;    // Left  leg neutral angle
const int RA0 = 110;   // Right leg neutral angle

const int RI  = 100;   // Roll mode leg tilt amount
const int WI  = 40;    // Walk mode weight-shift amount
const int WSI = 50;    // Walk mode swing leg raise amount

// Derived leg positions (calculated once in setup)
int LA1;    // Left  leg roll position
int RA1;    // Right leg roll position
int LATL;   // Left  leg tilt-left  position
int RATL;   // Right leg tilt-left  position
int LATR;   // Left  leg tilt-right position
int RATR;   // Right leg tilt-right position

// Foot step angles — how far the foot rotates per step.
const int LFFWRS = 30;   // Left  foot forward step
const int RFFWRS = 30;   // Right foot forward step
const int LFBWRS = 30;   // Left  foot backward step
const int RFBWRS = 30;   // Right foot backward step


// ================================================================
// SEQUENCE TIMING  (all times in milliseconds)
// ================================================================

// EXPLORE (push button) — ROLL mode
#define OBSTACLE_DISTANCE_CM   10     // Turn if obstacle is closer than this
#define EXPLORE_ROLL_ENTRY_MS  600    // Time to settle into roll position before moving
#define EXPLORE_TURN_MS        550    // How long to spin-turn for ~180° rotation
                                     // Increase if robot doesn't fully turn around

// Roll forward speed: feet angles while rolling forward in explore
// Both feet push the same direction to roll forward
#define ROLL_FWD_SPEED         40    // Degrees offset from 90 for forward rolling
                                     // Increase for faster roll, decrease for slower

// Roll turn: BOTH feet set to the SAME angle = true pivot in place.
// Forward rolls use OPPOSITE angles (left=135, right=45).
// Setting BOTH to 135 means left pushes forward, right pushes backward → robot spins.
// To reverse turn direction change both to 45 (left backward, right forward).
#define ROLL_TURN_ANGLE  135    // Both feet to this angle for pivot spin

// After a turn, ignore obstacles for this many ms so robot clears the wall
#define EXPLORE_CLEAR_MS      800    // Time to roll forward before checking again

// WAVE (button_A)
#define WAVE_TILT_MS          500    // Time to settle into standing tilt
#define WAVE_UP_MS            500   // Time for right leg to lift up
#define WAVE_DOWN_MS          500    // Time for right leg to lower down
#define WAVE_REPEATS            2    // Number of up/down waves
#define WAVE_LEG_LIFT          15    // Degrees the right leg waves up/down

// CIRCLE (button_B)
#define CIRCLE_DURATION_MS   4000    // How long to spin in circle
#define CIRCLE_FOOT_SPEED      20    // Left foot spin speed


// ================================================================
// STATE VARIABLES
// ================================================================
int  ModeCounter = 0;   // 0 = Walk mode,  1 = Roll mode

// Walk gait timer
unsigned long walkCycleStart = 0;

// Push button (EXPLORE) — now 3 phases:
//   Phase 0 = idle
//   Phase 1 = entering roll position (settling)
//   Phase 2 = rolling forward + obstacle watch
//   Phase 3 = spinning to avoid obstacle (~180°)
bool          seqActive         = false;
int           seqPhase          = 0;
unsigned long seqPhaseStart     = 0;
bool          clearingAfterTurn = false;   // ignore sensor briefly after each turn
unsigned long clearStart        = 0;

// Edge detection for push button — latched flag cleared each loop
bool prevButtonState  = false;
bool buttonEdge       = false;   // true for exactly one loop when button goes LOW→HIGH

// RemoteXY actions (WAVE / CIRCLE)
bool          rxActionActive  = false;
int           rxActionType    = 0;    // 1 = WAVE,  2 = CIRCLE
int           rxPhase         = 0;
unsigned long rxPhaseStart    = 0;
int           rxWaveCount     = 0;

// RemoteXY button edge detection
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
  if (duration == 0) return 999;   // no echo = clear path
  return duration * 0.034 / 2;
}


// ================================================================
// HELPER — STAND NEUTRAL (walk standing position)
// ================================================================
void standNeutral()
{
  leftLeg.write(LA0);
  rightLeg.write(RA0);
  leftFoot.write(90);
  rightFoot.write(90);
}


// ================================================================
// HELPER — ROLL NEUTRAL (roll standing position, feet flat)
// ================================================================
void rollNeutral()
{
  leftLeg.write(LA1);
  rightLeg.write(RA1);
  leftFoot.write(90);
  rightFoot.write(90);
}


// ================================================================
// WALK FORWARD GAIT
// Call this repeatedly every loop iteration while walking forward.
// Non-blocking — uses walkCycleStart timer.
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
    leftFoot.write(90);
    rightFoot.write(90);
  }
  if (e >= P1 - Overlap && e <= P2) {
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
    leftLeg.write(LATR);
  }
  if (e > P2 && e <= P3) {
    rightFoot.write(90 - RFFWRS);
  }
  if (e > P3 && e <= P4) {
    rightFoot.write(90);
    leftLeg.write(LA0);
    rightLeg.write(RA0);


  }

  if (e > P4 && e <= P5) {
    rightLeg.write(RATL);
    leftLeg.write(LA0);
    leftFoot.write(90);
  }
  if (e >= P5 - Overlap && e <= P6) {
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
    rightLeg.write(RATL);
  }
  if (e > P6 && e <= P7) {
    leftFoot.write(90 + LFFWRS);
  }
  if (e > P7 && e <= FullCycle) {
    leftFoot.write(90);
    leftLeg.write(LA0);
    rightLeg.write(RA0);
  }
}


// ================================================================
// SETUP
// ================================================================
void setup()
{
  leftFoot.attach(SERVO_LEFT_FOOT_PIN,  544, 2400);
  rightFoot.attach(SERVO_RIGHT_FOOT_PIN,544, 2400);
  leftLeg.attach(SERVO_LEFT_LEG_PIN,   544, 2400);
  rightLeg.attach(SERVO_RIGHT_LEG_PIN, 544, 2400);

  LA1  = LA0 + RI;
  RA1  = RA0 - RI;
  LATL = LA0 + WI;
  RATL = RA0 + WSI;
  LATR = LA0 - WSI;
  RATR = RA0 - WI;

  pinMode(PUSH_BUTTON_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  standNeutral();
  delay(500);

  Serial.begin(250000);
  RemoteXY_Init();
  Serial.println("OTTO NINJA ready.");
  Serial.println("D7=EXPLORE(roll) | A=WAVE | B=CIRCLE | X=Roll | Y=Walk");
}


// ================================================================
// MAIN LOOP
// ================================================================
void loop()
{
  RemoteXY_Handler();

  // ── Push button edge detection ────────────────────────────────
  // buttonEdge is true only on the single loop where button goes LOW→HIGH.
  // We read it once here and use the local variable everywhere below,
  // which ensures start and stop can never both trigger in the same press.
  bool currBtn   = (digitalRead(PUSH_BUTTON_PIN) == HIGH);
  buttonEdge     = (currBtn && !prevButtonState);
  prevButtonState = currBtn;

  // ── RemoteXY button edge detection ───────────────────────────
  bool btnAPressed = (RemoteXY.button_A && !prevBtnA);
  bool btnBPressed = (RemoteXY.button_B && !prevBtnB);
  prevBtnA = RemoteXY.button_A;
  prevBtnB = RemoteXY.button_B;


  // ================================================================
  // PUSH BUTTON — START EXPLORE (only when not already active)
  // ================================================================
  if (buttonEdge && !seqActive && !rxActionActive)
  {
    seqActive     = true;
    seqPhase      = 1;          // Phase 1: enter roll position
    seqPhaseStart = millis();
    Serial.println(">>> EXPLORE started — entering roll position");

    // Move legs to roll position immediately; feet stay flat
    leftLeg.write(LA1);
    rightLeg.write(RA1);
    leftFoot.write(90);
    rightFoot.write(90);
  }


  // ================================================================
  // EXPLORE STATE MACHINE
  // ================================================================
  if (seqActive)
  {
    unsigned long elapsed = millis() - seqPhaseStart;

    // ── Phase 1: Settle into roll position ───────────────────────
    // Just hold the roll position for EXPLORE_ROLL_ENTRY_MS ms so
    // the servos reach their target before the feet start spinning.
    if (seqPhase == 1)
    {
      // Keep legs in roll position while settling
      leftLeg.write(LA1);
      rightLeg.write(RA1);
      leftFoot.write(90);
      rightFoot.write(90);

      if (elapsed >= EXPLORE_ROLL_ENTRY_MS)
      {
        seqPhase      = 2;
        seqPhaseStart = millis();
        Serial.println(">>> Roll position reached — rolling forward");
      }
    }

    // ── Phase 2: Roll forward + obstacle detection ────────────────
    // Both feet spin forward. Legs stay in roll position (LA1/RA1).
    // After each turn, obstacle check is suppressed for EXPLORE_CLEAR_MS
    // so the robot physically moves away before it can trigger again.
    else if (seqPhase == 2)
    {
      // Keep legs in roll position
      leftLeg.write(LA1);
      rightLeg.write(RA1);

      // Both feet spin the same direction to roll forward
      leftFoot.write(90 + ROLL_FWD_SPEED);
      rightFoot.write(90 - ROLL_FWD_SPEED);

      // Expire the post-turn clearing window
      if (clearingAfterTurn && (millis() - clearStart >= EXPLORE_CLEAR_MS))
      {
        clearingAfterTurn = false;
        Serial.println(">>> Obstacle sensor re-enabled");
      }

      // Check for obstacle only outside the clearing window
      if (!clearingAfterTurn)
      {
        long dist = readDistanceCM();
        if (dist > 0 && dist < OBSTACLE_DISTANCE_CM)
        {
          Serial.print(">>> Obstacle at ");
          Serial.print(dist);
          Serial.println(" cm — spinning 180");

          // Stop feet before spinning
          leftFoot.write(90);
          rightFoot.write(90);
          delay(150);

          seqPhase      = 3;
          seqPhaseStart = millis();
        }
      }

      // Second button press stops exploration
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

    // ── Phase 3: Spin 180° to avoid obstacle ────────────────────────
    // Feet spin in OPPOSITE directions the entire time — this is what
    // actually rotates the robot. The loop must NOT call readDistanceCM()
    // here so the sensor cannot interrupt or reset the turn mid-way.
    // After EXPLORE_TURN_MS the turn is done; a clearing window then
    // suppresses the sensor in Phase 2 until the robot is clear.
    else if (seqPhase == 3)
    {
      // Keep legs in roll position
      leftLeg.write(LA1);
      rightLeg.write(RA1);

      // BOTH feet same angle = true pivot spin in place.
      // Forward = left 135 / right 45 (opposite).
      // Pivot   = left 135 / right 135 (same) → left pushes fwd, right pushes bwd.
      leftFoot.write(ROLL_TURN_ANGLE);
      rightFoot.write(ROLL_TURN_ANGLE);

      if (elapsed >= EXPLORE_TURN_MS)
      {
        // Brief pause, then enable clearing window before rolling again
        leftFoot.write(90);
        rightFoot.write(90);
        delay(150);

        clearingAfterTurn = true;
        clearStart        = millis();

        seqPhase      = 2;
        seqPhaseStart = millis();
        Serial.println(">>> 180 turn complete — rolling forward");
      }

      // Button press also stops during a turn
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

    // Block joystick and other controls while EXPLORE is running
    return;
  }


  // ================================================================
  // REMOTEXY ACTIONS — WAVE (A) and CIRCLE (B)
  // Only available in Walk mode (ModeCounter == 0)
  // ================================================================

  if (!rxActionActive && ModeCounter == 0)
  {
    if (btnAPressed)
    {
      rxActionActive = true;
      rxActionType   = 1;   // WAVE
      rxPhase        = 1;
      rxPhaseStart   = millis();
      rxWaveCount    = 0;
      Serial.println(">>> WAVE started");
    }
    else if (btnBPressed)
    {
      rxActionActive = true;
      rxActionType   = 2;   // CIRCLE
      rxPhase        = 1;
      rxPhaseStart   = millis();
      Serial.println(">>> CIRCLE started");
    }
  }

  if (rxActionActive)
  {
    unsigned long elapsed = millis() - rxPhaseStart;

    // ──────────────────────────────────────────────────────────
    // WAVE
    // ──────────────────────────────────────────────────────────
    if (rxActionType == 1)
    {
      if (rxPhase == 1)
      {
        leftLeg.write(LA0 + WI);
        rightLeg.write(RA0 + WSI);
        leftFoot.write(90);
        rightFoot.write(90);

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

    // ──────────────────────────────────────────────────────────
    // CIRCLE
    // ──────────────────────────────────────────────────────────
    else if (rxActionType == 2)
    {
      if (rxPhase == 1)
      {
        leftLeg.write(LA0 + WI);
        rightLeg.write(RA0 + WSI);
        leftFoot.write(90 + CIRCLE_FOOT_SPEED);
        rightFoot.write(90);

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

    return;   // block joystick while action runs
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
      if (e >= I1 && e <= I2) rightFoot.write(90 + RFBWRS);
      if (e >= I2 && e <= I3) {
        rightFoot.write(90);
        leftLeg.write(LATL);
        rightLeg.write(RATL);
      }
      if (e >= I3 && e <= I4) leftFoot.write(90 - LFBWRS);
      if (e >= I4 && e <= I5) leftFoot.write(90);
    }
  }

  // ── Roll mode ──────────────────────────────────────────────────
  if (ModeCounter == 1)
  {
    if (joystickIdle)
    {
      leftFoot.write(90);
      rightFoot.write(90);
      return;
    }

    int LWS = map(RemoteXY.J_y, 100, -100, 135,  45);
    int RWS = map(RemoteXY.J_y, 100, -100,  45, 135);
    int LWD = map(RemoteXY.J_x, 100, -100,  45,   0);
    int RWD = map(RemoteXY.J_x, 100, -100,   0, -45);
    leftFoot.write(LWS + LWD);
    rightFoot.write(RWS + RWD);
  }
}
