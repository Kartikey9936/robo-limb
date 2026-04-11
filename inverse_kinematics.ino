
#include <Servo.h>
#include <math.h>

// ── Link lengths (mm) ─────────────────────────────────────────────────────
#define L1      200.00f
#define L2      270.00f
#define DX       0.0f
#define DZ      150.00f



// ── Servo gap between joints ──────────────────────────────────────────────
#define SERVO_GAP_MS    200

// ── Gripper angles ────────────────────────────────────────────────────────
#define GRIPPER_OPEN    150
#define GRIPPER_CLOSED   0

// ── Servo config ──────────────────────────────────────────────────────────
// reversed = true  → flips physical rotation direction
// reversed = false → normal direction

struct ServoConfig {
    float min_a, max_a;
    int   pin;
    int   min_us, max_us;
    bool  reversed;
};

//                   min°   max°  pin  minµs  maxµs   reversed
ServoConfig CONFIG[] = {
    {  0.0, 180.0,  9,  500, 2500, false },  // [0] Base     — 40kg
    {  0.0, 180.0,  5, 500, 2500, true },  // [1] Shoulder — set true to reverse
    {  0.0, 180.0, 11, 500, 2500, true },  // [2] Elbow    — set true to reverse
    {  0.0, 180.0, 12, 500, 2000, false },  // [3] Gripper  — set true to reverse
};


// ── Per-servo motion profile ──────────────────────────────────────────────
struct MotionProfile { float stepDeg; int delayMs; };

MotionProfile PROFILE[] = {
    { 1.0f,  60 },   // [0] Base     — 40kg: slow & smooth
    { 1.0f,  80 },   // [1] Shoulder — standard
    { 1.0f,  100 },   // [2] Elbow    — standard
    { 180.0f,  0 },   // [3] Gripper  — standard
};

Servo servos[4];
// float currentAngles[4] = {90, 90, 90, GRIPPER_OPEN};
float currentAngles[4] = {90, 166, 180, GRIPPER_OPEN};

const char* NAMES[] = {"Base", "Shoulder", "Elbow", "Gripper"};

// ── Move orders ───────────────────────────────────────────────────────────
const int MOVE_ORDER[] = {0, 2, 1};   // To target:  Base → Elbow → Shoulder
const int HOME_ORDER[] = {1, 0, 2};   // To home:    Base → Shoulder → Elbow

// ── Inverse Kinematics ────────────────────────────────────────────────────
bool ikArm(float x, float y, float z, float out[3]) {
    float theta_base = atan2f(y, x);

    float r_eff = sqrtf(x*x + y*y) - DX;
    float z_eff = z - DZ;

    float D = sqrtf(r_eff*r_eff + z_eff*z_eff);
    if (D > (L1 + L2) || D < fabsf(L1 - L2)) return false;

    float cos_el      = (L1*L1 + L2*L2 - D*D) / (2.0f * L1 * L2);
    float theta_elbow = M_PI - acosf(constrain(cos_el, -1.0f, 1.0f));

    float phi            = atan2f(z_eff, r_eff);
    float cos_psi        = (L1*L1 + D*D - L2*L2) / (2.0f * L1 * D);
    float theta_shoulder = phi + acosf(constrain(cos_psi, -1.0f, 1.0f));

    out[0] = theta_base     * 180.0f / M_PI + 90.0f;
    out[1] = theta_shoulder * 180.0f / M_PI + 90.0f;
    out[2] = theta_elbow    * 180.0f / M_PI + 90.0f;
    

    return true;
}

// ── Write angle to servo (handles reversal + pulse width mapping) ─────────
// This is the ONLY place physical angle is written to hardware.
// Reversal happens here — IK and motion logic never need to change.
void writeAngle(int index, float angleDeg) {
    // flip direction if reversed flag is set
    float physicalAngle = CONFIG[index].reversed
                          ? 180.0f - angleDeg
                          : angleDeg;

    // map angle to microseconds and write
    int us = map((int)physicalAngle, 0, 180,
                 CONFIG[index].min_us,
                 CONFIG[index].max_us);

    servos[index].writeMicroseconds(us);
}

// ── Move a single servo smoothly using its motion profile ─────────────────
void moveSingleServo(int index, float target) {
    target = constrain(target, CONFIG[index].min_a, CONFIG[index].max_a);

    float stepDeg = PROFILE[index].stepDeg;
    int   delayMs = PROFILE[index].delayMs;

    while (true) {
        float diff = target - currentAngles[index];
        if (fabsf(diff) < 0.5f) {
            currentAngles[index] = target;
            writeAngle(index, currentAngles[index]);
            break;
        }
        if (diff > 0)
            currentAngles[index] += min(stepDeg,  diff);
        else
            currentAngles[index] -= min(stepDeg, -diff);

        writeAngle(index, currentAngles[index]);
        delay(delayMs);
    }
}

// ── Move arm joints in given order ───────────────────────────────────────
void writeInOrder(float targets[3], const int order[3]) {
    for (int i = 0; i < 3; i++) {
        int idx = order[i];
        Serial.print("Moving "); Serial.print(NAMES[idx]);
        Serial.print(" → ");
        Serial.print((int)constrain(targets[idx], 0, 180));
        Serial.println("°");

        moveSingleServo(idx, targets[idx]);

        Serial.print(NAMES[idx]); Serial.println(" done");
        delay(SERVO_GAP_MS);
    }
}

// ── Move gripper ──────────────────────────────────────────────────────────
void moveGripper(float angle) {
    Serial.print("Moving Gripper → ");
    Serial.print((int)angle); Serial.println("°");
    moveSingleServo(3, angle);
    Serial.println("Gripper done");
}


// ── Return to home ────────────────────────────────────────────────────────
void goHome() {
    float home[3] = {90, 175, 180};
    Serial.println("Returning to home...");
    writeInOrder(home, HOME_ORDER);
    delay(SERVO_GAP_MS);
    moveGripper(GRIPPER_OPEN);
    Serial.println("Home");
}


// ── Move arm to XYZ target ────────────────────────────────────────────────
bool moveTo(float x, float y, float z) {
    float angles[3];

    if (!ikArm(x, y, z, angles)) {
        Serial.print("[!] OUT OF REACH: ");
        Serial.print(x); Serial.print(", ");
        Serial.print(y); Serial.print(", ");
        Serial.println(z);
        return false;
    }

    Serial.println("─────────────────────");
    Serial.print("Target  X:"); Serial.print(x);
    Serial.print("  Y:");       Serial.print(y);
    Serial.print("  Z:");       Serial.println(z);
    Serial.print("Base     : "); Serial.println((int)constrain(angles[0], 0, 180));
    Serial.print("Shoulder : "); Serial.println((int)constrain(angles[1], 0, 180));
    Serial.print("Elbow    : "); Serial.println((int)constrain(angles[2], 0, 180));
    Serial.print("Gripper  : "); Serial.println(GRIPPER_CLOSED);

    writeInOrder(angles, MOVE_ORDER);
    delay(SERVO_GAP_MS);
    moveGripper(GRIPPER_CLOSED);

    Serial.println("OK");
    return true;
}

// ── Setup ─────────────────────────────────────────────────────────────────
// void setup() {
//     Serial.begin(115200);

//     // for (int i = 0; i < 4; i++) {
//     //     servos[i].attach(CONFIG[i].pin, CONFIG[i].min_us, CONFIG[i].max_us);
//     //     writeAngle(i, 90);     // centre each servo respecting reversal
//     //     delay(300);            // stagger startup — avoids inrush spike
//     // }
//     for (int i = 0; i < 4; i++) {
//     servos[i].attach(CONFIG[i].pin, CONFIG[i].min_us, CONFIG[i].max_us);
//     writeAngle(i, currentAngles[i]);   // start at home position
//     delay(300);
// }

//     delay(2000);
//     goHome();
// }


void setup() {
    Serial.begin(115200);

    // Step 1: Hold all pins LOW to prevent floating signals
    for (int i = 0; i < 4; i++) {
        pinMode(CONFIG[i].pin, OUTPUT);
        digitalWrite(CONFIG[i].pin, LOW);
    }
    delay(500); // Wait for power rail to fully stabilize

    // Step 2: Attach and write home one by one with generous delay
    for (int i = 0; i < 4; i++) {
        servos[i].attach(CONFIG[i].pin, CONFIG[i].min_us, CONFIG[i].max_us);
        writeAngle(i, currentAngles[i]);
        delay(500); // stagger to avoid current spike
    }

    delay(1000); // Final settle before goHome()
    goHome();
}

// ── Loop — receive "x,y,z" over Serial ───────────────────────────────────
void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);

        if (c1 < 0 || c2 < 0) {
            Serial.println("[!] Bad format. Use: x,y,z");
            return;
        }

        float x = line.substring(0,      c1).toFloat();
        float y = line.substring(c1 + 1, c2).toFloat();
        float z = line.substring(c2 + 1).toFloat();

        bool ok = moveTo(x, y, z);
        Serial.println(ok ? "OK" : "FAIL");

        delay(2000);
        goHome();
    }
}