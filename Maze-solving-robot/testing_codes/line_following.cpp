#include <Arduino.h>

/* =======================
   7-Sensor Line Follower (Clean)
   Board: Arduino Mega (pins 43..49 used)
   Sensors: digital (0 on black), IR enable on D10
   Motors: simple DIR + PWM driver per side
   ======================= */

// ---------- Config (EDIT THESE) ----------
const int IR_ENABLE = 10;                            // Flip HIGH/LOW if your board is active-LOW
const int SENSORS[7] = {43, 44, 45, 46, 47, 48, 49}; // left→right; reverse if needed

const int L_DIR = 8, L_PWM = 9; // Left motor pins
const int R_DIR = 5, R_PWM = 6; // Right motor pins

// Control gains & speed
int BASE_PWM = 50; // 0..255
float KP = 2.0f;   // Proportional gain
float KD = 0.1f;   // Derivative gain

// If your sensors output 0 on black (common), keep true
// If they output 1 on black, set to false
const bool ACTIVE_LOW = true;

// ---------- Internals (don’t edit) ----------
float lastError = 0.0f;
unsigned long lastPrintMs = 0;

// ---------- Small helpers ----------
void setMotor(int dirPin, int pwmPin, int speed)
{
    int mag = speed;
    if (mag < 0)
        mag = -mag;
    if (mag > 255)
        mag = 255;
    digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
    analogWrite(pwmPin, mag);
}

void drive(int left, int right)
{
    setMotor(L_DIR, L_PWM, left);
    setMotor(R_DIR, R_PWM, right);
}

// Return weighted line position in [-3..+3]; NAN if no sensor sees line
float readLineError()
{
    static const int W[7] = {-3, -2, -1, 0, 1, 2, 3};
    long sumW = 0;
    int n = 0;
    for (int i = 0; i < 7; ++i)
    {
        int v = digitalRead(SENSORS[i]);
        bool onLine = ACTIVE_LOW ? (v == LOW) : (v == HIGH);
        if (onLine)
        {
            sumW += W[i];
            ++n;
        }
    }
    if (n == 0)
        return NAN;
    return (float)sumW / (float)n;
}

// ---------- Setup / Loop ----------
void setup()
{
    Serial.begin(9600);

    for (int i = 0; i < 7; ++i)
        pinMode(SENSORS[i], INPUT);

    pinMode(IR_ENABLE, OUTPUT);
    digitalWrite(IR_ENABLE, HIGH); // Try HIGH first; if always 1s, try LOW

    pinMode(L_DIR, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);
    pinMode(R_PWM, OUTPUT);

    drive(0, 0);
    delay(300);
}

void loop()
{
    float err = readLineError();

    if (isnan(err))
    {
        // Line lost: gently turn toward last seen side
        int s = 90;
        if (lastError >= 0)
            drive(+s, -s);
        else
            drive(-s, +s);
    }
    else
    {
        // PD steering
        float d = err - lastError;
        float corr = KP * err + KD * d;

        int left = BASE_PWM - (int)corr;
        int right = BASE_PWM + (int)corr;

        // Clamp speeds
        if (left > 255)
            left = 255;
        if (left < -255)
            left = -255;
        if (right > 255)
            right = 255;
        if (right < -255)
            right = -255;

        drive(left, right);
        lastError = err;
    }

    // Light debug (10 Hz)
    unsigned long now = millis();
    if (now - lastPrintMs > 100)
    {
        Serial.print("err=");
        if (isnan(err))
            Serial.print("NaN");
        else
            Serial.print(err, 2);
        Serial.print("  PWM(L,R)=");
        // show commanded forward (no clamp) for feel
        Serial.print(BASE_PWM - (int)(KP * (isnan(err) ? lastError : err)));
        Serial.print(",");
        Serial.println(BASE_PWM + (int)(KP * (isnan(err) ? lastError : err)));
        lastPrintMs = now;
    }
}
