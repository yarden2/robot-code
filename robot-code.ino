#include <avr/interrupt.h>

// =====================================================
// PINS
// =====================================================

const int PWMA = 5;
const int PWMB = 11;

const double rad = 0.01745329;

const float L = 14.25;
volatile unsigned long lastM1EdgeMicros = 0;
volatile unsigned long lastM2EdgeMicros = 0;
volatile unsigned long lastM1Period = 0;

volatile long rejectedEdges1 = 0;
volatile long rejectedEdges2 = 0;
volatile bool m1NewPeriod = false;
const int filter = 5000;
const unsigned long SPEED_WINDOW_MS = 200;
const int AIN1 = 8;
const int AIN2 = 12;
const float maxSpeed = 50;
char rxBuffer[32];
byte rxIndex = 0;
int lastM1Ticks = 0;
int lastM2Ticks = 0;
const int BIN1 = 2;
const int BIN2 = 7;

const int STBY = 4;

const int M1_TICK_PIN = 3;
const int M2_TICK_PIN = 6;



// =====================================================
// ENCODER
// =====================================================

const float TICKS_PER_REV = 40.0;
const float WHEEL_DIAMETER = 6.5;

const float DISTANCE_PER_TICK =
    (3.14159265 * WHEEL_DIAMETER)
    / TICKS_PER_REV;

volatile long m1Ticks = 0;
volatile long m2Ticks = 0;

volatile bool prevM1State = HIGH;
volatile bool prevM2State = HIGH;

// =====================================================
// SPEED
// =====================================================

float m1Speed = 0;
float m2Speed = 0;

long prevM1Ticks = 0;
long prevM2Ticks = 0;

unsigned long lastSpeedUpdate = 0;

// =====================================================
// PID
// =====================================================

float wantedSpeedR = 0;
float wantedSpeedL= 0;
double m1Power = 0;
double m2Power = 0;

double m1Integral = 0;
double m2Integral = 0;

double m1PrevError = 0;
double m2PrevError = 0;

const double Kp = 0.15;
const double Ki = 0;
const double Kd = 0.05;

unsigned long lastControlTime = 0;

const float SPEED_RESOLUTION =
    DISTANCE_PER_TICK * (1000.0 / SPEED_WINDOW_MS);
// =====================================================
// SETUP
// =====================================================

void setup()
{
    byte reason = MCUSR;
    Serial.begin(9600);

    pinMode(STBY, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(M1_TICK_PIN, INPUT_PULLUP);
    pinMode(M2_TICK_PIN, INPUT_PULLUP);

    digitalWrite(STBY, HIGH);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    prevM1State = digitalRead(M1_TICK_PIN);
    prevM2State = digitalRead(M2_TICK_PIN);

    PCICR |= (1 << PCIE2);

    PCMSK2 |= (1 << PCINT19);
    PCMSK2 |= (1 << PCINT22);

    sei();

    lastControlTime = millis();
    lastSpeedUpdate = millis();
     
    
   

Serial.print("RESET CAUSE=");
Serial.println(reason);
    Serial.println("READY");

   // DriveAngle(20,30);
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    wantedSpeedR = round(wantedSpeedR / SPEED_RESOLUTION) * SPEED_RESOLUTION;
    wantedSpeedL = round(wantedSpeedL / SPEED_RESOLUTION) * SPEED_RESOLUTION;

    if(wantedSpeedR ==0 && wantedSpeedL == 0){
       // Stop();
    }
    GetSpeed();

    if (millis() - lastControlTime >= 20)
    {
        UpdateMotors();
    }

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= SPEED_WINDOW_MS && 0)
    {
        lastPrint = millis();

        Serial.print("m1Ticks:");
        Serial.print(m1Ticks-lastM1Ticks);

        Serial.print(",");

        Serial.print("m2Ticks:");
        Serial.print(m2Ticks-lastM2Ticks);

        lastM1Ticks = m1Ticks;
        lastM2Ticks = m2Ticks;

        Serial.print(",");

        Serial.print("speed1:");
        Serial.print(m1Speed);

        Serial.print(",");

         Serial.print("speed2:");
        Serial.print(m2Speed);

        Serial.print(",");

        Serial.print("pwm1:");
        Serial.print(m1Power);

        Serial.print(",");

        Serial.print("pwm2:");
        Serial.print(m2Power);

        Serial.print(", rej1:");
Serial.print(rejectedEdges1);

Serial.print(", rej2:");
Serial.print(rejectedEdges2);


        Serial.print(",");

         Serial.print("targetL:");
        Serial.print(wantedSpeedL);
        Serial.print(",");

        Serial.print("targetR:");
        Serial.println(wantedSpeedR);

        rejectedEdges1 = 0;
        rejectedEdges2 = 0;
        

        
    }

  

while (Serial.available())
{
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
        if (rxIndex > 0)
        {
            rxBuffer[rxIndex] = '\0';

            // Check for "(speed,angle)" format
            float speed, angle;
           // Serial.println(rxBuffer);
            char* comma = strchr(rxBuffer, ',');

if (rxBuffer[0] == '(' && comma)
{
    *comma = '\0';

    float speed = atof(rxBuffer + 1);
    float angle = atof(comma + 1);

    DriveAngle(speed*maxSpeed, angle);

   // Serial.print("DriveAngle(");
   // Serial.print(speed);
    //Serial.print(",");
    //Serial.print(angle);
    //Serial.println(")");
    
}
            else
            {
                // Treat input as a single speed value
                float speed = (atof(rxBuffer)/10.0)*60;

                wantedSpeedR = speed;
                wantedSpeedL = speed;

                //Serial.print("Target=");
                //Serial.println(speed);
            }

            rxIndex = 0;
        }
    }
    else if (rxIndex < sizeof(rxBuffer) - 1)
{
    rxBuffer[rxIndex++] = c;
}
else
{
    rxIndex = 0;
}
}



static unsigned long lastPrintPeriod = 0;

if (m1NewPeriod)
{
    noInterrupts();

    unsigned long period = lastM1Period;

    m1NewPeriod = false;

    interrupts();

    //Serial.print("edge_us=");
    //Serial.println(period);
}
}

// =====================================================
// SPEED
// =====================================================

void GetSpeed()
{
    if (millis() - lastSpeedUpdate < SPEED_WINDOW_MS)
        return;

    long currentM1;
    long currentM2;

    noInterrupts();

    currentM1 = m1Ticks;
    currentM2 = m2Ticks;

    interrupts();

    long deltaM1 =
        currentM1 - prevM1Ticks;

    long deltaM2 =
        currentM2 - prevM2Ticks;

    prevM1Ticks = currentM1;
    prevM2Ticks = currentM2;

    const float scale =
        1000.0 / SPEED_WINDOW_MS;

    m1Speed =
        deltaM1 *
        DISTANCE_PER_TICK *
        scale;

    m2Speed =
        deltaM2 *
        DISTANCE_PER_TICK *
        scale;

    lastSpeedUpdate =
        millis();
}

// =====================================================
// PID
// =====================================================

void UpdateMotors()
{
    unsigned long now =
        millis();

    double dt =
        (now - lastControlTime)
        / 1000.0;

    //SetMotorLDirection(wantedSpeedL>0);
    //SetMotorRDirection(wantedSpeedR>0);

   // SetMotorLDirection(1);
    //SetMotorRDirection(1);
    double wantedSpeedL2 = abs(wantedSpeedL);
    double wantedSpeedR2 = abs(wantedSpeedR);
    if (dt <= 0)
        return;

    lastControlTime =
        now;

    double error1 =
        wantedSpeedL2 - m1Speed;

    double error2 =
        wantedSpeedR2 - m2Speed;

    m1Integral += error1 * dt;
    m2Integral += error2 * dt;

    double derivative1 =
        (error1 - m1PrevError) / dt;

    double derivative2 =
        (error2 - m2PrevError) / dt;

    m1PrevError = error1;
    m2PrevError = error2;

    double output1 =
        Kp * error1 +
        Ki * m1Integral +
        Kd * derivative1;

    double output2 =
        Kp * error2 +
        Ki * m2Integral +
        Kd * derivative2;

    // Your preferred architecture:
    m1Power += output1;
    m2Power += output2;

    m1Power =
        constrain(m1Power, 0, 255);

    m2Power =
        constrain(m2Power, 0, 255);

    analogWrite(
        PWMA,
        m1Power);

    analogWrite(
        PWMB,
        m2Power);
}

// =====================================================
// STOP
// =====================================================

void Stop()
{
    
    m1Power = 0;
    m2Power = 0;

    m1Integral = 0;
    m2Integral = 0;

    m1PrevError = 0;
    m2PrevError = 0;

    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}
void SetMotorLDirection(bool dir){
     digitalWrite(AIN1, dir);
    digitalWrite(AIN2, !dir);

   
}
void SetMotorRDirection(bool dir){
     digitalWrite(BIN1, dir);
    digitalWrite(BIN2, !dir);  
}


// =====================================================
// ENCODER ISR
// =====================================================
void DriveAngle(double speed,double angle){
    
    wantedSpeedR=speed+0.5*angle*rad*L;
    wantedSpeedL = speed-0.5*angle*rad*L;
}


ISR(PCINT2_vect)
{
    bool m1State =
        (PIND & (1 << PD3));

    bool m2State =
        (PIND & (1 << PD6));

    
  if (m1State != prevM1State)
{
    unsigned long now = micros();

    if ((now - lastM1EdgeMicros) > filter)
    {
        m1Ticks++;
        lastM1EdgeMicros = now;
    }
    else{
        rejectedEdges1++;
    }

    prevM1State = m1State;
}

   if (m2State != prevM2State)
{
    unsigned long now = micros();

    if ((now - lastM2EdgeMicros) > filter)
    {
        m2Ticks++;
        lastM2EdgeMicros = now;
    }
    else
    {
        rejectedEdges2++;
    }

    prevM2State = m2State;
}
else{
        rejectedEdges2++;
    }
}