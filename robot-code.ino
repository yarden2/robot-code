const int PWMA = 5;
const int PWMB = 11;


const int AIN1 = 8;
const int AIN2 = 12;

const int BIN1 = 2;
const int BIN2 = 7;

const int STBY = 4;






void setup(){
Serial.begin(9600);
Serial.println("boot");
pinMode(STBY, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    digitalWrite(STBY, HIGH);

}

void loop(){
    analogWrite(PWMA,255);
    analogWrite(PWMB,255);
    
}