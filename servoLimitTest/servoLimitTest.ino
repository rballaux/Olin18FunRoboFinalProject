#include <Servo.h>

Servo motor;
int command=90;
void setup() {
  Serial.begin(9600);
  motor.attach(5);
  motor.write(90);

}

void loop() {
  while(Serial.available()){
    command = Serial.readString().toInt();
    Serial.print("The command you sent was: ");
    Serial.println(command);
    }
  
delay(1000);
motor.write(command);
}
