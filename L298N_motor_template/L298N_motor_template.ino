//pin definition
#define ENA 9
#define IN1 20
#define IN2 21

void setup() 
{

  //used for display information
  Serial.begin(9600);

  // set motor control pins to output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() 
{

  // set speed to 200 out of possible range 0~255
  analogWrite(ENA, 200);
 
  //tell the motor to go forward (may depend by your wiring)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  delay(3000);

  // now break motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);

  delay(3000);
  
   //tell the motor to go backward (may depend by your wiring)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  delay(3000);
  
  // now turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  delay(3000);

}
