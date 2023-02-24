//PID Control Template

// *********************************************************************************************************************** //
// Variables                                                                                                               //
// *********************************************************************************************************************** //

//pin definition
#define ENA 9
#define IN1 21
#define IN2 20
#define SENSOR_PIN A0

// ******** <TODO> **********************
// ******** define interval between recomputing error and adjusting feedback (in milliseconds) ********************** 
const int INTERVAL = 20; 
unsigned long previousTime = 0;

int motorSpeed = 0; // speed of the motor, values between 0 and 255
int target = 512; // position (as read by potentiometer) to move the motor to, default value 512
int error = 0;
float INTEGRAL = 0;
float epsilon = 3;

// ******** <TODO> **********************
// ******** define the different gains **********************
float kp = 0.0; // proportional gain
float ki = 0.0; // integral gain
float kd = 0.0; // derivative gain

int pos = 0; // current position for plotting


//serial communication variables
float PID_values[4];
byte i = 0;
char record[100];
char recvchar;
byte indx = 0;

// setup code, setting pin modes and initialising the serial connection
void setup() 
{
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);    
    pinMode(SENSOR_PIN, INPUT);   
}

void loop() 
{
        //  ******** <TODO> **********************
        //  ******** implement your code  here **********************

        // set timer interval 
        unsigned long currentMillis = millis();
        if(currentMillis - previousTime > INTERVAL) {
          previousTime = currentMillis; 
          update();  
        }
}


void update() {

        readInput();    
        pos = analogRead(SENSOR_PIN);
        int err = target - pos;
        float p = err;
        
        float integral = 0;
        
        if (abs(err) > epsilon){
          integrate(err, INTERVAL);
          integral = INTEGRAL;
        }
        
        float d = derive(error, err, INTERVAL);
        float u = - (kp * p) - (ki * integral) - (kd * d);
     
        
        error = err;
        
        if (u > 0) 
          setMovement(0, u);
        else
          setMovement(1, u);
        
        
        //print actual motor position and target value to serial-monitor/plotter
        Serial.print(" P: ");
        Serial.print(pos);
        Serial.print(" T: ");  
        Serial.println(target);
 
}

// method to set direction and speed of the motor
void setMovement(int dir, int speed1) 
{
      //  ******** <TODO> **********************
      //  ******** implement your code  here **********************
      
      motorSpeed = speed1; 
      analogWrite(ENA, abs(motorSpeed));
      if (dir == 0) {
        //tell the motor to go forward (may depend by your wiring)
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);  
      } else {
        //tell the motor to go backward (may depend by your wiring)
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      }
                    
}


void integrate (float y, float dt) {
  float di = y * 0.5 * dt;
  incrementInBounds(di);
}



const int MIN = 1;
const int MAX = 2000;


void incrementInBounds(float di) {
    float max_output = 255 / ki;

    if (INTEGRAL > 0) {
       if (INTEGRAL + di < MIN) INTEGRAL = MIN;   
       if (INTEGRAL + di > MAX) INTEGRAL = MAX;   
       if (INTEGRAL + di > max_output) INTEGRAL = max_output;
   }
   else {
       if (INTEGRAL + di > -MIN) INTEGRAL = -MIN;   
       if (INTEGRAL + di < -MAX) INTEGRAL = -MAX;  
       if (INTEGRAL + di < -max_output) INTEGRAL = -max_output;

   }
}


float derive(float a, float b, float dt) {
  return (b - a) / dt;
}


void shutMotorsDown() {
  // now turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// *********************************************************************************************************************** //
// method for receiving commands over the serial port
void readInput() 
{
      if (Serial.available())
    {
        recvchar = Serial.read();
        if (recvchar != '\n')
        { 
            record[indx++] = recvchar;
        }
        else if (recvchar == '\n')
        {
          record[indx] = '\0';
          indx = 0;
          
          convertData(record);
          if(i==4){
            target = PID_values[0];
            kp = PID_values[1];
            ki = PID_values[2];
            kd = PID_values[3];
            //printData(PID_values);
            resetIntegral();
          }
          else {
            Serial.println("Enter correct number of values separated by commas!!");            
          }
        }
    }
}

void resetIntegral() {
  INTEGRAL = 0;
}

// *********************************************************************************************************************** //
//method for reading/interpreting serial input 
void convertData(char record[])
{
    i = 0;
    char *index = strtok(record, ",");
    while(index != NULL)
    {
       PID_values[i++] = atof(index); 
        index = strtok(NULL, ",");
    }
}
// *********************************************************************************************************************** //
//method for printing values entered via the serial monitor
void printData(float data[])
{
    for (byte j = 0 ; j < 4 ; j++)
    {
      Serial.print(data[j]);
      Serial.print('\t');
    }
    Serial.println(); 
}
