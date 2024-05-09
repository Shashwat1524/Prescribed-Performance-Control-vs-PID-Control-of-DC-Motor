#include <Encoder.h>

Encoder myEnc(21, 22);
volatile int32_t lastEncoderValue = 0;
int32_t CPR = 3172;
const int PWM_PIN = 23;
IntervalTimer speedTimer;
float rpm = 0;
float target_rpm = 100;
float kp = 1.8;  // Proportional gain
float ki = 0.18; // Integral gain
float kd = 0.01; // Derivative gain
float integral = 0;
float prev_error = 0;
bool case1=true;
bool case2=true;
unsigned long targetChangeTime = 0;
const unsigned long delayTime = 9000; 

void setup() {
  Serial.begin(115200);
  speedTimer.begin(calculateRPM, 100000); // 0.1s
}

void loop() {

  float error = target_rpm - rpm;

  float pid_output = kp * error + ki * integral + kd * (error - prev_error);
  prev_error = error;
  integral += error;

  float pwm_value = constrain(pid_output, 0, 255); 

  analogWrite(PWM_PIN, pwm_value);

  // Print RPM and time
  Serial.print(millis()/1000.0); // Print current time in milliseconds
  Serial.print(",");
  Serial.println(rpm); // Print RPM
  if(rpm>100 && case1==true && millis() - targetChangeTime >= delayTime)
        {
          target_rpm=150;
          case1=false;
          targetChangeTime = millis();

        }
        if(rpm>150 && case2==true && millis() - targetChangeTime >= delayTime)
        {
          target_rpm=200;
          case2=false;
          targetChangeTime = millis();
        }

  delay(100);
}

void calculateRPM() {
  int32_t encoderValue = myEnc.read(); // Reads value from the encoder
  int32_t deltaEncoder = encoderValue - lastEncoderValue;
  lastEncoderValue = encoderValue;
  rpm = (float(deltaEncoder) / CPR) * (60.0 / 0.1); // Calculate RPM
}
