#define CPR 3172 // Counts Per Revolution at output shaft
static bool fineTuning = false;
const int encoderPinA = 21; // Encoder A pin (interrupt pin)
const int encoderPinB = 22; // Encoder B pin (interrupt pin)
const int PWM = 23;        // PWM output pin
const int motorDirPin = 6; // Motor direction control pin

volatile long encoderCounts = 0;
int lastEncoded = 0;
float fvbar = 0;
float errorIntegral = 0;
const int numReadings =3 ;
float readings[numReadings]; // Array to store the last 4 RPM readings
int readIndex = 0; // Current index in the readings array
const float maxDifference = 2.0; // Maximum allowed difference


unsigned long previousMillis = 0;
const int interval = 10; // Interval for control update (in milliseconds)
const float threshold = 8; // Threshold for switching to finer control
float targetRPM = 150;

void setup() {
    Serial.begin(115200); // Initialize serial communication
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(PWM, OUTPUT);
    pinMode(motorDirPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
    previousMillis = millis();
}

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long motorStartTime = 0;
    static unsigned long fineTuningStartTime = 0;
    static bool motorStarted = false;
    static float pwr = 25; // Initialize with a starting PWM value
    static int v_bar = 25;
    float t = 0;

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        float currentRPM = (encoderCounts * 6000.0) / CPR;
        encoderCounts = 0;
        // Serial.printf("Cuurent RPM: %f \n", currentRPM);
        Serial.print(millis()/1000.0); // Print current time in milliseconds
        Serial.print(",");
        Serial.println(currentRPM); // Print RPM

        readings[readIndex] = currentRPM;
        readIndex = (readIndex + 1) % numReadings;

        // Motor control logic
        if (!motorStarted && currentRPM > 0) {
            motorStartTime = currentMillis;
            motorStarted = true;
            fineTuning = false;
            pwr = 25; // Start motor with initial PWM value
        } else if (motorStarted && currentRPM == 0) {
            motorStarted = false;
            fineTuning = false;
        }

        if (motorStarted) {
            if (!fineTuning) {
                t = (currentMillis - motorStartTime) / 1000.0; // Time since motor start
                if (abs(targetRPM - currentRPM) > threshold) {
                    pwr = calculatePWM(targetRPM - currentRPM, t, v_bar); // Non-linear control logic
                    v_bar = min(v_bar + 1, 255); // Adjust v_bar
                } else {
                    fineTuning = true;
                    fineTuningStartTime = currentMillis; // Reset time for fine-tuning
                }
            } else {
                
                t = (currentMillis - fineTuningStartTime) / 1000.0; // Time since fine-tuning start
                int v2_bar = v_bar-3;
                calculatePWM(targetRPM - currentRPM, t, v2_bar); // Non-linear control logic
                 // Fine-tuning logic
                 if(abs(targetRPM - currentRPM)<3){
                  if(currentRPM>targetRPM){
                    v2_bar=v2_bar-2;
                    pwr=0.99*v2_bar;
                  }
                  if(currentRPM<targetRPM){
                    v2_bar=v2_bar+2;
                    pwr=0.99*v2_bar;}
            }
               }   }

        setMotor(targetRPM - currentRPM >= 0 ? 1 : 0, constrain((int)pwr, 0, 255), PWM, motorDirPin);
        
        
    }
}



float calculateDynamicC(float error) {
    float baseCValue = 0.01;
    float minCValue = 0.001;
    float maxCValue = 5;
    float newC = baseCValue / (1 + abs(error));
    return constrain(newC, minCValue, maxCValue);
}

float calculateSi(float targetRPM, float t) {
    float y =20 ;
    float psi_0 = targetRPM;
    float psi_i = targetRPM * 0.01;
    return (psi_0 - psi_i) * exp(-y * (t)) + psi_i;
}


float calculateScaledX(float error, float si, float c) {
    float errorIntegral = 0;
    errorIntegral += error;
    float k =10;
    k = max(k-1,2.0);
    return (PI / (2 * c)) * abs(tanh(((PI *error) / (2 * si))+(errorIntegral/k)));
}

float calculatePWM(float e, float t, int v_bar) {
    if(!fineTuning){
    float si = calculateSi(targetRPM, t);
    float c= 0.001;
    float x = calculateScaledX(e, si, c);
    float pwr_inp = ((2.0 * v_bar) / PI) * atan(x);
    return ((2.0 * v_bar) / PI) * atan(x);}
    
}

float fineTuneControl(float e, float t, int &v_bar) {
    // Exit the function and maintain the current PWM value if error is less than 1
    float si = targetRPM*0.01;
    float c= 0.001;
    float x = calculateScaledX(e, si, c);
    float pwr_inp = ((2.0 * v_bar) / PI) * atan(x);
    return ((2.0 * v_bar) / 3.14) * atan(x);
    
}

void setMotor(int dir, int pwmVal, int pwmPin, int dirPin) {
    analogWrite(pwmPin, pwmVal);
    digitalWrite(dirPin, dir);
    
}

void updateEncoder() {
    int MSB = digitalRead(encoderPinA);
    int LSB = digitalRead(encoderPinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCounts++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCounts--;
    lastEncoded = encoded;}