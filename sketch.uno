/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the p
  if(sensorValue == HIGH){otentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */
//#define DEBUG


#define STATE_ERROR_CABLE    -2
#define STATE_ERROR          -1
#define STATE_WAIT_WATER      0
#define STATE_PUMPED          1

#define LEVEL_UNKNOWN        -1   // unknown level
#define LEVEL_NONE            0   // no water
#define LEVEL_MIN             1   // water between min and max level
#define LEVEL_MAX             2   // water above max level(wrong min/max cable)


const int supPin = A0;            // Suplay power
const int minPin = A3;            // Min level water
const int maxPin = A4;            // Max level water
const int lampPin = 6;            // select the pin for the LED
const int pumpPin = 7;            // select the pin for the LED

const int countChecks           = 3;        // Count checks water line
const int delayBetweenChecks    = 100;      // Delay between checks water line
const int minLevelSignal        = 100;       // Min level singna on pin
const int timeWaitWaterCheck    = 3000;     // Time wait between checks water line
const unsigned long maxWaitPump = 3600000;    // Max time work pump


int state = 0;

void setup() {
    #ifdef DEBUG
    Serial.begin(9600);
    #endif

    pinMode(lampPin, OUTPUT);
    pinMode(pumpPin, OUTPUT);
    
    pinMode(minPin, INPUT);
    pinMode(maxPin, INPUT);
  
    // Set start status
    state = STATE_WAIT_WATER;
}


void loop() {
    switch(state){
        case STATE_WAIT_WATER:
            state = do_wait_water();
            delay(timeWaitWaterCheck);
            break;
        case STATE_PUMPED:
            state = do_pumped();
            break;
        case STATE_ERROR:
            do_error();
            break;
        case STATE_ERROR_CABLE:
            do_error_cable();
            break;
    }
    #ifdef DEBUG
    printState(state);
    #endif
}

int do_error_cable(){
    for(int i = 0; i < 3; i++){
        digitalWrite(lampPin, HIGH);
        delay(500);
        digitalWrite(lampPin, LOW);
        delay(500);
    }
}

int do_error(){
    for(int i = 0; i < 3; i++){
        digitalWrite(lampPin, HIGH);
        delay(500);
        digitalWrite(lampPin, LOW);
        delay(500);
    }
    for(int i = 0; i < 3; i++){
        digitalWrite(lampPin, HIGH);
        delay(1500);
        digitalWrite(lampPin, LOW);
        delay(1500);
    }
}

int do_wait_water(){
    int level = getLevel();
    switch(level){
        case LEVEL_UNKNOWN:
            return STATE_ERROR_CABLE;
        case LEVEL_NONE:
            return STATE_WAIT_WATER;
        case LEVEL_MIN:
            return STATE_WAIT_WATER;
        case LEVEL_MAX:
            return STATE_PUMPED;
    }
    return STATE_ERROR;
}

int do_pumped(){
    pump_on();
    unsigned long currentWait = 0;
    while(currentWait < maxWaitPump){
        #ifdef DEBUG
        printState(state);
        #endif
        // Check water level
        if(getLevel() == LEVEL_NONE){
            pump_off();
            return STATE_WAIT_WATER;
        }
        // Wait
        delay(timeWaitWaterCheck);
        // Counter time
        currentWait = currentWait + timeWaitWaterCheck + (delayBetweenChecks * 3);
    }
    pump_off();
    return STATE_ERROR;
}

void pump_on(){
    digitalWrite(lampPin, HIGH);
    digitalWrite(pumpPin, HIGH);
}

void pump_off(){
    digitalWrite(lampPin, LOW);
    digitalWrite(pumpPin, LOW);
}

// Get water level
// LEVEL_NONE    - no water
// LEVEL_MIN     - water between min and max level
// LEVEL_MAX     - water above max level
// LEVEL_UNKNOWN - unknown level
int getLevel(){
    int minVal = 0;
    int maxVal = 0;
    int minTmp = 0;
    int maxTmp = 0;
    for(int i = 0; i < countChecks; i++){
        analogWrite(supPin, 255);
        minTmp = analogRead(minPin);
        maxTmp = analogRead(maxPin);
        minVal = minVal + minTmp;
        maxVal = maxVal + maxTmp;
        analogWrite(supPin, 0);
        #ifdef DEBUG
        printVals(minTmp, maxTmp);
        #endif
        delay(delayBetweenChecks);
    }
    minVal = minVal / countChecks;
    maxVal = maxVal / countChecks;

    if(minVal < minLevelSignal && maxVal < minLevelSignal){
        return LEVEL_NONE;
    } else if(minVal >= minLevelSignal && maxVal < minLevelSignal){
        return LEVEL_MIN;
    } else if(minVal >= minLevelSignal && maxVal >= minLevelSignal){
        return LEVEL_MAX;
    }
    return LEVEL_UNKNOWN;
}
 
#ifdef DEBUG
void printVals(int min, int max){
  Serial.print("Min: ");
  Serial.print(min);
  Serial.print(" Max: ");
  Serial.println(max);
}

// Debug: Print current state to COM port
void printState(int state){
    switch(state){
        case STATE_WAIT_WATER:
            Serial.println("State STATE_WAIT_WATER");
            break;
        case STATE_PUMPED:
            Serial.println("State STATE_PUMPED");
            break;
       case STATE_ERROR:
            Serial.println("State STATE_ERROR");
            break;
        case STATE_ERROR_CABLE:
            Serial.println("State STATE_ERROR_CABLE");
            break;
    }
}
#endif
