#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

#define OLED_RESET 4                                  //it's a thing
Adafruit_SSD1306 display(OLED_RESET);                 //create instance of the 

//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;
int timer[10];
int slope[10];
unsigned int totalTimer;
unsigned int period;
byte index = 0;
float frequency;
int maxSlope = 0;
int newSlope;

//variables for decided whether you have a match
byte noMatch = 0;
byte slopeTol = 3;
int timerTol = 10;

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 40;

// Rotary Encoder
int encoderPinA = 2;
int encoderPinB = 3;
int encoderPos = 0;
int encoderPinALast = LOW;
int n = LOW;
int encoderValue = 0;

// LED Pins
int greenLEDPin = 5;
int redLEDPin1 = 6;
int redLEDPin2 = 7;

// String and Frequency Arrays
const char *strings[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
const float targetFrequencies[] = {82.41, 110.0, 146.8, 196.0, 246.9, 329.6};

void setup(){
  
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);          //begin the OLED @ hex addy )x3C
  display.display();                                  //show the buffer
  delay(2000);                                        //bask in the glory of LadyAda
  display.clearDisplay();                             //enough basking

  Serial.begin(9600);
  
  pinMode(13,OUTPUT);//led indicator pin
  pinMode(12,OUTPUT);//output pin
  
  cli();//diable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  // Rotary Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // LED setup
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin1, OUTPUT);
  pinMode(redLEDPin2, OUTPUT);

  cli();
  // ... (same interrupt setup as before)
  sei();

}

ISR(ADC_vect) {//when new ADC value ready
  
  PORTB &= B11101111;//set pin 12 low
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 127 && newData >=127){//if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope-maxSlope)<slopeTol){//if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){//new max slope just reset
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0]-timer[index])<timerTol && abs(slope[0]-newSlope)<slopeTol){//if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i=0;i<index;i++){
          totalTimer+=timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
      }
      else{//crossing midpoint but not match
        index++;//increment index
        if (index > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope){//if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else{//slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch>9){
        reset();
      }
    }
  }
    
  if (newData == 0 || newData == 1023){//if clipping
    clipping = 1;//currently clipping
    Serial.println("clipping");
  }
  
  time++;//increment timer at rate of 38.5kHz
  
  ampTimer++;//increment amplitude timer
  if (abs(127-ADCH)>maxAmp){
    maxAmp = abs(127-ADCH);
  }
  if (ampTimer==1000){
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
  
}


void reset(){//clean out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}


void checkClipping(){//manage clipping indication
  if (clipping){//if currently clipping
    clipping = 0;
  }
}

void updateDisplay(const char *note, float goalFrequency, float medianFrequency) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Tuning Note ");
  display.println(note);
  display.print("Goal: ");
  display.println(goalFrequency);
  display.print("Frequency: ");
  display.println(medianFrequency);
  display.display();
}

void updateLEDs(float goalFrequency, float medianFrequency) {
  if (abs(medianFrequency - goalFrequency) <= 3.0) {
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin1, LOW);
    digitalWrite(redLEDPin2, LOW);
  } else if (medianFrequency > goalFrequency) {
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin1, HIGH);
    digitalWrite(redLEDPin2, LOW);
  } else {
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin1, LOW);
    digitalWrite(redLEDPin2, HIGH);
  }
}

float frequencyArray[5]; // Array to store frequency values
byte freqIndex = 0; // Index to keep track of the current position in the array
unsigned long lastInputTime = 0; // Time of the last input

void loop(){
  checkClipping();
  encoderRead();

  if (checkMaxAmp > ampThreshold) {
    frequency = 38462 / float(period); // Calculate frequency timer rate/period

    // Store the frequency value in the array
    frequencyArray[freqIndex] = frequency;

    // Update the time of the last input
    lastInputTime = millis();

    // Increment the array index
    freqIndex++;

    if (freqIndex >= 5) {
      // Calculate the median of the stored frequency values
      float medianFrequency = calculateMedian(frequencyArray, 5);

      // Reset the array and index
      freqIndex = 0;
      for (int i = 0; i < 5; i++) {
        frequencyArray[i] = 0;
      }

      // Get the current note and goal frequency
      int currentNoteIndex = (encoderPos % 6 + 6) % 6; // 6 notes in total
      const char *currentNote = strings[currentNoteIndex];
      float goalFrequency = targetFrequencies[currentNoteIndex];

      updateDisplay(currentNote, goalFrequency, medianFrequency);
      updateLEDs(goalFrequency, medianFrequency);

    }
  }

  // Check for inactivity and reset the array if needed
  if (millis() - lastInputTime >= 500) {
    freqIndex = 0;
    for (int i = 0; i < 5; i++) {
      frequencyArray[i] = 0;
    }
  }

  delay(100);
}

// Function to calculate the median of an array of floats
float calculateClosestFrequency(float arr[], int size, float goalFrequency) {
  // Find the frequency closest to the goal frequency
  float closestFrequency = arr[0];
  float minDifference = abs(arr[0] - goalFrequency);

  for (int i = 1; i < size; i++) {
    float difference = abs(arr[i] - goalFrequency);
    if (difference < minDifference) {
      minDifference = difference;
      closestFrequency = arr[i];
    }
  }

  return closestFrequency;
}

  // Calculate the median
  if (size % 2 == 0) {
    // If the array has an even number of elements, average the middle two
    int mid = size / 2;
    return (arr[mid - 1] + arr[mid]) / 2.0;
  } else {
    // If the array has an odd number of elements, return the middle value
    int mid = size / 2;
    return arr[mid];
  }
}

void encoderRead() {
  n = digitalRead(encoderPinA);
  if ((encoderPinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoderPinB) == LOW) {
      encoderPos--;
    } else {
      encoderPos++;
    }

    // Get the current note and goal frequency based on the rotary encoder position
    int currentNoteIndex = (encoderPos % 6 + 6) % 6;  // Ensure encoderPos is positive and within the range [0, 5]
    const char *currentNote = strings[currentNoteIndex];
    float goalFrequency = targetFrequencies[currentNoteIndex];

    // Update the display and LEDs immediately
    float medianFrequency = calculateMedian(frequencyArray, 5);
    updateDisplay(currentNote, goalFrequency, medianFrequency);
    updateLEDs(goalFrequency, medianFrequency);
  }
  encoderPinALast = n;
}
