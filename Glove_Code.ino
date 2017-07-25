const int thumbPin = A0;
const int indexPin = A1;
const int middlePin = A2;
const int ringPin = A3;
const int pinkiePin = 10;

const float VCC = 4.98; //Actual Value of 5v
const float R_DIV = 47500.0; //Actual Value of Resistor
const float STRAIGHT_RESISTANCE = 37300.0; //Change this until the angle pretty much correlates to the degree your bending the sensor
const float BEND_RESISTANCE = 90000.0; //Ditto ^^^

void setup() {
  pinMode(thumbPin, INPUT);
  pinMode(indexPin, INPUT);
  pinMode(middlePin, INPUT);
  pinMode(ringPin, INPUT);
  pinMode(pinkiePin, INPUT);
}

void loop() {
  
}

void readFlex (byte finger) {
  
  int flexADC = analogRead(finger);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  
}

