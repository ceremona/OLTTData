
const int analogInPin = A9; // Analog output pin that the LED is attached to
float sensorValue = 0;        // value read from the pot
float Transmitance = 0;        // value output to the PWM (analog out)
const float maxValue = 818.4;  // max 10 bit value for 4V output 1023*(4/5)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(A9,INPUT);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  Transmitance = sensorValue/maxValue;
  // change the analog out value:
//  analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
  Serial.printf("sensor = %4.2f\n", sensorValue);
  Serial.printf("Transmitance = %.3f\n",Transmitance);
//  Serial.print("\t output = ");
//  Serial.println(outputValue);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(200);
}
