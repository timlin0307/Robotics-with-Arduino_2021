int S0 = 4;
int S1 = 5;
int S2 = 6;
int S3 = 7;
int OUT = 8;
int LED = 3;
int white = 9;
int transmit = 2;
char color = 'n';

// Save the value read from the photodiode
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(white, OUTPUT);
  pinMode(transmit, OUTPUT);
  // set the frequency scaling (頻率尺度) = 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(white, LOW);
  digitalWrite(transmit, LOW);
}

void loop() {
  // set the photodiode to read the filted value of red
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // read the output frequency
  redFrequency = pulseIn(OUT, LOW);
  // print the value of red
  Serial.print("R = ");
  Serial.println(redFrequency);
  delay(100);
  // set the photodiode to read the filted value of green
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // read the output frequency
  greenFrequency = pulseIn(OUT, LOW);
  // print the value of green
  Serial.print("G = ");
  Serial.println(greenFrequency);
  delay(100);
  // set the photodiode to read the filted value of blue
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // read the output frequency
  blueFrequency = pulseIn(OUT, LOW);
  // print the value of blue
  Serial.print("B = ");
  Serial.println(blueFrequency);
  delay(100);
  if(redFrequency <= 40 && greenFrequency <= 40 && blueFrequency <= 40) {
    delay(10);
    if(redFrequency <= 40 && greenFrequency <= 40 && blueFrequency <= 40) {
      color = 'w';
      digitalWrite(white, HIGH);
      digitalWrite(transmit, LOW);
    }
  } else if(redFrequency >= 90 && greenFrequency >= 90 && blueFrequency >= 90) {
    delay(10);
    if(redFrequency >= 900 && greenFrequency >= 90 && blueFrequency >= 90) {
      color = 'b';
    digitalWrite(white, LOW);
    digitalWrite(transmit, HIGH);
    }
  }
  Serial.println(color);
  Serial.println("-----------------");
  delay(1000);
}
