//VARIABLES
float Forces[7];
float hand1;
float hand2;

int hCTR;
int consec_weakgrip = 0;
int consec_normgrip = 0;

float averageGrip = 0.42;
float thresh = 0.01;
float FSRreading;

void setup() {
  Serial.begin(115200);

}

void loop() {
  for (int i = 0; i < 7; i++) {
    FSRreading = convertNtP(analogRead(i)); //FSR reading into Pound Force
    Forces[i] = FSRreading;
    //Serial.println(Forces[i]);
  }

  for (int i = 0; i < 7; i++) {
    if (Forces[i] > thresh) {
      if (hCTR == 1) {
        hand2 = Forces[i];
        hCTR = 0;
      } else {
        hand1 = Forces[i];
        hCTR = 1;
      }
    }
  }

  if((hand1 < averageGrip) && (hand2 < averageGrip)) {
    consec_weakgrip++;
    consec_normgrip = 0;
    Serial.println(consec_weakgrip);
  } else {
    consec_normgrip++;
    consec_weakgrip = 0;
    Serial.println(consec_weakgrip);
  }

  if(consec_weakgrip >= 500) {
    //Code Bi-directional Serial Communciation here
    if(Serial.available() > 0) {
      String rpiSignal = Serial.readStringUntil('\n');
      if (rpiSignal = "Drowsy") { //drowsy
        Serial.println("Drowsy Grip");
      } else {
        Serial.println("Loose Grip");
      }
    }
  }
}

float convertNtP( float reading) {
  float N = exp((reading - 555.53)/156.34); //Ohms to Newtons
  float Lb = N * 0.224809; //Newtons to Pounds
  return Lb;
}
