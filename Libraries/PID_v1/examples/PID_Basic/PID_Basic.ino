
byte pinMoteurPwm=10;//9
//byte pinSensMoteur=4;
byte pinSensMoteur=11;//12
int pinAnalogique =A1;

//Define Variables we'll be connecting to
double consigne, vitesse,lastAngle,Angle,erreur,output,prevOutput,somme;
int sampleTime=100;

//Specify the links and initial tuning parameters
double Kp=0.05, Ki=0, Kd=0;
unsigned long tAbs;
unsigned long tempsAbs=0;
//PARAMETRES POUR LE TETE SANS AREF
double coeffDir =0.558;
double ordonneeOrigine=-133.51;

void setup()
{
  //initialize the variables we're linked to
  pinMode(pinMoteurPwm,OUTPUT);
  pinMode(pinSensMoteur,OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 9 et 10 à (31 250Hz)  
  
  Angle = mesure();
  lastAngle=Angle;
  output=0;
  prevOutput=0;
  consigne = -20;
  delay(1000);
  tAbs=millis();
  tempsAbs=millis();
}

void loop()
{
  limite(5,125);
  if(millis()-tempsAbs<=5000){
    somme=0;
    for(int i=0;i<10;i++){
      somme+=mesure();
    }
    if(millis()-tAbs>=sampleTime){
      Angle = somme/10;
      vitesse=(double)1000*(Angle-lastAngle)/sampleTime;
      erreur=consigne-vitesse;
      output=prevOutput+Kp*erreur;
      output=constrain(output,-255,255);
      tournerMoteur(output);
      Serial.print("angle = ");
      Serial.print(Angle);
      Serial.print("\tvitesse = ");
      Serial.print(vitesse);
      Serial.print("\terreur = ");
      Serial.print(erreur);
      prevOutput=output;
      lastAngle=Angle;
      tAbs+=sampleTime;
    } 
  }
  else{
    tournerMoteur(0);
    Serial.println("fini !");
  }
}

void tournerMoteur(double out){
  Serial.print("\tsortie = ");
  Serial.print(int(round(out)));
  Serial.println("");
  if(out<0){
    digitalWrite(pinSensMoteur,LOW);
    out=-out;
  }
  else{
    digitalWrite(pinSensMoteur,HIGH);
  }
  
  analogWrite(pinMoteurPwm,int(round(out))); 
}

double mesure(){
  double value = analogRead(pinAnalogique);
  return (double)coeffDir*value+ordonneeOrigine;
}

void limite(double amin, double amax){
  if(Angle<amin){
    Serial.println("DANGER LIMITES DEPASSEES"); 
   digitalWrite(pinSensMoteur,HIGH);
   tournerMoteur(0);
   delay(1000);
  }
  else if(Angle>amax){
    Serial.println("DANGER LIMITES DEPASSEES"); 
    digitalWrite(pinSensMoteur,LOW);
    tournerMoteur(0);
   delay(1000);
  }
}
  
