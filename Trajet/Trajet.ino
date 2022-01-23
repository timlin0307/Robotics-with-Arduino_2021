// #include <NewPing.h>
#include <DCMotor.h>
#include <BaseRoulante.h>
#include <PID_v1.h>
#include <Arduino.h>

// Bras
int motor_1 = 49;
int motor_2 = 50;
int motor_pwm = 46;
byte motor_speed = 40;
int moteurBras1 = 30;
int moteurBras2 = 31;
int moteurBras_enb = 34;
unsigned long duree;

// Capteur choses sur le robot
int pinTrigger_g = 22;
int pinTrigger_d = 32;
int pinTrigger_c = 24;
int pinTrigger_b = 11;
int pinEcho_g = 23;
int pinEcho_d = 33;
int pinEcho_c = 25;
int pinEcho_b = 10;
// bool tourner = false;
// float distance_max = 200;
float retour_g = 0;
float retour_d = 0;
float retour_c = 0;
float retour_b = 0;
int led_test = 38;
int led_show = 37;

// Autres choses sur le robot
int bleu_changer_intr_2a = 44;
int bleu_changer_intr_2b = 43;
int bleu_changer_intr_2c = 42;
int jaune_intr_1 = 51;
int jaune_intr_2 = 52;
int led_start = 29;
bool parcour_commencer = false;
int bleu_changer_intr_1a = 41;
int bleu_changer_intr_1b = 40;
char parcour_direct = 'G';
int led_direct_g = 27;
int led_direct_d = 26;
int led_mode_0 = 36;
int led_mode_1 = 35;
int color_get = 45;
int color_show = 28;

// Moteur choses sur le robot
byte codeurGauche1 = 20; 
byte codeurGauche2 = 21;
byte codeurDroite1 = 19; 
byte codeurDroite2 = 18;
byte Moto_gauche_PWM = 7;
byte Moto_gauche_IN1 = 6;
byte Moto_gauche_IN2 = 5;
byte Moto_droit_PWM = 2;
byte Moto_droit_IN1 = 4;
byte Moto_droit_IN2 = 3;

// Certaines variables
float precision_position = 3; // larger number faster speed
float precision_angle = 1; // larger number faster speed
int etape_trajet = 0;
int etape_avoid = 0;
int etape_bras = 0;
int led_change = 1;
int times = 0;
int mode = 2;
int pavillon_count = 0;
bool mode1_stop = false;
int run_time = 0;

// Definie les deux detecteurs
// NewPing sonar_g(pinTrigger_g, pinEcho_g, distance_max);
// NewPing sonar_d(pinTrigger_d, pinEcho_d, distance_max);

// Faire bien correspondre les branchements !!
DCMotor moteurDroit(Moto_droit_PWM, Moto_droit_IN1, Moto_droit_IN2, codeurDroite1 , codeurDroite2);  // PWM, IN1, IN2, VOIEA, VOIEB
DCMotor moteurGauche(Moto_gauche_PWM, Moto_gauche_IN1, Moto_gauche_IN2, codeurGauche1 , codeurGauche2); // PWM, IN1, IN2, VOIEA, VOIEB

// Fonction structure : BaseRoulante(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues)
// Roue gauche, roue droite, empattement = 207.5, rayon roue = 39.4
BaseRoulante monRobot = BaseRoulante(&moteurGauche, &moteurDroit, 207.5/2, 39.5);

// float now_px = monRobot.getPosX();
// float now_py = monRobot.getPosY();
// float now_ag = monRobot.getAngle();
// int now_step = 0;
float ori_px = 0;
float ori_py = 0;
float ori_ag = 0;

void setup() {
  Serial.begin(9600); // Serial output

  // Pavillon
  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  digitalWrite(motor_1, LOW);
  digitalWrite(motor_2, LOW);

  // Moteur bras
  pinMode(moteurBras1, OUTPUT);
  pinMode(moteurBras2, OUTPUT);
  digitalWrite(moteurBras1, LOW);
  digitalWrite(moteurBras2, LOW);

  // Ultrason
  pinMode(pinTrigger_g, OUTPUT);
  pinMode(pinEcho_g, INPUT);
  pinMode(pinTrigger_d, OUTPUT);
  pinMode(pinEcho_d, INPUT);
  pinMode(pinTrigger_c, OUTPUT);
  pinMode(pinEcho_c, INPUT);
  pinMode(pinTrigger_b, OUTPUT);
  pinMode(pinEcho_b, INPUT);
  pinMode(led_show, OUTPUT);
  
  // Interrupteur démarrage
  pinMode(jaune_intr_1, INPUT_PULLUP);
  pinMode(jaune_intr_2, OUTPUT);
  digitalWrite(jaune_intr_2, LOW);
  pinMode(led_start, OUTPUT);
  
  // Interrupeur choix de la mode
  pinMode(bleu_changer_intr_1a, INPUT_PULLUP);
  pinMode(bleu_changer_intr_1b, OUTPUT);
  digitalWrite(bleu_changer_intr_1b, LOW);
  pinMode(bleu_changer_intr_2a, INPUT_PULLUP);
  pinMode(bleu_changer_intr_2b, OUTPUT);
  pinMode(bleu_changer_intr_2c, INPUT_PULLUP);
  digitalWrite(bleu_changer_intr_2b, LOW);
  
  // LEDs parcours choisi
  pinMode(led_direct_g, OUTPUT);
  pinMode(led_direct_d, OUTPUT);

  // LEDs mode choisi
  pinMode(led_mode_0, OUTPUT);
  pinMode(led_mode_1, OUTPUT);

  // LED systeme timer1
  pinMode(led_test, OUTPUT);

  // color info get
  pinMode(color_get, INPUT);

  // Setup base roulante
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // permet d'augmenter la fréquence du pwm des pins 5, 3 et 2 au maximum (31 250Hz)
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // permet d'augmenter la fréquence du pwm des pins 8, 7 et 6 au maximum (31 250Hz)

  Timer1_Init();
  
  moteurDroit.setPID(8.387, -7.008, 0); // Reglage de l'asservissement en vitesse du moteur droit
  moteurGauche.setPID(5, -4.129, 0); // Reglage de l'asservissement en vitesse du moteur gauche
  attachInterrupt(digitalPinToInterrupt(codeurGauche1), motorCodeurIncrementalGA, CHANGE); // Permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurGauche2), motorCodeurIncrementalGB, CHANGE); // Permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurDroite1), motorCodeurIncrementalDA, CHANGE); // Permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurDroite2), motorCodeurIncrementalDB, CHANGE); // Permet d'automatiquement compter le nombre de tour de roue du moteur
  monRobot.setAngleCorrecteur(3, 0, 0); // P, I, D
  monRobot.setDistanceCorrecteur(0.33, 0, 0); // P, I, D
  monRobot.setAngleSvrPtCorrecteur(1, 0, 0.33); 
  monRobot.setRapportAvancerTourner(0.01); // Plus l'angle vers la cible est grand, moins on va vite
  monRobot.setPeriode(5); // Periode d'échantillonnage de tous les asservissements !
  monRobot.setAcceleration(0.15); // Accélération maximale du robot
  monRobot.setDebugMode(1, false); // 1 pour avoir le nom des données ; 2 sinon // 2ème paramètre otpionel : true pour avoir des données sur l'asservissement, false sinon
  monRobot.setPosition(0, 0, 0); // Initialise la position de départ du robot en millimètre : x, y , angle en degrès
  
  //delay(10);
}

// Fonctions pour les biblio de la base roulante 
void motorCodeurIncrementalDA(){
  moteurDroit.codeurIncrementalA();
}
void motorCodeurIncrementalDB(){
  moteurDroit.codeurIncrementalB();
}
void motorCodeurIncrementalGA(){
  moteurGauche.codeurIncrementalA();
}
void motorCodeurIncrementalGB(){
  moteurGauche.codeurIncrementalB();
}

void robotSuivre_trajet(float x, float y, float pc) {
  monRobot.actualiserPosition();
  if(monRobot.suivrePoint(x, y, pc)) {
    // Serial.println(etape_trajet);
    etape_trajet++;
  }
  // now_step = 0;
  
}

void robotTourner_trajet(float theta, float pc) {
  monRobot.actualiserPosition();
  if(monRobot.tournerPrecis(theta, pc)) {
    // Serial.println(etape_trajet);
    etape_trajet++;
  }
  // now_step = 1;
}

void robotSuivre_avoid(float x, float y, float pc) {
  monRobot.actualiserPosition();
  if(monRobot.suivrePoint(x, y, pc)) {
    etape_avoid++;
  }
}

void robotTourner_avoid(float theta, float pc) {
  monRobot.actualiserPosition();
  if(monRobot.tournerPrecis(theta, pc)) {
    etape_avoid++;
  }
}

void trajetDepartG() {
  switch(etape_trajet){
    case 0:
      robotSuivre_trajet(300, 0, precision_position);
      ori_px = 300;
      ori_py = 0;
      break;
    case 1:
      robotTourner_trajet(90, precision_angle);
      ori_ag = 90;
      break;
    case 2:
      robotSuivre_trajet(300, 300, precision_position);
      ori_px = 300;
      ori_py = 0;
      break;
    case 3:
      robotTourner_trajet(180, precision_angle);
      ori_ag = 180;
      break; 
    case 4:
      robotSuivre_trajet(0, 300, precision_position);
      ori_px = 300;
      ori_py = 0;
      break;
    case 5:
      robotTourner_trajet(-90, precision_angle);
      ori_ag = -90;
      break;
    case 6:
      robotSuivre_trajet(0, 0, precision_position);
      ori_px = 300;
      ori_py = 0;
      break;
    case 7:
      robotTourner_trajet(0, precision_angle);
      ori_ag = 0;
      break;
  }
}

void trajetDepartD() {
  switch(etape_trajet){
    case 0:
      robotTourner_trajet(180, precision_angle);
      ori_ag = 180;
      break;
    case 1:
      robotSuivre_trajet(-300, 0, precision_position);
      ori_px = -300;
      ori_py = 0;
      break;
    case 2:
      robotTourner_trajet(90, precision_angle);
      ori_ag = 90;
      break;
    case 3:
      robotSuivre_trajet(-300, 300, precision_position);
      ori_px = -300;
      ori_py = 300;
      break;
    case 4:
      robotTourner_trajet(0, precision_angle);
      ori_ag = 0;
      break; 
    case 5:
      robotSuivre_trajet(0, 300, precision_position);
      ori_px = 0;
      ori_py = 300;
      break;
    case 6:
      robotTourner_trajet(-90, precision_angle);
      ori_ag = -90;
      break;
    case 7:
      robotSuivre_trajet(0, 0, precision_position);
      ori_px = 0;
      ori_py = 0;
      break;
  }
}

void avoidDepart() {
  switch(etape_avoid){
    case 0:
      robotTourner_avoid(-90, precision_angle);
      break;
    case 1:
      robotSuivre_avoid(0, -300, precision_position);
      break;
    case 2:
      robotTourner_avoid(0, precision_angle);
      break;
    case 3:
      robotSuivre_avoid(300, -300, precision_position);
      break;
    case 4:
      robotTourner_avoid(90, precision_angle);
      break; 
    case 5:
      robotSuivre_avoid(300, 0, precision_position);
      break;
    case 6:
      robotTourner_avoid(0, precision_angle);
      break;
    case 7:
      robotSuivre_avoid(600, 0, precision_position);
      break;
    case 8:
      robotTourner_avoid(180, precision_angle);
      monRobot.setPosition(0, 0, 0); // reset the position
      times++;
      break;    
  }
}

bool pavillon() {
  switch(pavillon_count){
    case 0:
      pavillon_count++;
      delay(1000);
      return true;
      break;
    case 1:
      pavillon_count++;
      delay(1000);
      return true;
      break;
    case 2:
      pavillon_count++;
      delay(1000);
      return true;
      break;
    case 3:
      pavillon_count++;
      delay(1000);
      return true;
      break; 
    case 4:
      digitalWrite(motor_1, HIGH);
      digitalWrite(motor_2, LOW);
      analogWrite(motor_pwm, motor_speed);
      pavillon_count++;
      return true;
      break;
    case 5:
      pavillon_count++;
      delay(1000);
      return true;
      break;
    case 6:
      digitalWrite(motor_1, LOW);
      digitalWrite(motor_2, LOW);
      pavillon_count++;
      return true;
      break;
    case 7:
      pavillon_count = 7;
      return false;
      break;
  }
  delay(100);
  // Serial.println(pavillon_count);
}

void activationBras() {
  switch(etape_bras) {
    case 0:
      digitalWrite(moteurBras1, HIGH);
      digitalWrite(moteurBras2, LOW);
      delay(1040);
      etape_bras++;
      break;
    case 1:
      delay(500);
      etape_bras++;
      break;
    case 2:
      digitalWrite(moteurBras1, LOW);
      digitalWrite(moteurBras2, HIGH);
      delay(1000);
      etape_bras++;
      break;
    case 3:
      delay(500);
      etape_bras++;
      break;
    case 4:
      etape_bras = 4;
      break;
  }
}

float ultrason(int trigger, int echo) {
  // return distance in unit "cm"
  int duration = 0;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  return (duration/2)/29.1;
}

void Timer1_Init(void) {
  noInterrupts();
  TCCR1A = 0; // Timer/Counter register which configure the prescaler
  TCCR1B = 0; // Timer/Counter register which configure the prescaler
  TCNT1 = 0; // Timer/Counter register which is stored timer value
  OCR1A = 31250; // compare match register : 16MHz/256/2Hz = 31250
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12); // 256 prescalar
  TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
  interrupts();
}

ISR(TIMER1_COMPA_vect) {  
  // led high and low
  if(led_change == 1) {
    digitalWrite(led_test, HIGH);
  } else {
    digitalWrite(led_test, LOW);
  }
  led_change = -led_change;

  // Ultrason
  retour_g = ultrason(pinTrigger_g, pinEcho_g);
  retour_d = ultrason(pinTrigger_d, pinEcho_d);
  retour_c = ultrason(pinTrigger_c, pinEcho_c);
  retour_b = ultrason(pinTrigger_b, pinEcho_b);
  if((retour_g <= 20 && retour_g >= 3) || (retour_d <= 20 && retour_d >= 3) || (retour_c <= 20 && retour_c >= 3)) {
    // Serial.println("STOP");
    if(mode == 0) {
      times++;
      if(times == 1) {
        etape_avoid = 0;
      } else if(times == 4) {
        times = 0;
      }
    }
    if(mode == 1) {
      // now_px = monRobot.getPosX();
      // now_py = monRobot.getPosY();
      // now_ag = monRobot.getAngle();
      times = 1;
      mode1_stop = true;
    }
    digitalWrite(led_show, HIGH);
  } else {
    // Serial.println("KEEP");
    if(mode == 1) {
      times = 0;
      mode1_stop = false;
    }
    digitalWrite(led_show, LOW);
  }
  
  /* retour_g = sonar_g.convert_cm(sonar_g.ping_median());
  retour_d = sonar_d.convert_cm(sonar_d.ping_median());
  if(retour_g < 15 && retour_d < 15) {
    if((retour_g <= 10 || retour_d <= 10) && retour_g > 5 && retour_d > 5) {
      tourner = true;
      Serial.println("true");
      Serial.println(retour_g);
      Serial.println(retour_d);
    } else if(retour_g > 10 || retour_d > 10) {
      tourner = false;
      Serial.println("false");
      Serial.println(retour_g);
      Serial.println(retour_d);
    }
  } else {
    tourner = false;
  }
  Serial.println("------------------------");
  
  // Ultrason + Parcours
  if(parcour_commencer) {
    if(tourner) { 
      etape_trajet = etape_trajet - 2;
      if(etape_trajet <= 0) {
        etape_trajet = 0;
      }
      // monRobot.tournerPrecis(360, 1);
      tourner = false;
    }
  }
  Serial.println(etape_trajet);
  Serial.println("------------------------");*/
}

void loop() {
  // Démarrage
  if(digitalRead(jaune_intr_1)){
    parcour_commencer = true;
    digitalWrite(led_start, HIGH);
    // Serial.println("Le robot commence maintenant."); // delay(500);
  } else {
    parcour_commencer = false;
    digitalWrite(led_start, LOW);
    // Serial.println("Le robot arrete maintenant."); // delay(500);
  }
  
  // Choix du parcours
  if(digitalRead(bleu_changer_intr_1a)){
    digitalWrite(led_direct_d, LOW); // Parcours Droit
    digitalWrite(led_direct_g, HIGH);
    parcour_direct = 'D';
    // Serial.println("La direction est a droit."); // delay(500);
  } else {
    digitalWrite(led_direct_d, HIGH);
    digitalWrite(led_direct_g, LOW); // Parcours Gauche
    parcour_direct = 'G';
    // Serial.println("La direction est a gauche."); // delay(500);
  }

  // Choix de la mode
  if(digitalRead(bleu_changer_intr_2a) && !digitalRead(bleu_changer_intr_2c)){
    digitalWrite(led_mode_0, HIGH);
    digitalWrite(led_mode_1, LOW);
    mode = 0;
  } else if(digitalRead(bleu_changer_intr_2c) && !digitalRead(bleu_changer_intr_2a)) {
    digitalWrite(led_mode_0, LOW);
    digitalWrite(led_mode_1, HIGH);
    mode = 1;
  } else {
    digitalWrite(led_mode_0, LOW);
    digitalWrite(led_mode_1, LOW);
    mode = 2;
  }

  // Color Info Get
  if(digitalRead(color_get)) {
    digitalWrite(color_show, HIGH);
  } else {
    digitalWrite(color_show, LOW);
  }

  while(pavillon());

  // Démarrage + Parcours
  if(parcour_commencer) {
    if(mode == 0) {
      if(times == 1) {
        avoidDepart();
        // Serial.println("Eviter la collision."); // delay(500);
      }
    }
    if(mode == 1) {
      if(times == 1) {
        /*if(mode1_stop) {
          monRobot.actualiserPosition();
          if(now_step == 0) {
            monRobot.suivrePoint(now_px, now_py, precision_position);
          }
          if(now_step == 1) {
            monRobot.tournerPrecis(now_ag, precision_angle);
          }
        }*/
        monRobot.avancer(0,0);
      } else {
        if(parcour_direct == 'G') {
          trajetDepartG(); // Trajet depart a gauche
          // Serial.println("Trajet depart a gauche."); // delay(500);
        }
        if(parcour_direct == 'D') {
          trajetDepartD(); // Trajet depart a droit
          // Serial.println("Trajet depart a droit."); // delay(500);
        }
      }
    }
    if(mode == 2) {
      monRobot.avancer(0,0);
      activationBras();
    }
  }
  // Serial.println(etape_trajet);
  if(etape_trajet == 8) {
    etape_trajet = 0;
  }
}
