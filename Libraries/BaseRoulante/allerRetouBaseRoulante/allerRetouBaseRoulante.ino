#include <BaseRoulante.h>

byte codeurGauche1 = 20; //voie A du codeur incrémental gauche // pour interruptPin seulement les pin 2, 3, 18, 19, 20 , 21 fonctionnent
byte codeurGauche2 = 21; //voie B du codeur incrémental gauche // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite1 = 19; //voie A du codeur incrémental Droit  // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite2 = 18; //voie B du codeur incrémental Droit  // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent

//faire bien correspondre les branchements !!
DCMotor moteurDroit(2, 4, 3, codeurDroite1 , codeurDroite2);  //PWM, IN1, IN2, VOIEA, VOIEB
DCMotor moteurGauche(7,6,5, codeurGauche1 , codeurGauche2); //PWM, IN1, IN2, VOIEA, VOIEB

BaseRoulante monRobot = BaseRoulante(&moteurGauche,&moteurDroit, 207.5/2, 39.4); //roue gauche, roue droite, empattement = 207.5, rayon roue = 39.4


void setup() 
{ 
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 5,3 et 2 au maximum (31 250Hz)
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 8,7 et 6 au maximum (31 250Hz)
  
  moteurDroit.setPID(8.387, -7.008, 0); //Reglage de l'asservissement en vitesse du moteur droit
  moteurGauche.setPID(5, -4.129, 0);    //Reglage de l'asservissement en vitesse du moteur gauche
  
  attachInterrupt(digitalPinToInterrupt(codeurGauche1), motorCodeurIncrementalGA, CHANGE);  //permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurGauche2), motorCodeurIncrementalGB, CHANGE);  //permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurDroite1), motorCodeurIncrementalDA, CHANGE);  //permet d'automatiquement compter le nombre de tour de roue du moteur
  attachInterrupt(digitalPinToInterrupt(codeurDroite2), motorCodeurIncrementalDB, CHANGE);  //permet d'automatiquement compter le nombre de tour de roue du moteur
  monRobot.setAngleCorrecteur(3, 0, 0);  //P, I, D
  monRobot.setDistanceCorrecteur(0.33, 0, 0); //P, I, D
  monRobot.setAngleSvrPtCorrecteur(1, 0, 0.33); 
  monRobot.setRapportAvancerTourner(0.01); // plus l'angle vers la cible est grand, moins on va vite
  monRobot.setPeriode(5); //Periode d'échantillonnage de tous les asservissements !
  monRobot.setAcceleration(0.15); //accélération maximale du robot
  monRobot.setDebugMode(1,false); // 1 pour avoir le nom des données ; 2 sinon // 2ème paramètre otpionel : true pour avoir des données sur l'asservissement, false sinon
  monRobot.setPosition(0,0,0);  //initialise la position de départ du robot en millimètre : x, y , angle en degrès
}

void loop() 
{
  monRobot.actualiserPosition(); // /!\ obligatoire à chaque tour de boucle
  monRobot.allerRetour(500,0);  //effectue des aller retour entre le point actuel et le point ciblé en paramètre : x, y en mm
  monRobot.debug();   //Avec un Baudrate sur le serial de 1000000
}

void motorCodeurIncrementalDA()
{
  moteurDroit.codeurIncrementalA();
}

void motorCodeurIncrementalDB()
{
  moteurDroit.codeurIncrementalB();
}

void motorCodeurIncrementalGA()
{
  moteurGauche.codeurIncrementalA();
}

void motorCodeurIncrementalGB()
{
  moteurGauche.codeurIncrementalB();
}
