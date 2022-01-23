#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(byte brochePWM, byte IN1, byte IN2, byte voieA, byte voieB)
{


    m_broche = brochePWM;
    m_IN1 = IN1;
    m_IN2 = IN2;
    m_voieA = voieA;
    m_voieB = voieB;

    pinMode(m_broche,OUTPUT);
    pinMode(m_voieA,INPUT_PULLUP);
    pinMode(m_voieB,INPUT_PULLUP);
    pinMode(m_IN1,OUTPUT);
    pinMode(m_IN2,OUTPUT);
    digitalWrite(m_IN1, LOW);
    digitalWrite(m_IN2, LOW);

//    TCCR3B = TCCR3B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm des pins 5,3 et 2 au maximum (environ 30kHz)
//    TCCR4B = TCCR4B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm des pins 6,7 et 8 au maximum
}

void DCMotor::actualiserVitesse()
{
    int ticks = m_compteur- m_precedentCompteur; //on compte le nombre de ticks depuis le dernier appel de la fonction actualiser vitesse
    m_precedentCompteur=m_compteur;//on met à jour le compte total de ticks

    m_vitesse = ticks * 60000 / (m_a *(m_periode)); //On calcule la vitesse moyenne sur la période en tr/min.

}

void DCMotor::tourneRPM(float consigne)
{ 
    unsigned long diff = millis() - m_temps_absolu; //on calcule le temps écoulé depuis la dernière action du correcteur
    if (diff > 3*m_periode)  m_temps_absolu = millis() - m_periode;//ligne pour éviter que le moteur ne s'emballe à l'initialiation lorsque temps_absolu vaut 0 et que millis peut etre très grand
    if (diff >=m_periode) //le correcteur agit toutes les périodes

    {
        actualiserVitesse();//on met à jour la vitesse du moteur
        m_error = consigne-m_vitesse;//on met à jour l'a vitesse du moteur l'erreur
        m_command = m_previousCommand + m_Kp * m_error + m_Ki * m_previousError; //équation de récurrence PI
        int commandePuissance = constrain(int(m_command),-m_sat,m_sat);

        envoiCommande(commandePuissance);//envoi de la puissance au moteur
        m_previousError=m_error;//mise à jour des erreurs et commandes précédantes
        m_previousCommand=m_command;
        m_temps_absolu+=m_periode;
    }
}


void DCMotor::envoiCommande(int puissance)
{
    bool sens = puissance >= 0;

    if (puissance==0)//cas d'un arret : on freine le moteur
    {
        digitalWrite(m_IN1, LOW);
        digitalWrite(m_IN2, LOW);
    }
    else {
      puissance=abs(puissance)+m_offset;  // le +90 (offset de jean michel) est spécifique au pont en H utilisé et est dû à sa non linéarité
    }

    if (sens == 1){ //marche avant
      digitalWrite(m_IN1, HIGH);
      digitalWrite(m_IN2, LOW);
    }
    else {//marche arrière
      digitalWrite(m_IN1, LOW);
      digitalWrite(m_IN2, HIGH);
    }

    analogWrite(m_broche, puissance);//envoi de la puissance au moteur
}

void DCMotor::codeurIncrementalA()
{
  int sens = 2*int(digitalRead(m_voieB)==digitalRead(m_voieA))-1; // renvoie 1 si on tourne dans le sens horaire, -1 sinon (ou inversement! mdr)
  m_compteur = m_compteur + sens;
}

void DCMotor::codeurIncrementalB()
{
  int sens = 2*int(digitalRead(m_voieA)!=digitalRead(m_voieB))-1; // renvoie 1 si on tourne dans le sens horaire, -1 sinon (ou inversement! mdr)
  m_compteur = m_compteur + sens;
}

void DCMotor::setPID(float kp, float ki, float kd)
{
    m_Kp = kp;
    m_Ki = ki;
    m_Kd = kd;
}

void DCMotor::setDivisionFrequence(byte facteur)//méthode inutile mdr, cf header
{
    m_division_frequence = facteur; // la fréquence vaut 62500Hz ou 31250Hz (fréquence maximale fournie par la PWM => provient de la fréquence du quartz / 256)
}

void DCMotor::setPeriode(int periode)
{
    m_periode = periode;
}

void DCMotor::setOffset(int offset)
{
    m_offset = offset;
}

long DCMotor::getCodeur()
{
    return m_compteur;
}

float DCMotor::getVitesse()
{
    return m_vitesse;
}

float DCMotor::getReducteur()
{
    return m_a;
}

