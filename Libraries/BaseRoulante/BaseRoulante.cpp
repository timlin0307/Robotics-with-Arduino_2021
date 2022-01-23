#include "BaseRoulante.h"

//Constructeur:
BaseRoulante::BaseRoulante(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues) //pour jean-Michel : empattement = 207.5, rayon roue = 39.4
{
    m_roueGauche = roueGauche;
    m_roueDroite = roueDroite;
    m_empattement = empattement; // ! \\ Il faut ABSOLUMENT calibrer ce valeur pour améliorer la précision du robot.
    m_rayon = rayonRoues; // ! \\ Il faut ABSOLUMENT calibrer cette valeur pour améliorer la précision du robot.
    setPosition(0,0,0); //x,y,angle, Par défaut
    m_encodeurPrecedentGauche = m_roueGauche->getCodeur();  //normalement il vaut 0 au débbut
    m_encodeurPrecedentDroit = m_roueDroite->getCodeur();   //normalement il vaut 0 au débbut
    setPeriode(m_periode);

    setIntegralSaturation(2); //Si asservissement intégral : tuto choix valeur sur la def de la fonction
}

void BaseRoulante::debug()
{
    int diff = millis() - lastTimeDebug;
    if (diff >10*m_periode) lastTimeDebug = millis() - m_periode;
    if(diff>=m_periode)
    {
        if (!Serial.available()) Serial.begin(BAUDRATE);

        if (m_debugMode==1)
        {
            //Debug de position
            Serial.print(millis());
            Serial.print("\t\tX ");
            Serial.print(m_posX);
            Serial.print("\tY ");
            Serial.print(m_posY);
            Serial.print("\tangle ");
            Serial.print(m_angle);
            Serial.print("\td ");
            Serial.print(m_distance);

            if(m_debug_asservissement)
            {
                Serial.print("\t\tC_rot°\t");   //consigne de rotation
                Serial.print(m_vitesseRotationSvrPt*m_modeSvrPt+m_vitesseRotation*(!m_modeSvrPt));
                Serial.print("\tC_vit\t");      //consigne de vitesse
                Serial.print(m_vitesseMoyenne);
                Serial.print("\tC_angle\t");
                Serial.print(m_consigneAngleSvrPt*m_modeSvrPt+m_consigneAngle*(!m_modeSvrPt));
            }
        }
        else if (m_debugMode == 2)
        {
            Serial.print("t\tX\tY\tangle");
            if(m_debug_asservissement)
            {
                Serial.print("\tC_rot\tC_vit\tC_angle\td");
            }
            m_debugMode++;
        }
        else if(m_debugMode ==3)
        {
            Serial.print(millis());
            Serial.print("\t");
            Serial.print(m_posX);
            Serial.print("\t");
            Serial.print(m_posY);
            Serial.print("\t");
            Serial.print(m_angle);
            if(m_debug_asservissement)
            {
                Serial.print("\t");
                Serial.print(m_vitesseRotationSvrPt*m_modeSvrPt+m_vitesseRotation*(!m_modeSvrPt));
                Serial.print("\t");
                Serial.print(m_vitesseMoyenne);
                Serial.print("\t");
                Serial.print(m_consigneAngleSvrPt*m_modeSvrPt+m_consigneAngle*(!m_modeSvrPt));
                Serial.print("\t");
                Serial.print(m_distance);
                Serial.print("\t");
            }
        }

        Serial.println();
    }
}

void BaseRoulante::setDebugMode(byte mode)
{
    setDebugMode(mode, false);
}

void BaseRoulante::setDebugMode(byte mode, bool debugAsserv)
{
    m_debug_asservissement=debugAsserv;
    m_debugMode = mode;
    if (mode ==2) debug();
}


void BaseRoulante::allerRetour(float x, float y)
{
    if (!m_setupAllerRetour)
    {
        xInit = m_posX;
        yInit = m_posY;
        angleInit = m_angle;
        m_setupAllerRetour = true;
    }
    switch (m_etape)
    {
    case 0 :
        if (BaseRoulante::suivrePoint(xInit+x,yInit+y,5))
        {
            m_compteur++;
        }
        else
        {
            m_compteur = 0;
        }

        if (m_compteur>200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    case 1 :
        if (BaseRoulante::tournerPrecis(angleInit+180,2))
        {
            m_compteur++;
        }
        else
        {
            m_compteur = 0;
        }
        if (m_compteur > 200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    case 2 :
        if (BaseRoulante::suivrePoint(xInit,yInit,5))
        {
            m_compteur++;
        }
        else
        {
            m_compteur = 0;
        }
        if (m_compteur>200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    case 3 :
        if (BaseRoulante::tournerPrecis(angleInit,2))
        {
            m_compteur++;
        }
        else
        {
            m_compteur = 0;
        }
        if (m_compteur > 200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    default :
        m_etape = 0;
        break;
    }
    BaseRoulante::actualiserPosition();
}


void BaseRoulante::actualiserPosition()
{
    long encodeurGauche = m_roueGauche->getCodeur();
    long encodeurDroit = m_roueDroite->getCodeur();
    int nbPasEncodeurGauche = encodeurGauche - m_encodeurPrecedentGauche;
    int nbPasEncodeurDroit = encodeurDroit - m_encodeurPrecedentDroit;
    m_encodeurPrecedentGauche = encodeurGauche;
    m_encodeurPrecedentDroit = encodeurDroit;

    //dans un intervalle de temps :
    double distance_parcourue_roue_gauche = (double) nbPasEncodeurGauche * 2 * PI * m_rayon / m_roueGauche->getReducteur();
    double distance_parcourue_roue_droite = (double) nbPasEncodeurDroit * 2 * PI * m_rayon / m_roueDroite->getReducteur();
    double distance_parcourue = (distance_parcourue_roue_gauche + distance_parcourue_roue_droite) / 2;     // on constate avec un raisonmment physique que la distance parcourue par le ce,tre du robot est la moitiÃ© de la somme de celles parcourues par chacunes de ses roues
    m_angle = m_angle - 180/PI*atan((distance_parcourue_roue_gauche - distance_parcourue_roue_droite) / (2 * m_empattement));   // cf definition d'un angle en radian
    m_angle = modulo360(m_angle); // permet d'avoir un angle entre -180 et +180°
    m_posX += distance_parcourue * cos(PI/180*m_angle);
    m_posY += distance_parcourue * sin(PI/180*m_angle);
}

bool BaseRoulante::tournerPrecis(float theta, float precision) // le robot vise l'angle theta en radians
{
  m_vitesseMoyenne = 0; // pour éviter qu'il aille lentement !
  m_modeSvrPt= false;
  m_consigneAngle = modulo360(theta);
  correctionAngle.Compute();
  if (abs(theta - m_angle) <= precision) {
    avancerTourner(0, 0);
    return true;
  }
  BaseRoulante::avancerTourner(0, m_vitesseRotation);
  return false;
}

bool BaseRoulante::suivrePoint(float *coordonnees, float precision)
{
    return suivrePoint(coordonnees[0],coordonnees[1],precision);
}

bool BaseRoulante::suivrePoint(float xCible, float yCible, float precision)
{
    m_modeSvrPt = true;
    m_distance = sqrt(pow((m_posX - xCible), 2) + pow((m_posY - yCible), 2));
    int sens = ((cos(PI/180*m_angle) * (xCible - m_posX) + sin(PI/180*m_angle) * (yCible - m_posY) ) > 0 ) * 2 - 1; // produit scalaire pour savoir si le robot a dépassé la cible
    m_distance = - sens * m_distance;
    correctionDistance.Compute();

    m_consigneAngleSvrPt = 180/PI*atan((yCible - m_posY) / (xCible - m_posX));
    if(abs(modulo360(m_angle-m_consigneAngleSvrPt))>90) m_consigneAngleSvrPt=modulo360(180+m_consigneAngleSvrPt);
    correctionAngleSvrPt.Compute();
    if (abs(m_distance) >= precision) //vérifier que abs fonctionne tout le temps avec un float !
    {
        BaseRoulante::avancerTourner((m_vitesseMoyenne / (1 + abs(m_vitesseRotationSvrPt)*m_rapportAvancerTourner)), m_vitesseRotationSvrPt);
        return false;
    }
    else
    {
        BaseRoulante::avancerTourner(0, 0);
        return true;
    }
}


void BaseRoulante::avancerTourner(float v, float theta)
{
  theta = constrain(theta, -150, 150);
  v = constrain(v, -m_vitesseMoyenneMax + abs(theta), m_vitesseMoyenneMax - abs(theta));
  avancer(v-theta,v+theta);
}

void BaseRoulante::avancer(float vg, float vd)
{
    float acc = m_acc;
    if (m_vg*vg<0) m_vg =0;
    else if (abs(m_vg)<abs(vg)) m_vg = constrain(vg,m_vg-acc,m_vg+acc); //le constrain permet de ne pas accelerer trop fort
    else m_vg = vg;
    if (m_vd*vd<0) m_vd =0;
    else if (abs(m_vd)<abs(vd)) m_vd = constrain(vd,m_vd-acc,m_vd+acc); //le constrain permet de ne pas accelerer trop fort
    else m_vd = vd;

    m_roueGauche->tourneRPM(m_vg);
    m_roueDroite->tourneRPM(m_vd);
}

void BaseRoulante::setPosition(double x, double y, double angle)
{
    m_posX = x;
    m_posY = y;
    m_angle = angle;
}

double BaseRoulante::getPosX()
{
    return m_posX;
}

double BaseRoulante::getPosY()
{
    return m_posY;
}

double BaseRoulante::getAngle()
{
    return m_angle;
}

void BaseRoulante::setRapportAvancerTourner(float r)
{
    m_rapportAvancerTourner = r;
}

void BaseRoulante::setAngleCorrecteur(float kp, float ki, float kd)
{
    correctionAngle.SetTunings(kp,ki,kd);
}

void BaseRoulante::setDistanceCorrecteur(float kp, float ki, float kd)
{
    correctionDistance.SetTunings(kp,ki,kd);
}

void BaseRoulante::setAngleSvrPtCorrecteur(float kp, float ki, float kd)
{
    correctionAngleSvrPt.SetTunings(kp,ki,kd);
}

double BaseRoulante::modulo360(double angle) //retourne un angle entre -180 et 180°
{
  return angle - 360 * floor((angle + 180) / 360);
}

void BaseRoulante::setIntegralSaturation(float sat)
{// /!\ Cette fonction est utile que si le Robot a un asservissement en position Intégral !!

  /* Cette saturation va permettre au robot d'avoir un asservissement intégrale efficace
   * en évitant au maximum le dépassement de la consigne.
   * Si le robot ne pocède pas de moteurs asservis en vitesse (chose faite grâce à la Classe DCMotor) :
       * Il faut mettre la valeur à partir de laquelle le robot avance en utilisant
       * la fonction avancer(valeur,valeur);
   * Sinon, ça se fait au jugé, typiquement 2 c'est pas mal !
   */
    correctionAngle.SetIntegralSaturation(sat);
    correctionDistance.SetIntegralSaturation(sat);
}

void BaseRoulante::setPeriode(byte periode)
{
    m_periode = periode;
    m_roueGauche->setPeriode(m_periode);
    m_roueDroite->setPeriode(m_periode);
    correctionAngle.SetSampleTime(periode);
    correctionDistance.SetSampleTime(periode);
    correctionAngleSvrPt.SetSampleTime(periode);
}

void BaseRoulante::setAcceleration(float acc)
{
    m_acc =acc;
}

float BaseRoulante::getVg()
{
    return m_vg;
}

float BaseRoulante::getVd()
{
    return m_vd;
}

