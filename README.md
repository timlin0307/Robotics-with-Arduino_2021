# Robotics-with-Arduino_2021

---
## Introduction
- C'est un projet de cours Project Innovation à ECM (1A). Le projet a pour objectif de se familiariser avec les problématiques rencontrées en robotique, il s’organisera en synergie avec le groupe d’élèves de 2A participant à la coupe de France de robotique.
- L’objectif de ce projet est représentation de l’École au plus grand événement de robotique national : La Coupe de France de Robotique (CFR) 2021.

---
## Règlement de CFR Général
- Cette année (comme l’an dernier) , la CFR a pour thème : « Sail the World »; et se concentre sur le domaine maritime.
- Nous devons donc concevoir un robot et un phare qui vont devoir réaliser des tâches en rapport avec ce thème afin de gagner des points. Lors du concours, les robots des différentes équipes se rencontrent lors de matchs opposants deux équipes sur un terrain de 2 mètres par 3 mètres :
  <p align="center">
    <img src="https://user-images.githubusercontent.com/54052564/150683843-1a4257ba-3c8e-430c-8533-80842a31d256.png" />
  </p>
- Les robots de chaque équipe commencent dans leurs aires de départ respectives et ont alors 100 secondes pour réaliser différentes tâches afin de gagner un maximum de points.

---
## Stratégie
- Présentant deux robots à la CFR et un match durant 100 secondes, nous nous sommes répartis les actions à mener pour essayer de marquer un maximum de points dans le temps imparti. L’équipe des 2A s’occupant de la création des chenaux, notre équipe s’occupe des autres actions, plus nombreuses et diverses mais moins chronophages et répétitives.
- Les actions à effectuer par notre robot sont réalisées dans l’ordre suivant :
  - Allumer le phare : 15 points
  - Arriver à bon port (lire la girouette et transmettre l’information) : 10 points
  - Hisser le pavillon : 10 points
- De plus, durant les 100 secondes que dure un match, les robots ne doivent pas se heurter ou se foncer dessus, sous peine de recevoir de lourdes pénalités. Notre stratégie est simple : le robot doit s’arrêter s’il détecte un autre robot. Cette stratégie est sûre mais comprend l’inconvénient majeur que si le robot à éviter possède la même stratégie, les deux robots seront immobiles jusqu’à la fin du match.

---
## Annuaires
```
Robotics with Arduino 2021 :.
├─ Libraries
│  ├─ BaseRoulante
│  │  ├─ BaseRoulante.cpp
│  │  ├─ BaseRoulante.h
│  │  └─ ...
│  │
│  ├─ DCMotor
│  │  ├─ DCMotor.cpp
│  │  ├─ DCMotor.h
│  │  └─ ...
│  │
│  └─ PID_v1
│     ├─ PID_v1.cpp
│     ├─ PID_v1.h
│     └─ ...
│
├─ Phare
│  └─ Phare.ino
│
├─ SEN-Color
│  └─ SEN-Color.ino
│
├─ Trajet
│  └─ Trajet.ino
│
└─ README.md
```

---
## Travail Réalisé
1. **Le Moteur avec Encodeur** : 
2. **Le Capteur Ultrasonique** : Nous avons implémentéun système de capteurs afin de détecter les obstacles lors du déplacement du robot. Les capteurs mesurent la distance entre le robot et l'obstacle et si elle est inférieure à une distance critique, il stoppe sa progression. Le cas échéant, il poursuit son parsours.
3. **Le Phare** : Il est une pièce à part entière du robot. Sa fonction est d'indiquer la parti rocheuse du plateau au robot. Au cours de la partie, le robot l'activera avec un contact physique et le phare va se déplier tout en s'éclairant. La levée du phare se fait par un système poulies et de cordage animés par un moteur.
4. **Le Capteur de Couleur** : Disposée au centre de la table de jeu, la girouette indique la zone de mouillage dans laquelle les robots doivent se trouver à la fin du match. Sur cette girouette, elle se décline en deux couleurs différentes. Nous utilisons ce capteur pour détecter la couleur et faire savoir au robot ce que la girouette indique.
5. **Le Pavillon** : Il sert à montrer les couleurs de notre école en faisant lever par le robot les différents drapeaux au cours de la partie. Il est positionné sur la face latérale du robot. Le hissage des pavillons se fait par un système de roue et railles crantés.

---
## Supports


---
## Référence
