# Autonomous Navigation with Tentacles Method (TIPE)

Projet d'initiative personnelle encadrée (TIPE) sur le thème **« La Ville »**[cite: 7, 9].  
Implémentation et simulation sous Python d'un algorithme réactif d'évitement d'obstacles et de génération de trajectoires pour véhicule autonome, basé sur la **méthode des tentacules** (F. von Hundelshausen et al., 2008).

![Clothoïdes](assets/GIF_obstacle.png)

## 📌 Présentation du Projet
L'objectif est d'assurer la navigation sécurisée d'un véhicule en environnement urbain contraint[cite: 7]. Face aux limites d'une première approche matricielle discrète en temps de calcul[cite: 1, 7], le projet s'appuie sur une structure orientée objet :
- Échantillonnage de trajectoires candidates (tentacules) sous forme d'**arcs de cercle** et de **clothoïdes** (intégrales de Fresnel via `scipy.integrate`)[cite: 4, 5, 9].
- Calcul des corridors de sécurité via `shapely` (zones de support et distances de collision dépendantes de la vitesse)[cite: 4, 5, 6].
- Évaluation de navigabilité et sélection dynamique du tentacule optimal minimisant les variations de braquage[cite: 5, 9].

## 📂 Structure du Répertoire
- `final_version.py` : Script principal exécutant la simulation complète (dynamique du véhicule, animation interactive et suivi en temps réel vitesse/braquage).
- `docs/` : Support de présentation orale, MCOT et article de recherche de référence[cite: 6, 7, 9].
- `archive_dev/` : Historique des versions et prototypes de recherche intermédiaires (approche matricielle, expérimentations géométriques conservées en l'état)[cite: 1, 2, 4].

## ⚙️ Installation & Utilisation
```bash
git clone [https://github.com/](https://github.com/)<votre-pseudo>/autonomous-vehicle-tentacles-tipe.git
cd autonomous-vehicle-tentacles-tipe
pip install -r requirements.txt
python final_version.py
```

## 📚 Références bibliographiques

1. **F. von Hundelshausen, M. Himmelsbach, F. Hecker, A. Mueller, H.-J. Wuensche**,  
   *Driving with Tentacles: Integral Structures for Sensing and Motion*,  
   Journal of Field Robotics, vol. 25, n° 9, pp. 640–673, 2008.  
   DOI : [10.1002/rob.20256](https://doi.org/10.1002/rob.20256)

2. **M. Himmelsbach, T. Luettel, F. Hecker, F. von Hundelshausen, H.-J. Wuensche**,  
   *Autonomous Off-Road Navigation for MuCAR-3: Improving the Tentacles Approach: Integral Structures for Sensing and Motion*,  
   KI - Künstliche Intelligenz, vol. 25, n° 2, pp. 143–148, 2011.  
   DOI : [10.1007/s13218-011-0091-1](https://doi.org/10.1007/s13218-011-0091-1)

3. **G. Tagne Fokam**,  
   *Commande et planification de trajectoires pour la navigation de véhicules autonomes*,  
   Thèse de doctorat, Université de Technologie de Compiègne (UTC), 2014.  
   Archive ouverte : [HAL Id: tel-01160233](https://tel.archives-ouvertes.fr/tel-01160233)

4. **R. Rajamani**,  
   *Vehicle Dynamics and Control*, Mechanical Engineering Series,  
   Springer, New York, 2006 (e-ISBN: 0-387-28823-6).  
   DOI : [10.1007/978-0-387-28823-6](https://doi.org/10.1007/978-0-387-28823-6)

5. **H. B. Pacejka**,  
   *Tyre and Vehicle Dynamics*, 2nd Edition,  
   Elsevier / Butterworth-Heinemann, 2006 (ISBN: 978-0-7506-6918-4).  
   DOI : [10.1016/B978-0-7506-6918-4.X5000-X](https://doi.org/10.1016/B978-0-7506-6918-4.X5000-X)
