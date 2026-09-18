# -*- coding: utf-8 -*-
"""
Created on Mon Dec 19 19:50:23 2022

@author: adaml
"""


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation
import matplotlib.patches
import matplotlib.transforms
import matplotlib.lines
import scipy.interpolate
import shapely
import shapely.geometry
import time
from scipy.integrate import quad as integrale


VITESSE_INIT_VOIT = 10  # vitesse initiale du véhicule
VITESSE_MAX_VOIT = 15  # vitesse maximale (pour rester dans les zones linéaires des modèles)
ACCEL_LAT_MAX = 1.5  # accélération latérale maximale (pour rester dans les zones linéaires des modèles)
LONGUEUR_VEHIC = 5  # longueur du véhicule
LARGEUR_VEHIC = 2  # largeur du véhicule
POSITION_VOITURE_INIT = (-10, -2)  # position (x, y) initiale de la voiture
INCLINAISON_VOITURE_INIT = 0  # inclinaison initiale de la voiture (en degrés)
BRAQUAGE_VOLANT_INIT = 10  # angle volant/voiture en degrés
PRECISION = 10  # nombre de points définissant l'arc de cercle. ie nombre de positions où la voiture ira
TEMPS_REAC = 90  # temps de génération de tentacules
CONE_BRAQUAGE = 0.02  # valeur de braquage définissant des trajectoires "lisses". cf get_tentacules_navigables_et_lisses
COMMENTAIRES = True  # si des commentaires sur l'évolution de l'algo doivent être affichés dans la console
PETIT_POUCET = True  # si on veut récupérer la trajectoire de la voiture
SECURITE = True  # vérifie les collisions. False si aucune sécu, None si check sans arrêter, True si arrêt lorsque collision

# courbure = ro
# courbure = 1/rayon dans le cas d'un cercle
# braquage = delta = arctan(L/rayon) quasi logique (schéma + perpendiculaire)
# longueur = angle*rayon
# empattement +-= longueur voiture


class _Tentacule():
    """
    Classe dont héritent tout type de tentacule. Elle rassemble des méthodes communes universelles. N'est pas censée être utilisée directement ; elle est appelée automatiquement par d'autres instances.
    """

    def __init__(self, axes, voiture):
        """
        Parameters
        ----------
        axes : matplotlib.axes._axes.Axes
            Axes dans lesquels la tentacule sera affichée.
        voiture : __main__.Voiture
            Voiture à laquelle appartient la tentacule.

        Returns
        -------
        None.

        """
        self.axes = axes
        self.voiture = voiture
        self.precision = PRECISION

    def get_zone_support(self):
        """
        Génère et stocke une zone de support pour la tentacule (distance de sécurité autour de la tentacule pour éviter quelconque collision).
        cf von Hundelshausen et al 2008 p.647

        Raises
        ------
        ValueError
            Si la vitesse du véhicule est trop grande (55km/h).

        Returns
        -------
        shapely.geometry.polygon.Polygon
            La zone de support.

        """
        if self.voiture.vitesse < 3:
            self.dist_zone_support = 1.4 + 0.2*self.voiture.vitesse/3  # formule empirique p.123 eq(9.6) thèse
        elif 3 <= self.voiture.vitesse <= 15:
            self.dist_zone_support = 1.6 + 0.6 * (self.voiture.vitesse-3) / 15  # idem
        else:
            raise ValueError("La vitesse est trop élevée !")
        self.zone_support = shapely.geometry.LineString(self.get_parametric()).buffer(self.dist_zone_support)
        return self.zone_support

    def is_navigable(self, objets_alentours):
        """
        Navigabilité de la tentacule.

        Parameters
        ----------
        objets_alentours : __main__.Voiture ou __main__.Obstacle
            Objet ayant un attribut 'shapely_poly'.

        Returns
        -------
        bool
            True si la tentacule est navigable, False sinon.

        """
        self.zone_to_be_clean = self.get_zone_support().intersection(self.voiture.get_espace_collision())
        for objet in objets_alentours:
            if objet == self.voiture:  # Pas grave si la tentacule cogne sa propre voiture
                pass
            elif self.zone_to_be_clean.intersection(objet.shapely_poly).is_empty is False:
                self.patch.set_color("red")
                return False
        return True


class Tentacule_inconnue(_Tentacule):
    """
    Tentacule space inconnue au bataillon. Fonctionnalités pas nécessaire pour notre étude donc classe aux focntionalités pas totalement développées.
    """

    def __init__(self, axes, voiture, patch, braquages):
        """
        Parameters
        ----------
        axes : matplotlib.axes._axes.Axes
            Axes dans lesquels la tentacule sera affichée.
        voiture : __main__.Voiture
            Voiture à laquelle appartient la tentacule.
        patch : matplotlib.patches.Patch
            Forme de la tentacule (étant donné que sa caractérisation n'est pas explicite.
        braquages : list ou itérable
            Itérable contenant les braquages successifs que prendra la voiture.

        Returns
        -------
        None.

        """
        super().__init__(axes, voiture)
        self.patch = patch
        self.braquages = braquages

    def get_parametric(self):
        """
        Permet d'obtenir la forme paramétrée de la tentacule. Utilise une interpolation.

        Returns
        -------
        chemin : numpy.ndarray
            Abscisses et ordonnées de la tentacule.
            Ex: numpy.array([[ x1,   y1],
                             [ x2,   y2],
                             [ x3,   y3]])

        """
        self.chemin = (self.patch.get_transform() - axes_simu.transData).transform_path(self.patch.get_path()).cleaned().vertices[:-1][::self.signe]
        # Interpolation du chemin
        self.chemin_interpol = scipy.interpolate.interp1d(self.chemin[:, 0], self.chemin[:, 1])
        lch = len(self.chemin[:, 0])
        # Interpolation de x(t) pour faire une génération d'abscisses supplémentaires
        points = scipy.interpolate.interp1d(range(0, (lch)*self.precision, self.precision), self.chemin[:, 0])  # va jusque (lch-1)*precision inclus
        # Génération des abscisses supplémentaires avec le points(range) et des ordonnées interpolées correspondantes avec le chemin_interpol(x)
        chemin = np.array([[x, self.chemin_interpol(x)] for x in points(range(0, (lch-1)*self.precision, lch-1))])
        return chemin

    def __iter__(self):
        """
        Implémente l'itérabilité.
        """
        self.chemin = self.get_parametric()
        return self

    def __next__(self):
        """
        Passe à l'élément suivant (la tentacule étant un itérable) : met à jour la forme paramétrée, et retourne les nouveaux états de la voiture.

        Raises
        ------
        StopIteration
            Si la tentacule est parcourue en entier.

        Returns
        -------
        new_x : float
            Nouvelle abscisse.
        new_y : float
            Nouvelle ordonnée.
        dx : float
            Déplacement effectué en x.
        dy : float
            Déplacement effectué en y.
        new_angle : numpy.float64
            Nouvel angle de la voiture.
        braquage : numpy.float64 ou celui de la liste de braquage passée en paramètre lors de la définition de la tentacule.
            Nouveau braquage de la voiture.

        """
        try:
            new_x = self.chemin[1][0]
            new_y = self.chemin[1][1]
            dx = new_x - self.chemin[0][0]
            dy = new_y - self.chemin[0][1]
            new_angle = np.degrees(np.arctan2(dy, dx))
            braquage = next(self.braquages)
            self.chemin = np.delete(self.chemin, 0, 0)
            return new_x, new_y, dx, dy, new_angle, braquage
        except IndexError:
            raise StopIteration("Fin de la tentacule")


class Tentacule_clothoïdaire(_Tentacule):
    """
    Tentacule en forme de clothoïde, dépendante d'une voiture et caractérisée par une courbure initiale et d'une variation de courbure.
    """

    def __init__(self, voiture, courbure_init, var_courbure, axes):
        """
        Parameters
        ----------
        voiture : __main__.Voiture
            Voiture à laquelle appartient la tentacule.
        courbure_init : float
            Courbure initiale de la tentacule.
        var_courbure : float
            Variation de la courbure.  # TODO: clarifier avec Adrien
        axes : matplotlib.axes._axes.Axes
            Axes dans lesquels la tentacule sera affichée.

        Returns
        -------
        None.

        """
        super().__init__(axes, voiture)

        self.longueur = 7*self.voiture.vitesse - 5  # formule empirique, équation (9.15) p.128
        self.courbure_init = courbure_init
        self.var_courbure = var_courbure
        self.courb_finale = self.courbure_init + self.var_courbure*self.longueur

        self.xdata = [self.x(t, self.var_courbure, self.courb_finale) for t in np.linspace(0, int(self.longueur), self.precision)]
        self.ydata = [self.y(t, self.var_courbure, self.courb_finale) for t in np.linspace(0, int(self.longueur), self.precision)]
        self.patch = matplotlib.lines.Line2D(self.xdata, self.ydata, color='green')
        self.patch.set_transform(matplotlib.transforms.Affine2D().rotate_deg_around(0, 0, voiture.angle) + matplotlib.transforms.Affine2D().translate(voiture.x, voiture.y) + self.axes.transData)

        self.abscisse_curviligne_carre = 0
        self.braquage = 0
        self.V_lisse = 0

    def x(self, t, var_courbure, courb_finale):
        """
        Abscisse à t de la clothoïde : intégrale de Fresnel.

        Parameters
        ----------
        t : numpy.float64
            Borne de l'intégrale.
        var_courbure : float
            Variation de la courbure.
        courb_finale : float
            Courbure finale de la clothoïde.

        Returns
        -------
        None.

        """
        return integrale(lambda s: np.cos(var_courbure*s**2 + courb_finale*s)/3, 0, t)[0]  # Intégrale de Fresnel avec facteur 1/3 correctif (meilleure mise à l'échelle)

    def y(self, t, var_courbure, courb_finale):
        """
        Ordonnée à t de la clothoïde : intégrale de Fresnel.

        Parameters
        ----------
        t : numpy.float64
            Borne de l'intégrale.
        var_courbure : float
            Variation de la courbure.
        courb_finale : float
            Courbure finale de la clothoïde.

        Returns
        -------
        None.

        """
        return integrale(lambda s: np.sin(var_courbure*s**2 + courb_finale*s)/3, 0, t)[0]  # Intégrale de Fresnel avec facteur 1/3 correctif (meilleure mise à l'échelle)

    def get_parametric(self):
        """
        Permet d'obtenir la forme paramétrée de la tentacule. Récupère simplement les points car clothoïde paramétrée par définition puis transformée dans le plan.

        Returns
        -------
        chemin : numpy.ndarray
            Abscisses et ordonnées de la tentacule.
            Ex: numpy.array([[ x1,   y1],
                             [ x2,   y2],
                             [ x3,   y3]])

        """
        chemin = (self.patch.get_transform() - axes_simu.transData).transform_path(self.patch.get_path()).cleaned().vertices[:-1]
        return chemin

    def __iter__(self):
        """
        Implémente l'itérabilité.
        """
        self.chemin = self.get_parametric()
        return self

    def __next__(self):
        """
        Passe à l'élément suivant (la tentacule étant un itérable) : met à jour la forme paramétrée, et retourne les nouveaux états de la voiture.

        Raises
        ------
        StopIteration
            Si la tentacule est parcourue en entier.

        Returns
        -------
        new_x : float
            Nouvelle abscisse.
        new_y : float
            Nouvelle ordonnée.
        dx : float
            Déplacement effectué en x.
        dy : float
            Déplacement effectué en y.
        new_angle : numpy.float64
            Nouvel angle de la voiture.
        braquage : numpy.float64 ou celui de la liste de braquage passée en paramètre lors de la définition de la tentacule.
            Nouveau braquage de la voiture.

        """
        try:
            new_x = self.chemin[1][0]
            new_y = self.chemin[1][1]
            dx = new_x - self.chemin[0][0]
            dy = new_y - self.chemin[0][1]
            new_angle = np.degrees(np.arctan2(dy, dx))
            self.abscisse_curviligne_carre += new_x**2 + new_y**2
            braquage = self.var_courbure * np.sqrt(self.abscisse_curviligne_carre)
            self.braquage = braquage
            self.chemin = np.delete(self.chemin, 0, 0)
            return new_x, new_y, dx, dy, new_angle, braquage
        except IndexError:
            raise StopIteration("Fin de la tentacule")


class Tentacule_Circulaire(_Tentacule):
    """
    Objet tentacule en forme de cercle, dépendant d'une voiture, et caractérisé par son ordre et son angle de braquage.
    """

    def __init__(self, voiture, k, delta_k, axes):
        super().__init__(axes, voiture)

        self.numero = k
        self.braquage = delta_k

        self.L1 = 8 + 72 * (self.voiture.vitesse/self.voiture.vitesse_max)**1.2  # formule empirique, équation (9.5) p.123
        # formules pour la longueur p.123 équation (9.4)
        # formules pour le rayon de courbure p.122 équation (9.3)

        if 1 <= k <= 40:  # tentacule à orienter vers la droite
            self.longueur = self.L1 + 20 * np.sqrt((k-1) / 40)
            self.courbure = -np.tan(delta_k) / self.voiture.longueur

            self.rayon = 1 / self.courbure
            self.angle_rad = self.longueur / self.rayon
            self.angle = self.angle_rad * 180 / np.pi

            # generation de l'arc de cercle
            self.angle_debut = 180 - self.angle
            self.angle_fin = 180
            centre_x = self.voiture.x + self.rayon / 2

            self.signe = -1  # parcours de la tentacule dans le sens inverse, voir Voiture.update_tentacule()

        elif k == 41:
            self.longueur = self.L1 + 20 * np.sqrt((k-1) / 40)
            self.courbure = -np.tan(delta_k) / self.voiture.longueur

            self.rayon = 1 / self.courbure
            self.angle_rad = self.longueur / self.rayon
            self.angle = self.angle_rad * 180 / np.pi

            self.angle_debut = 180
            self.angle_fin = 180 - self.angle
            centre_x = self.voiture.x + self.rayon / 2

            self.signe = 1  # parcours de la tentacule dans le bon sens, voir Voiture.update_tentacule()

        else:  # 42<=k<=81   # tentacule à orienter vers la gauche
            self.longueur = self.L1 + 20 * np.sqrt((81-k) / 40)
            self.courbure = np.tan(delta_k) / voiture.longueur

            self.rayon = 1 / self.courbure
            self.angle_rad = self.longueur / self.rayon  # en radian
            self.angle = self.angle_rad * 180 / np.pi  # en degrés

            self.angle_debut = 0
            self.angle_fin = self.angle
            centre_x = self.voiture.x - self.rayon / 2

            self.signe = 1  # parcours de la tentacule dans le bon sens, voir Voiture.update_tentacule()

        # On créé le dessin de la tentacule comme si la voiture était orientée vers le Nord
        self.patch = matplotlib.patches.Arc((centre_x, self.voiture.y), self.rayon, self.rayon,
                                            theta1=self.angle_debut, theta2=self.angle_fin, color='pink')
        # On applique la rotation de la voiture au dessin de la tentacule
        self.patch.set_transform(matplotlib.transforms.Affine2D().rotate_deg_around(self.voiture.x,
                                                                                    self.voiture.y,
                                                                                    self.voiture.angle - 90)
                                 + self.axes.transData)

        self.V_lisse = abs(self.braquage - self.voiture.braquage) / (2 * self.voiture.braquage_max)

    def get_parametric(self):
        """
        Permet d'obtenir la forme paramétrée de la tentacule. Modifie le patch en définissant un nombre de points explicite, et récupère les coordonnées.

        Returns
        -------
        chemin : numpy.ndarray
            Abscisses et ordonnées de la tentacule.
            Ex: numpy.array([[ x1,   y1],
                             [ x2,   y2],
                             [ x3,   y3]])

        """
        # Définition d'un arc avec un nombre de points contrôlé
        self.patch._path = matplotlib.patches.Path.arc(self.angle_debut, self.angle_fin, self.precision)
        # Paramétrisation de la tentacule
        chemin = (self.patch.get_transform() - axes_simu.transData).transform_path(self.patch.get_path()).cleaned().vertices[:-1][::self.signe]
        return chemin

    def __iter__(self):
        """
        Implémente l'itérabilité.
        """
        self.chemin = self.get_parametric()
        return self

    def __next__(self):
        """
        Passe à l'élément suivant (la tentacule étant un itérable) : met à jour la forme paramétrée, et retourne les nouveaux états de la voiture.

        Raises
        ------
        StopIteration
            Si la tentacule est parcourue en entier.

        Returns
        -------
        new_x : float
            Nouvelle abscisse.
        new_y : float
            Nouvelle ordonnée.
        dx : float
            Déplacement effectué en x.
        dy : float
            Déplacement effectué en y.
        new_angle : numpy.float64
            Nouvel angle de la voiture.
        braquage : numpy.float64 ou celui de la liste de braquage passée en paramètre lors de la définition de la tentacule.
            Nouveau braquage de la voiture.

        """
        try:
            new_x = self.chemin[1][0]
            new_y = self.chemin[1][1]
            dx = new_x - self.chemin[0][0]
            dy = new_y - self.chemin[0][1]
            new_angle = np.degrees(np.arctan2(dy, dx))
            braquage = self.braquage
            self.chemin = np.delete(self.chemin, 0, 0)
            return new_x, new_y, dx, dy, new_angle, braquage
        except IndexError:
            raise StopIteration("Fin de la tentacule")


class Voiture:
    """
    Objet voiture, défini par sa forme. D'autres paramètres fixes sont définis après, on pourra les mettre en paramètre plus tard si plusieurs voitures il y a.
    """

    def __init__(self, shape_voiture, axes, type_tentac=Tentacule_Circulaire, nombre_tentac=80):
        """
        Parameters
        ----------
        shape_voiture : matplotlib.patches.Rectangle
            Forme de la voiture.
        axes : matplotlib.axes._axes.Axes
            Axes dans lesquels la voiture sera affichée.
        type_tentac : __main__._Tentacule, optional
            Le type de tentacule qui vont être générées par la voiture. The default is Tentacule_Circulaire.
        nombre_tentac : int, optional
            Nombre de tentacules à générer. The default is 80.

        Returns
        -------
        None.

        """

        self.shape_voiture = shape_voiture
        self.shape_voiture.rotation_point = 'center'

        self.shapely_poly = shapely.geometry.Polygon((self.shape_voiture.get_transform()).transform_path(self.shape_voiture.get_path()).vertices)

        self.longueur = self.shape_voiture.get_height()
        self.largeur = self.shape_voiture.get_width()
        self.vitesse_max = VITESSE_MAX_VOIT
        self.accel_lat_max = ACCEL_LAT_MAX

        self.vitesse = VITESSE_INIT_VOIT
        self.image_vitesse = self.vitesse
        self.x, self.y = self.shape_voiture.get_center()
        self.angle = INCLINAISON_VOITURE_INIT
        self.braquage = 0
        self.distance_collision = self.vitesse**2 / 2*self.accel_lat_max
        self.courbure_max = self.accel_lat_max / self.vitesse ** 2  # formule p.122 équation (9.1)
        self.braquage_max = np.arctan(self.longueur * self.courbure_max)  # formule p.122 équation (9.2)

        self.type_tentac = type_tentac
        self.nombre_tentac = nombre_tentac
        self.tentacules = []

        self.chemin = iter(())  # Chemin que la voiture a à suivre. Iterable d'une tentacule, cf update_chemin

        self.tps_debut = time.monotonic()

        self.axes_simu, self.axes_vitesse, self.axes_braquage = axes

        self.traceur = PETIT_POUCET
        self.trace = matplotlib.lines.Line2D([], [])
        self.courbe_vitesse = matplotlib.lines.Line2D([time.monotonic() - self.tps_debut], [VITESSE_INIT_VOIT])
        self.courbe_braquage = matplotlib.lines.Line2D([time.monotonic() - self.tps_debut], [BRAQUAGE_VOLANT_INIT])
        self.axes_simu.add_line(self.trace)
        self.axes_vitesse.add_line(self.courbe_vitesse)
        self.axes_braquage.add_line(self.courbe_braquage)
        self.axes_vitesse.set_ylim(0, self.vitesse_max)
        self.axes_braquage.set_ylim(-self.braquage_max, self.braquage_max)

    def collision(self, objet):
        """
        Si la voiture cogne l'objet passé en argument, la simulation est mise en pause.

        Parameters
        ----------
        objet : shapely.geometry.polygon.Polygon
            Forme de l'objet (obstacle ou voiture) issue du module shapely.

        Returns
        -------
        None.

        """
        if self.shapely_poly.intersects(objet) is True:
            print("COLLISION VOITURE ", id(self))
            if SECURITE is True:
                print("Arrêt de la simulation")
                self.simu.pause_replay()

    def deplace(self):
        """
        S'il existe un chemin (itérable de la tentacule suivie), déplace la voiture suivant ce chemin et met à jour les données.

        Raises
        ------
        erreur
            Le chemin à suivre n'est pas défini ou est terminé.

        Returns
        -------
        None.

        """
        try:
            self.x, self.y, dx, dy, self.angle, self.braquage = next(self.chemin)
            tp = time.monotonic() - self.tps_debut
            dt = tp - self.courbe_vitesse.get_data()[0][-1]
            tps = np.append(self.courbe_vitesse.get_data()[0], tp)
            # La "vitesse" ne change pas car normalement le véhicule garde une vitesse à peu près constante. On peut obtenir (tracé dans les datas) la vitesse du véhicule sur la simulation,
            # qui nous indique si ce qu'on voit est fidèle à la réalité ou non (temps de latence trop grands et déplacements chaotiques par accoups)

            self.shape_voiture.set(x=self.x - self.largeur/2, y=self.y - self.longueur/2, angle=self.angle)
            self.shapely_poly = shapely.geometry.Polygon((self.shape_voiture.get_transform() - self.axes_simu.transData).transform_path(self.shape_voiture.get_path()).vertices)

            if self.traceur is True:
                self.trace.set_data(np.append(self.trace.get_data()[0], self.x), np.append(self.trace.get_data()[1], self.y))
            self.courbe_vitesse.set_data(tps, np.append(self.courbe_vitesse.get_data()[1], (dx**2 + dy**2)**0.5 / dt))
            self.courbe_braquage.set_data(tps, np.append(self.courbe_braquage.get_data()[1], self.braquage))
            self.axes_braquage.set_xlim(right=tp + 10)

        except StopIteration:
            print("TENTACULE ÉPUISÉE, ON NE SAIT PLUS QUEL CHEMIN SUIVRE")
            print("Mise à jour forcée du chemin à suivre")
            print("~ appel du gestionnaire de tentacules et coloration en marron ~")
            self.simu.gestion_tentac(self)
            for tentac in self.tentacules:
                tentac.patch.set_linestyle(":")

    def freiner(self):
        """
        Freine la voiture : divise la vitesse par 2, simulant un coup de frein (vitesse moyenne de 35km/h).
        Arrête la simulation si la vitesse est <1m/s (on cale).

        Returns
        -------
        None.

        """
        self.tentacule_suivie = self.choix_tentacule(self.tentacules)  # TODO: modifier paramètre lambda avec critère dégagement de tentacule seulement (pas braquage)
        self.tentacule_suivie.patch.set(color="black", linewidth=5)
        self.chemin = iter(self.tentacule_suivie)
        self.vitesse /= 2
        if self.vitesse < 0.5:
            print("Tu as calé bg")
            self.simu.pause_replay(stop=True)

    def update_chemin(self):
        """
        Supprime les tentacules actuelles et en génère de nouvelles, puis met à jour le chemin que la voiture doit emprunter.

        Returns
        -------
        None.

        """
        self.remove_tentacules()
        self.generation_tentacules()

        tentacules_navigables = self.get_tentacules_navigables_et_lisses(self.simu.obstacles_fixes + self.simu.obstacles_mvt + self.simu.voitures)
        if tentacules_navigables == []:  # les tentacules du milieu ne sont pas navigables : on va chercher les tentacules extrémales
            print("Les tentacules lisses ne sont pas navigables")
            tentacules_navigables = self.get_all_tentacules_navigables(self.simu.obstacles_fixes + self.simu.obstacles_mvt + self.simu.voitures)

        try:
            tentacule_a_suivre = self.choix_tentacule(tentacules_navigables)

        except ValueError as err:  # Pas de tentacule navigable
            print(err)
            print("Freinage !")
            self.freiner()

        else:
            tentacule_a_suivre.patch.set_color("purple")
            tentacule_a_suivre.patch.set_linewidth(3)
            self.tentacule_suivie = tentacule_a_suivre
            self.chemin = iter(tentacule_a_suivre)

    def generation_tentacules(self):
        """
        Génère les 80 tentacules que la voiture aura la possibilité d'emprunter.

        Returns
        -------
        None.

        """
        if self.type_tentac == Tentacule_Circulaire:
            self.tentacules = []
            deltas = np.linspace(-self.braquage_max, self.braquage_max, 80)  # 80 est un nombre empirique de tentacules défini dans la thèse (il est bon compromis entre précision et temps de calcul)
            for k in range(1, 81):
                self.tentacules.append(self.type_tentac(self, k, deltas[k-1], self.axes_simu))  # Ajout de la tentacule aux tentacules appartenant au véhicule
            for k in range(80):
                self.axes_simu.add_patch(self.tentacules[k].patch)  # Ajout de la tentacule à la figure
        elif self.type_tentac == Tentacule_clothoïdaire:
            self.tentacules = []
            ro_max = self.accel_lat_max / self.vitesse**2
            courbure_tentac_init = self.braquage * np.pi / 180
            rap_min = 2*(-ro_max - courbure_tentac_init) / self.distance_collision
            rap_max = 2*(+ro_max - courbure_tentac_init) / self.distance_collision
            deltas = np.linspace(rap_min, rap_max, 40)
            for rap in deltas:
                self.tentacules.append(Tentacule_clothoïdaire(self, courbure_tentac_init, rap, self.axes_simu))
            for tentacule in self.tentacules:
                self.axes_simu.add_line(tentacule.patch)
        else:
            pass  # TODO: compléter avec tentac inconnue

    def remove_tentacules(self):
        """
        Enlève les tentacules (qui appartiennent au véhicule) de la figure.

        Returns
        -------
        None.

        """
        for tentacule in self.tentacules:
            tentacule.patch.set(visible=False)  # car si simple remove, on perd les axes_simu de référence et c pas bon pour le blit...

    def get_espace_collision(self):
        """
        Génère et stocke une zone de support pour le véhicule (distance de sécurité autour de la voiture pour éviter quelconque collision).
        cf von Hundelshausen et al 2008 p.650

        Returns
        -------
        shapely.geometry.polygon.Polygon
            La zone de support évitant les collisions.

        """
        self.distance_collision = self.vitesse**2 / 2*self.accel_lat_max
        self.espace_collision = shapely.geometry.Point((self.x, self.y)).buffer(self.distance_collision + self.longueur/2)
        return self.espace_collision

    def is_tentacule_navigable(self, tentacule, objets_alentours):
        """
        Teste si la tentacule spécifiée est navigable (cogne des objets alentours dans le périmètre de sécurité). Méthode non utilisée.

        Parameters
        ----------
        tentacule : __main__._Tentacule
            Tentacule à tester.
        objets_alentours : __main__.Voiture ou __main__.Obstacle
            Objets à éviter.

        Returns
        -------
        bool
            True si la tentacule est navigable, False sinon.

        """
        return tentacule.is_navigable(objets_alentours)

    def choix_tentacule(self, tentacules_navigables, ponderation_meilleure_tentac=lambda tentac: tentac.V_lisse**4 + tentac.braquage**2):
        """
        Choisi une tentacule à suivre.

        Parameters
        ----------
        tentacules_navigables : list de __main__._Tentacule
            Tentacules dispo à trier pour choisir.
        ponderation_meilleure_tentac : callable, optional
            Fonction de tri. The default is lambda tentac: tentac.V_lisse**4 + tentac.braquage**2.

        Raises
        ------
        ValueError
            S'il n'y a aucune tentacule navigable (tentacules_navigables = [] par exemple).

        Returns
        -------
        __main__._Tentacule
            Tentacule à suivre (meilleure parmi celles données en argument).

        """
        # TODO : faire un critère de dégagement du tentacule (p. 125 section 9.2.4.1)
        # TODO moins important car nécessite GPS et tt (pas forcément notre objet d'étude) : faire un critère de rapprochement de la trajectoire globale (p.126 section 9.2.4.3)

        try:
            self.tentacule_suivie = min(tentacules_navigables, key=ponderation_meilleure_tentac)  # On trie selon une loi empirique fonction de V_lisse (écart au braquage actuel) et de braquage (trajectoire droite). Puis on prend le min
            return self.tentacule_suivie
        except ValueError:
            raise ValueError("AUCUNE TENTACULE NAVIGABLE")

    def get_all_tentacules_navigables(self, objets_alentours):
        """
        Permet d'obtenir toutes les tentacules navigables.

        Parameters
        ----------
        objets_alentours : __main__.Voiture ou __main__.Obstacle
            Objets à éviter.

        Returns
        -------
        tentacules_navigables : list of __main__._Tentacule
            Liste des tentacules navigables et que la voiture peut emprunter.

        """
        tentacules_navigables = []
        for tentacule in self.tentacules:
            if tentacule.is_navigable(objets_alentours):
                tentacule.patch.set_color('g')
                tentacules_navigables.append(tentacule)
        return tentacules_navigables

    def _simu(self, simu):
        """
        À ne pas appeler directement. Permet de définir la simulation dans laquelle évolue la voiture.

        Parameters
        ----------
        simu : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        self.simu = simu

    def get_tentacules_navigables_et_lisses(self, objets_alentours):
        """
        Permet d'obtenir toutes les tentacules navigables et 'lisses' ie dont la variation de braquage demandée n'est pas trop forte (<CONE_BRAQUAGE).

        Parameters
        ----------
        objets_alentours : __main__.Voiture ou __main__.Obstacle
            Objets à éviter.

        Returns
        -------
        tentacules_navigables : list of __main__._Tentacule
            Liste des tentacules navigables et que la voiture peut emprunter.

        """
        tentacules_lisses_et_navigables = []
        for tentacule in [tentac for tentac in self.tentacules if abs(tentac.braquage - self.braquage) < CONE_BRAQUAGE]:
            if tentacule.is_navigable(objets_alentours):
                tentacule.patch.set_color('g')
                tentacules_lisses_et_navigables.append(tentacule)
        return tentacules_lisses_et_navigables


class Obstacle():
    """
    Instance commune à tous type d'obstacle. Réunit des méthodes et attributs universels. Ne pas appeler directement, est automatiquement appelée par d'autres instances.
    """

    def __init__(self, patch):
        """
        Parameters
        ----------
        patch : matplotlib.patches.Rectangle
            Forme de l'obstacle.

        Returns
        -------
        None.

        """
        self.patch = patch
        self.x, self.y = self.patch.get_center()
        self.shapely_poly = shapely.geometry.Polygon((self.patch.get_transform()).transform_path(self.patch.get_path()).vertices)


class Obstacle_fixe(Obstacle):
    """
    Instance pour un obstacle qui ne bougera pas au cours du temps.
    """

    def __init__(self, patch):
        """
        Parameters
        ----------
        patch : matplotlib.patches.Rectangle
            Forme de l'obstacle.

        Returns
        -------
        None.

        """
        super().__init__(patch)


class Obstacle_mvt():
    """
    Instance pour un obstacle qui est amené à bouger au cours du temps. Fonctionnalité encore non développée correctement.
    """

    def __init__(self):
        pass


class Simulation_Ref_Terre:
    """
    Classe qui gère tout l'aspect simulation : animation + gestion des objets.

    Parameters
    ----------
    voitures: list
        Liste des voitures présentes
    obstaces: list  # TODO
        liste des obstacles FIXES. Ils ne seront pas animés
    obstacles_mvt: list  # TODO
        liste des obstacles à déplacer. Fonction dévelopée plus tard
    """

    def __init__(self, voitures=[], obstacles_fixes=[], obstacles_mvt=[], animations=[], timers=[]):
        """
        Parameters
        ----------
        voitures : list of __main__.Voiture, optional
            Voitures dans la simulation. The default is [].
        obstacles_fixes : list of __main__.Obstacle_fixe, optional
            Obstacles fixes dans la simulation. The default is [].
        obstacles_mvt : list of __main__.Obstacle_mvt, optional
            Obstacles en mouvement dans la simulation. The default is [].
        animations : list of matplotlib.animation.FuncAnimation, optional
            Animations qui gèrent l'affichage et l'interaction. The default is [].
        timers : list of matplotlib.backends.backend_qt.TimerQT, optional
            Timers qui gèrent l'animation et l'interaction. The default is [].

        Returns
        -------
        None.

        """
        self.paused = False

        self.animations = animations
        self.timers = timers

        self.voitures = voitures
        self.obstacles_fixes = obstacles_fixes
        self.obstacles_mvt = obstacles_mvt
        self.new_tentacules = []  # liste des tentacules à afficher (blit)
        self.old_tentacules = []  # liste des tentacules qui ne sont plus d'actualité, mais qui doivent être supprimées (blit)

        for voitures in self.voitures:
            voiture._simu(self)
            self.gestion_tentac(voiture)  # créé les tentacules de la voiture, et créé un chemin à suivre (surement inexistants jusqu'alors...)

        self.commentaires = COMMENTAIRES

    def animer(self, i):
        """
        Anime la figure. S'occupe de déplacer les voitures et les obstacles.
        Note : fait appel au "task-manager des tentacules" pour savoir quoi actualiser dans la figure (blit)

        Parameters
        ----------
        i : int
            Numéro d'appel de la fonction (elle est censée être appelée itérativement par une matplotlib.animation.

        Returns
        -------
        list of artists
            Liste des dessins à actualiser (blit).

        """
        # Déplace les voitures
        for voiture in self.voitures:
            voiture.deplace()
        # Déplace les obstacles
        for obstacle in self.obstacles_mvt:
            obstacle.deplace()
        try:
            # Retourne la liste des dessins à actualiser dans la figure
            # TODO : ajouter les obstacles lors de l'import de la fonctionnalité
            return [item for item in [voiture.shape_voiture, voiture.trace, voiture.courbe_vitesse, voiture.courbe_braquage] for voiture in self.voitures] + [objet.patch for objet in self.old_tentacules + self.new_tentacules + self.obstacles_fixes + self.obstacles_mvt]
        finally:
            # Met juste après les valeurs adéquates à jour pour ne plus avoir à redessiner des dessins fixes (blit)
            self.old_tentacules = []  # Suppression des tentacules obsoletes, qu'on a bien effacé avec le return
            for tentac in self.old_tentacules:
                tentac.patch.remove()
            for tentac in self.new_tentacules:
                tentac.patch.set_animated(False)  # Dés-anime les nouvelles tentacules (affichées avec le return) : elles ne doivent plus bouger jusqu'à une redfinition de celles-ci. Sert ?

    def gestion_tentac(self, voiture):
        """
        "Task-manager des tentacules"... Gère l'ajout et la suppression des tentacules (appartenant à la voiture *voiture*) de la figure.

        Parameters
        ----------
        voiture : Voiture
            objet voiture.

        Returns
        -------
        None.

        """
        for tentac in voiture.tentacules:  # pour mettre à jour et enlever les tentacules qui vont être supprimées
            tentac.patch.set_animated(True)  # Anime les tentacules à supprimer (elles étaient dés-animées) (blit)
            self.old_tentacules.append(tentac)  # ajout à la liste des dessins à actualiser (blit)

        voiture.update_chemin()  # suppression du système actuel de tentacules pour la voiture, puis génération de nouvelles tentacules pour la voiture

        for tentac in voiture.tentacules:  # pour mettre à jour et ajouter les tentacules qui vont être ajoutées
            tentac.patch.set_animated(True)  # anime les tentacules à ajouter (blit)
            self.new_tentacules.append(tentac)  # ajout à la liste des dessins à actualiser (blit)

    def collisions(self):
        """
        Teste les collisions pour chaque voiture.

        Returns
        -------
        None.

        """
        for voiture in self.voitures:
            for obstacle in self.obstacles_fixes:
                voiture.collision(obstacle.shapely_poly)
            for obstacle in self.obstacles_mvt:
                voiture.collision(obstacle.shapely_poly)
            for voit in self.voitures:
                if voit != voiture:  # pas de collision qd la voiture se touche elle-même
                    voiture.collision(voit.shapely_poly)

    def pause_replay(self, stop=False):
        """
        Met en pause ou remet en play.

        Returns
        -------
        None.

        """
        if self.paused:
            print("PLAY")
            for anim in self.animations:
                anim.resume()
            for timer in self.timers:
                timer.start()
            self.paused = False
        else:
            print("PAUSE")
            for anim in self.animations:
                anim.pause()
            for timer in self.timers:
                timer.stop()
                """
            if stop is False:
                self.paused = True  # Si juste une pause on le fait savoir, si c'est un stop, on fait croire que c'est encore la simu (pour que qd on replay ça replay pas)
                """
            self.paused = True

    def ajout_anim(self, anim):
        """
        Ajoute une animation.

        Parameters
        ----------
        anim : matplotlib.animation.FuncAnimation
            Animation à ajouter.

        Returns
        -------
        None.

        """
        self.animations.append(anim)

    def ajout_timer(self, timer):
        """
        Ajoute un timer.

        Parameters
        ----------
        timer : matplotlib.backends.backend_qt.TimerQT
            Timer à ajouter.

        Returns
        -------
        None.

        """
        self.timers.append(timer)


# création figure + axes
figure_ref_terre, axes_simu = plt.subplots()
figure_data, (axes_vitesse, axes_braquage) = plt.subplots(2, 1, sharex=True)
# titre
figure_ref_terre.suptitle("Animation")
figure_data.suptitle("Données en temps réel")
axes_vitesse.set_title("Vitesse")
axes_braquage.set_title("Braquage")
axes_braquage.set_xlabel("Temps")

# création de la forme de la voiture
shape_voiture = matplotlib.patches.Rectangle(POSITION_VOITURE_INIT, LONGUEUR_VEHIC, LARGEUR_VEHIC, fill=False, zorder=float('inf'))
shape_voiture.rotation_point = 'center'
shape_voiture.set_angle(INCLINAISON_VOITURE_INIT)

# création de la voiture
voiture = Voiture(shape_voiture, [axes_simu, axes_vitesse, axes_braquage], Tentacule_clothoïdaire)

# ajout du dessin de la voiture à la figure
axes_simu.add_patch(voiture.shape_voiture)

# création de la forme de l'obstacle
shape_obstacle = matplotlib.patches.Rectangle((30, -2.5), 2, 1.5, fill=True, zorder=float('inf'), color='cyan')
obstacle1 = Obstacle_fixe(shape_obstacle)
axes_simu.add_patch(obstacle1.patch)

# création des bords de route
shape_route_haut = matplotlib.patches.Rectangle((-10, 5), 90, 2)
# shape_route_milieu = matplotlib.patches.Rectangle((-30, 0), 190, 2)
shape_route_bas = matplotlib.patches.Rectangle((-10, -5), 90, 2)
route_haut = Obstacle_fixe(shape_route_haut)
# route_milieu = Obstacle_fixe(shape_route_milieu)
route_bas = Obstacle_fixe(shape_route_bas)
axes_simu.add_patch(route_bas.patch)
# axes_simu.add_patch(route_milieu.patch)
axes_simu.add_patch(route_haut.patch)


# création de la simulation
simulation = Simulation_Ref_Terre([voiture], [obstacle1, route_bas, route_haut])

# création et lancement de l'animation
anim = matplotlib.animation.FuncAnimation(figure_ref_terre,
                                          simulation.animer,
                                          init_func=lambda: [voiture.shape_voiture for voiture in simulation.voitures] + [tentacule.patch for tentacule in simulation.new_tentacules] + [tentacule.patch for tentacule in simulation.old_tentacules],
                                          fargs=[],
                                          interval=80,
                                          cache_frame_data=False,
                                          blit=True,
                                          repeat=True)

# création d'un timer
timer = figure_ref_terre.canvas.new_timer(interval=TEMPS_REAC)
if SECURITE is not False:
    timer2 = figure_ref_terre.canvas.new_timer(40)
    timer2.add_callback(simulation.collisions)
    timer2.start()
    simulation.ajout_timer(timer2)
    figure_ref_terre.canvas.mpl_connect('close_event', lambda event: timer2.stop())
# ajout d'une fonction appelée itérativement
timer.add_callback(simulation.gestion_tentac, voiture)
# démarrage du timer
timer.start()

simulation.ajout_anim(anim)
simulation.ajout_timer(timer)
# stop le timer si on ferme la figure (pour éviter qu'une fonction continue de tourner en boucle sans intérêt une fois tout terminé, c'est plutôt mal vu...!)
figure_ref_terre.canvas.mpl_connect('close_event', lambda event: timer.stop())
figure_ref_terre.canvas.mpl_connect('key_press_event', lambda event: (print("Key Pressed"), simulation.pause_replay()))

axes_simu.autoscale()
# axes_simu.set(xlim=(-50, 50), ylim=(-50, 150))
axes_simu.axis('equal')
plt.show()
