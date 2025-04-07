# LQR-control-algorithm
L’objectif de ce projet  est de contrôler le robot Niryo avec l’algorithme du LQR. L’algorithme LQR fait une optimisation de trajectoires en utilisant le modèle cinématique direct et inverse. Il utilise deux critères de coût Q et R, respectivement la distance euclidienne à un but désiré d, et l’effort pour l’atteindre dépendant des actions u.

J² = SUM A Q² + B R²
A = distance euclidienne sqrt(power(x-d, 2)) ; x position actuelle d position désirée
B = sum u² ; somme des actions
La trajectoire optimale est celle qui minimise J tel que J* = min J
