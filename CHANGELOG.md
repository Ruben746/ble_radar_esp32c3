# Historique

## 1.0.2 — Carte et témoin BLE

- Bannière corrigée à partir de la photo de la SuperMini fournie, incluse dans les deux README.
- LED intégrée activée sur GPIO8 avec polarité active LOW, éteinte au démarrage.
- Flash non bloquant de 80 ms au démarrage accepté du scan puis chaque seconde de scan continu actif, indépendamment de la liste de surveillance.
- Ancien déclenchement par observation d’un appareil surveillé supprimé pour éviter un allumage quasi continu en présence de nombreux paquets.
- Documentation bilingue du GPIO, de la polarité et de la signification du clignotement.

## 1.0.1 — Mot de passe AP modifiable

- Valeur AP initiale remplacée par `ChangeMe123!`, générique et volontairement publique.
- Champs de saisie et confirmation ajoutés dans Paramètres → Réseau, avec les styles existants.
- Validation côté navigateur et serveur : 8 à 63 caractères ASCII imprimables ; vide = inchangé.
- Sauvegarde en NVS vérifiée avant confirmation, priorité au mot de passe sauvegardé au démarrage et redémarrage différé après modification effective.
- README principal en anglais et version française liée.

## 1.0.0 — Préparation à la publication

- Import du firmware fourni, avec interface Web et logique de détection conservées.
- Inspection des valeurs sensibles ; maintien du mécanisme de mot de passe AP par défaut à la demande de l’auteur (valeur remplacée en 1.0.1).
- Cible documentée corrigée vers Arduino-ESP32 3.3.7 / NimBLE, conforme aux API BLE utilisées, au lieu de la mention initiale 3.2.0 / Bluedroid.
- Message de fin de configuration corrigé pour indiquer l’accès par IP : mDNS est désactivé par défaut.
- Ajout du README, de la licence MIT, des exclusions Git et de la documentation des limites de sécurité et de validation.
- Aucune modification de la disposition des données NVS ni des réglages de détection. Le mode TLS Telegram d’origine est conservé et sa limite documentée.
