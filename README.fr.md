# ESP32-C3 BLE Radar

[English](README.md) · **Français**

![ESP32-C3 BLE Radar](assets/ble-radar-presentation.png)

L’image `assets/ble-radar-presentation.png` sert uniquement à la présentation du dépôt sur GitHub. Elle n’est ni intégrée au sketch, ni compilée dans le firmware, ni téléversée sur l’ESP32.

Radar de présence BLE autonome pour ESP32-C3 SuperMini, avec interface Web en français, liste de surveillance persistante, estimation de proximité par RSSI et notifications Telegram.

Le firmware, le HTML, le CSS, le JavaScript et l’API sont regroupés dans `ble_radar_esp32c3.ino`. Aucun serveur externe n’est nécessaire pour le radar local ; Telegram nécessite Internet.

## Fonctionnalités

- Scan BLE actif et continu, nom annoncé, MAC, type d’adresse, RSSI brut et filtré, informations fabricant/service lorsqu’elles sont disponibles.
- Jusqu’à 50 appareils visibles et 20 appareils surveillés, avec noms personnalisés et réglages par appareil.
- Alertes d’apparition, de proximité et de disparition ; confirmations, temporisations et hystérésis configurables.
- Calibration à 1 mètre et estimation indicative de distance.
- Détection optionnelle des nouvelles adresses, mémoire de 200 MAC et limitation des alertes. Cette détection est désactivée par défaut ; les adresses aléatoires sont ignorées par défaut pour ces alertes.
- Configuration et liste de surveillance conservées en NVS ; journal circulaire de 30 événements en mémoire vive.
- Assistant de première configuration, PIN Web de quatre chiffres, sessions et limitation des tentatives de connexion.
- Connexion Wi-Fi, réseau AP de secours après environ deux minutes de perte de connexion, reconnexion automatique et mDNS optionnel.

## Matériel et versions

| Élément | Configuration cible |
| --- | --- |
| Carte | ESP32-C3 SuperMini ; sélectionner **ESP32C3 Dev Module** |
| Cœur Arduino | **esp32 by Espressif Systems 3.3.7** |
| Bluetooth | Bibliothèque BLE intégrée au cœur, pile NimBLE sur cette cible |
| Flash | 4 Mo si votre carte possède effectivement 4 Mo |
| Partition | **Huge APP (3MB No OTA/1MB SPIFFS)** |
| USB CDC On Boot | **Enabled** pour le port USB natif |
| Moniteur série | **115200 bauds** |
| Alimentation | Câble USB de données et alimentation stable |
| LED bleue intégrée | **GPIO8**, active à LOW (`LOW` = allumée, `HIGH` = éteinte) |

Aucun capteur, écran ou câblage externe n’est nécessaire. Le Wi-Fi doit être compatible 2,4 GHz. Vérifier les caractéristiques de votre variante de SuperMini.

La mention 3.2.0/Bluedroid de l’en-tête initial a été corrigée : le scan appelle directement les API GAP NimBLE. Ne pas installer une ancienne bibliothèque `ESP32 BLE Arduino` ou une bibliothèque NimBLE externe pour cette configuration. Voir [le code BLE du cœur 3.3.7](https://github.com/espressif/arduino-esp32/blob/3.3.7/libraries/BLE/src/BLEDevice.h).

## Témoin de recherche BLE

La SuperMini noire standard de la bannière possède une **LED utilisateur bleue sur GPIO8**, à logique inversée : **LOW l’allume et HIGH l’éteint**. Le voyant d’alimentation est distinct. Voir la [documentation de la carte](https://nuttx.incubator.apache.org/docs/latest/platforms/risc-v/esp32c3/boards/esp32c3-supermini/index.html).

Le sketch définit maintenant `LED_PIN` à **8**. La LED est éteinte à l’initialisation, émet un flash d’environ **80 ms** lorsqu’un démarrage de scan est accepté, puis clignote environ **une fois par seconde tant que le scan est actif**. Le scan étant continu (`BLE_HS_FOREVER`), ce clignotement indique son activité : il ne correspond pas à un nouveau scan chaque seconde. Il fonctionne même sans appareil détecté et s’arrête lorsque le scan est arrêté ou que son démarrage échoue. Après deux minutes sans annonce reçue, le mécanisme existant de relance peut interrompre brièvement le clignotement.

La temporisation est non bloquante : aucun `delay()` n’est ajouté au scan ou au serveur Web. La boucle principale éteint la LED après l’intervalle prévu ; une boucle chargée peut allonger un flash. Ce témoin indique l’activité logicielle du scan, pas la réception certaine d’un paquet ni l’envoi d’une alerte.

Pour désactiver le témoin, définir `LED_PIN` à `-1`. Ce réglage vise la SuperMini noire standard de la photo fournie ; certains clones diffèrent. GPIO8 est aussi une broche de sélection du mode de démarrage : ne pas ajouter de circuit externe qui la force à LOW pendant le reset.

## Installation avec Arduino IDE

1. Installer Arduino IDE et ajouter l’URL suivante aux URL supplémentaires du gestionnaire de cartes :
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Dans le gestionnaire de cartes, installer **esp32 by Espressif Systems**, version **3.3.7**.
3. Extraire l’archive. Conserver le nom de dossier `ble_radar_esp32c3`, identique au nom du sketch. Ouvrir `ble_radar_esp32c3.ino`.
4. Sélectionner **ESP32C3 Dev Module**, le port de la carte et les options du tableau ci-dessus. Laisser les autres réglages par défaut.
5. Vérifier/compiler puis téléverser. Si la carte ne passe pas en mode téléchargement, maintenir BOOT, appuyer brièvement sur RESET, relâcher BOOT, puis réessayer.
6. Ouvrir le moniteur série à 115200 bauds et redémarrer la carte pour lire les informations de connexion.

Les bibliothèques WiFi, WebServer, Preferences, ESPmDNS, HTTPClient, WiFiClientSecure et BLE sont incluses dans ce cœur. Aucune bibliothèque JSON supplémentaire n’est requise.

[Installation officielle du cœur Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).

### Compilation avec Arduino CLI

Depuis le répertoire parent du dossier `ble_radar_esp32c3` :

```sh
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.7 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app ble_radar_esp32c3
```

## Première configuration

1. Relever dans le moniteur série le SSID **ESP-C3-XXXX**. Le mot de passe AP initial est **`ChangeMe123!`** sur une installation sans mot de passe déjà enregistré. La constante `AP_DEFAULT_PASS` contient volontairement un mot de passe public commun aux installations ; voir la section sécurité.
2. Se connecter à ce réseau Wi-Fi et ouvrir manuellement **http://192.168.4.1**. Le firmware ne fournit pas de portail captif DNS.
3. Saisir le SSID et le mot de passe de votre Wi-Fi, puis créer et confirmer votre PIN à quatre chiffres.
4. Telegram est facultatif : laisser les deux champs vides pour l’omettre. Pour l’activer, créer un bot via le compte officiel BotFather de Telegram, démarrer une conversation avec ce bot et fournir son token et le Chat ID destinataire. Utiliser « Tester Telegram » pour vérifier les paramètres.
5. Enregistrer, attendre le redémarrage et rejoindre votre Wi-Fi habituel. Ouvrir l’adresse IP indiquée dans le moniteur série ou dans les baux DHCP du routeur.
6. S’authentifier avec le PIN, puis ajouter des appareils depuis le radar ou par leur MAC. Configurer les alertes dans la liste de surveillance.

`USE_MDNS` vaut **0** par défaut : `http://bleradar.local` n’est donc pas disponible. Pour activer mDNS, passer cette constante à `1`, recompiler et utiliser le nom configuré si votre réseau prend en charge mDNS.

Le réseau AP s’arrête lorsque la connexion Wi-Fi normale est établie. En secours, il utilise le même mot de passe AP et l’interface reste protégée par le PIN existant.

## Réglages et persistance

- Les identifiants Wi-Fi et Telegram sont saisis dans l’interface et enregistrés en NVS, pas dans le dépôt.
- Un champ de mot de passe Wi-Fi ou de token Telegram laissé vide dans les paramètres conserve sa valeur actuelle. Le token n’est pas renvoyé par l’API des paramètres.
- Le mot de passe AP se modifie dans **Paramètres → Réseau → Nouveau mot de passe du réseau de secours**. Saisir 8 à 63 caractères ASCII imprimables, confirmer puis cliquer sur **ENREGISTRER**. Les champs vides conservent le mot de passe actuel. Une modification effective provoque un redémarrage après enregistrement ; si vous êtes connecté au réseau de secours, reconnectez-vous avec le nouveau mot de passe. Le Wi-Fi domestique possède son propre champ, distinct.
- La valeur enregistrée en NVS est prioritaire sur `AP_DEFAULT_PASS` et survit au redémarrage comme au retéléversement normal. Modifier uniquement la constante ne remplace donc plus une valeur enregistrée. Une remise à zéro d’usine efface cette valeur et rétablit le défaut de compilation au prochain démarrage.
- Mettre `AP_DEFAULT_PASS` à `""` active le mécanisme existant de génération d’un mot de passe de 12 caractères, conservé en NVS. Un mot de passe valide déjà présent est réutilisé ; le reflasher ne renouvelle pas systématiquement les secrets.
- Les paramètres et la liste de surveillance survivent au redémarrage. Les événements, appareils visibles, sessions et files de notifications sont volatils. Les nouvelles MAC connues sont sauvegardées par lots ; les plus récentes peuvent être perdues après une coupure.
- La remise à zéro d’usine de l’interface efface les données du namespace applicatif `bleradar`. Elle ne constitue pas un effacement sécurisé de toute la flash. Si le PIN est perdu, un effacement complet de la flash avec les outils Espressif puis un nouveau téléversement permet de repartir de zéro, en perdant les données enregistrées.

## Limites de détection

Une adresse BLE n’est pas une identité permanente. Les téléphones et d’autres appareils emploient des MAC privées/aléatoires qui changent : un appareil surveillé peut apparaître absent et revenir sous une nouvelle adresse. Le firmware ne résout pas les identités privées et ne contourne pas ce mécanisme.

Seuls les appareils émettant des annonces BLE peuvent être observés ; le Bluetooth classique et les appareils silencieux ne sont pas détectés. Une absence d’annonce n’est pas une preuve d’absence physique. Le filtrage des MAC aléatoires ne garantit pas une classification parfaite.

Le RSSI dépend de l’orientation, des murs, des personnes, de l’antenne et de la puissance d’émission. La distance est une approximation, pas une mesure de position ou de direction. Calibrer à 1 mètre dans l’environnement d’usage améliore la cohérence sans supprimer ces erreurs.

Le BLE et le Wi-Fi partagent la radio. La latence des alertes dépend des annonces, des confirmations configurées, du réseau et de Telegram. La file Telegram est limitée à quatre messages et abandonne le plus ancien si elle est pleine ; aucune livraison garantie ni conservation durable hors connexion. Le firmware ne remplace pas une alarme de sécurité certifiée.

## Sécurité et usage légitime

**Le mot de passe AP par défaut est volontairement générique : `ChangeMe123!`. Il est public et ne doit pas être considéré comme secret.** Le changer dans l’interface dès la première configuration, ou utiliser le mode aléatoire. Le modifier ultérieurement n’efface pas les anciennes versions de l’historique Git.

L’interface fonctionne en **HTTP**, le PIN n’a que quatre chiffres, et le stockage NVS n’est pas explicitement chiffré par ce sketch. Les paramètres authentifiés exposent le Chat ID et le mot de passe AP. Le port série peut afficher des informations sensibles. Utiliser un réseau de confiance et ne pas exposer le port 80 sur Internet.

**Telegram utilise actuellement `client.setInsecure()` : le certificat du serveur n’est pas vérifié.** Le transport est chiffré mais l’identité du serveur n’est pas authentifiée, ce qui laisse un risque d’interception du token sur un réseau compromis. Ce comportement d’origine est conservé et ne doit pas être présenté comme du TLS pleinement vérifié. Voir [SECURITY.md](SECURITY.md).

Utiliser ce projet uniquement pour ses propres équipements ou avec l’autorisation des personnes concernées, dans le respect des règles applicables. Ne pas surveiller des personnes à leur insu. Les alertes Telegram peuvent transmettre des MAC, noms, horaires et estimations de distance : limiter les destinataires et ne pas publier ces données.

## Vérifications avant utilisation

Le statut de compilation est décrit dans [VALIDATION.md](VALIDATION.md). Aucun essai sur carte physique n’est inclus dans la préparation de cette archive.

Après téléversement, vérifier la première configuration, le scan d’un équipement autorisé, la conservation des paramètres après redémarrage, les trois types d’alertes avec un bot de test, la reconnexion Wi-Fi et le secours AP. Vérifier également la persistance du nouveau mot de passe AP et le clignotement de la LED bleue pendant le scan, y compris avec une liste de surveillance vide. Tester la remise à zéro uniquement après avoir accepté la perte de configuration.

## Publication sur GitHub

Créer un dépôt vide, puis y déposer **le contenu** du dossier extrait, y compris `.gitignore`. GitHub ne décompresse pas automatiquement un ZIP téléversé : extraire l’archive avant d’ajouter les fichiers. Le ZIP peut aussi être joint à une release.

Ne pas ajouter le firmware original, captures non anonymisées, exports NVS, journaux série, fichiers de configuration privés ou binaires contenant des secrets. `.gitignore` aide à éviter des ajouts accidentels mais ne retire rien d’un historique déjà publié.

## Licence

Le code de ce dépôt est distribué sous [licence MIT](LICENSE). Les bibliothèques et outils installés séparément conservent leurs propres licences ; ils ne sont pas inclus dans l’archive.
