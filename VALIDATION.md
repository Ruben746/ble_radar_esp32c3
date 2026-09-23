# Validation de la préparation

## Périmètre

Source : firmware transmis par l’auteur. Préparation du dépôt sans flashage de carte ni publication distante. Version livrée : `1.0.1`. Le format de stockage reste `CONFIG_VERSION = 1` ; la clé existante `appass` sert au mot de passe AP modifiable.

## Contrôles statiques

- Valeur de `AP_DEFAULT_PASS` remplacée par `ChangeMe123!` à la demande de l’auteur ; ancienne valeur absente des livrables.
- Identifiants Wi-Fi station et Telegram initialisés à vide ; aucun PIN personnel préconfiguré identifié.
- Recherche de signatures de tokens Telegram et de clés privées : aucun résultat.
- Comparaison avec l’original : marqueur de licence, cible documentée et texte d’accès corrigés ; ajout ciblé du changement de mot de passe AP, de son enregistrement et de sa priorité au démarrage. Logique BLE et alertes inchangées.
- Syntaxe des trois scripts JavaScript embarqués vérifiée avec Node.js.
- Onze scénarios exécutés sur le gestionnaire JavaScript réseau extrait du sketch : limites de longueur, caractères autorisés, confirmation, champs vides, annulation et transmission des paramètres. Ces essais isolés simulent les contrôles et l’appel réseau, sans exécuter un serveur ESP32.
- Vérification du chemin serveur : route protégée par authentification, validation avant écriture, contrôle du résultat de sauvegarde du mot de passe, réponse HTTP avant redémarrage différé. Aucun essai NVS sur matériel.
- Liens locaux des documents vérifiés.

## Compilation

Compilation locale réussie (code de sortie 0) du sketch final avec Arduino-ESP32 **3.3.7** et le profil `esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app`.

- Programme : **1 493 279 octets**, soit 47 % des 3 145 728 octets de la partition.
- Variables globales : **64 804 octets**, soit 19 % des 327 680 octets de mémoire dynamique annoncés par l’outil.
- Mémoire restante annoncée : **262 876 octets** avant allocations d’exécution ; ce chiffre n’est pas une mesure de heap en fonctionnement.

Archive vérifiée après création : huit fichiers autorisés uniquement, `.gitignore` inclus, intégrité ZIP et correspondance octet par octet avec les fichiers du dossier de livraison. Aucun binaire, log, export NVS ou source originale inclus. Empreinte SHA-256 fournie à côté du ZIP.

## Essais matériels

Non effectués. La compilation ne prouve pas la stabilité radio, la réception des alertes, le bon fonctionnement du point d’accès ou la tenue en fonctionnement continu. Effectuer les vérifications de première mise en service décrites dans le README.

## Sécurité

Inspection ciblée, sans audit exhaustif. Le mot de passe AP public, HTTP, le PIN court, le stockage NVS et le mode TLS Telegram sans vérification sont explicitement documentés dans SECURITY.md.
