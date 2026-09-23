# Sécurité

## Périmètre de la préparation

Le firmware fourni a été inspecté pour rechercher les identifiants Wi-Fi, secrets Telegram, Chat IDs, PIN, mot de passe AP et identifiants personnels préconfigurés. Aucun SSID privé, mot de passe Wi-Fi station, token Telegram, Chat ID ou PIN personnel codé en dur n’a été identifié. Le mot de passe AP est la seule valeur d’accès préconfigurée identifiée ; il est remplacé par le défaut générique public `ChangeMe123!`, modifiable dans les paramètres réseau. Il ne doit pas être considéré comme secret.

Cette inspection ne constitue pas un audit exhaustif ni une garantie d’absence de vulnérabilités.

## Limites connues

- HTTP sur le réseau local : les identifiants et cookies ne disposent pas d’une protection TLS applicative.
- PIN de quatre chiffres : dérivation SHA-256 salée et limitations de tentatives présentes, mais espace de recherche réduit ; ne protège pas contre l’extraction physique du stockage.
- Credentials stockés avec Preferences/NVS. Ce sketch ne configure ni chiffrement de flash, ni chiffrement NVS, ni Secure Boot.
- `setInsecure()` conservé pour Telegram : aucune validation du certificat distant. Un attaquant capable d’intercepter le réseau peut se faire passer pour Telegram et récupérer le token et les messages.
- Mot de passe AP connu par défaut, imprimé au premier démarrage et lors de certains diagnostics ; visible également via les paramètres après authentification.
- L’assistant initial n’a pas de PIN préalable. Toute personne ayant accès au réseau et à cet assistant avant configuration peut tenter de configurer l’appareil.
- Les contrôles de session et l’en-tête de requête ne remplacent pas une protection complète contre les attaques Web ou un réseau hostile.

## Pratiques de déploiement

1. Utiliser un réseau local de confiance, de préférence isolé, sans redirection Internet vers l’interface.
2. Changer le mot de passe AP dans Paramètres → Réseau avant utilisation réelle, ou activer le mode aléatoire avant la première initialisation. Ne pas réutiliser ce mot de passe sur d’autres services.
3. Protéger l’accès physique, le port série et les sauvegardes. Ne pas joindre de logs non anonymisés aux issues.
4. Pour un usage exigeant une authentification TLS, remplacer `setInsecure()` par une validation de chaîne et de nom d’hôte avec autorités de confiance, synchronisation de l’heure et procédure de renouvellement ; vérifier ce changement sur matériel.
5. En cas de fuite d’un token Telegram, le révoquer auprès de BotFather, le remplacer dans l’interface et supprimer les copies exposées. Un simple nouveau commit ne retire pas un secret de l’historique.
6. Avant cession de la carte, effacer ses données et considérer qu’une remise à zéro logique n’est pas un effacement sécurisé.

## Signalement

Ne pas publier de secrets ni de détails permettant une exploitation immédiate dans une issue publique. Si le dépôt propose un signalement privé GitHub, l’utiliser ; sinon demander au mainteneur un canal privé avec une description générale sans données sensibles.

Aucun service de support, délai de correctif ou programme de primes n’est promis.
