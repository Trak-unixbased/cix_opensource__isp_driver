# CIX OpenSource ISP Driver

## 📖 Description
Ce dépôt contient le code source du pilote noyau Linux (*kernel module*) pour le processeur d'image (ISP - *Image Signal Processor*) des SoC CIX Technology (ex: CIX P1 / CD8180). Il est conçu pour être compilé en tant que module hors-arbre (*out-of-tree*) sur des distributions basées sur Debian/Ubuntu.

## 🎯 À quoi ça sert ?
L'ISP gère la conversion et l'optimisation matérielle des flux vidéo. Les capteurs optiques envoient des données brutes (*Bayer/RAW*) à la carte mère. Ce pilote interagit avec l'accélérateur matériel ISP pour traiter ces données (correction des couleurs, exposition automatique, réduction de bruit) de façon transparente et très performante. Il expose ensuite le flux vidéo final au système d'exploitation via l'API standard **V4L2 (Video4Linux2)**, permettant la capture vidéo par des applications comme GStreamer ou FFmpeg.

## 📦 Contenu du dépôt
- **`*.c` / `*.h`** : Code source C de l'implémentation du pilote noyau.
- **`Makefile` / `Kbuild`** : Scripts liant le code aux en-têtes du noyau pour la compilation.
- **`Kconfig`** : Paramètres de configuration du module noyau.

---

## 🛠️ Instructions de Compilation

### 1. Prérequis
Assurez-vous d'avoir les outils de compilation et les en-têtes (*headers*) correspondant à la version actuelle de votre noyau Linux :
```bash
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r) bc
