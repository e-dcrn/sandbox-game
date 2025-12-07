#!/bin/bash

# Arrêter le script si une commande échoue
set -e

# Couleurs pour l'affichage
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Initialisation des dépendances du Moteur ===${NC}"

# 1. Vérification de l'emplacement
# On s'assure d'être dans le dossier où se trouve le script
cd "$(dirname "$0")"
echo -e "Dossier de travail : $(pwd)"

# ---------------------------------------------------------
# 2. Installation de SDL2 (Linux / Debian / Ubuntu)
# ---------------------------------------------------------
echo -e "\n${BLUE}[1/3] Vérification de SDL2...${NC}"

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if dpkg -l | grep -q libsdl2-dev; then
        echo -e "${GREEN}SDL2 est déjà installé.${NC}"
    else
        echo "Installation de SDL2 (nécessite sudo)..."
        sudo apt-get update
        sudo apt-get install -y libsdl2-dev
        echo -e "${GREEN}SDL2 installé avec succès.${NC}"
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "MacOS détecté. Assurez-vous d'avoir installé SDL2 via : brew install sdl2"
else
    echo "OS non géré automatiquement pour SDL2. Assurez-vous de l'avoir installé manuellement."
fi

# ---------------------------------------------------------
# 3. Clonage de EnTT (Header Only)
# ---------------------------------------------------------
echo -e "\n${BLUE}[2/3] Installation de EnTT...${NC}"

if [ -d "entt" ]; then
    echo -e "${GREEN}EnTT est déjà cloné.${NC}"
else
    echo "Clonage de EnTT..."
    # On clone seulement la branche master, profondeur 1 (plus rapide, pas d'historique)
    git clone --depth 1 https://github.com/skypjack/entt.git
    echo -e "${GREEN}EnTT cloné.${NC}"
fi

# ---------------------------------------------------------
# 4. Clonage de Dear ImGui
# ---------------------------------------------------------
echo -e "\n${BLUE}[3/3] Installation de ImGui...${NC}"

if [ -d "imgui" ]; then
    echo -e "${GREEN}ImGui est déjà cloné.${NC}"
else
    echo "Clonage de Dear ImGui (Docking branch pour plus de features)..."
    # La branche 'docking' est souvent recommandée pour les moteurs modernes
    git clone --depth 1 -b docking https://github.com/ocornut/imgui.git
    echo -e "${GREEN}ImGui cloné.${NC}"
fi

echo -e "\n${GREEN}=== Installation terminée ! ===${NC}"
echo "N'oubliez pas de mettre à jour votre Makefile pour inclure :"
echo " -Ilibs/entt/src"
echo " -Ilibs/imgui"
echo " -Ilibs/imgui/backends"