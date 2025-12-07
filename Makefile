# --- Configuration du Compilateur ---
CXX      = g++
# Drapeaux communs (Warnings + Version C++)
CXXFLAGS_COMMON = -std=c++17 -Wall -Wextra

# --- Configuration des Modes (Debug vs Release) ---
# Par défaut, on est en RELEASE (Rapide)
MODE ?= release

ifeq ($(MAKECMDGOALS),debug)
    MODE = debug
endif

# Configuration spécifique selon le mode
ifeq ($(MODE),debug)
    # Mode Debug : Symboles (-g), Pas d'opti (-O0)
    CXXFLAGS = $(CXXFLAGS_COMMON) -g -O0
    BUILD_DIR = build/debug
    BIN_DIR   = bin/debug
    TARGET_NAME = sandbox_debug
else
    # Mode Release : Optimisation Max (-O3), Pas d'asserts (-DNDEBUG)
    CXXFLAGS = $(CXXFLAGS_COMMON) -O3 -DNDEBUG
    BUILD_DIR = build/release
    BIN_DIR   = bin/release
    TARGET_NAME = sandbox_app
endif

# --- Dossiers ---
SRC_DIR  = src
LIBS_DIR = libs

# --- Chemins des Includes (-I) ---
INCLUDES = -I$(SRC_DIR) \
           -I$(LIBS_DIR)/imgui \
           -I$(LIBS_DIR)/imgui/backends \
           -I$(LIBS_DIR)/entt/src \
           $(shell sdl2-config --cflags)

# --- Librairies à lier (-l) ---
LIBS     = $(shell sdl2-config --libs) -ldl -lpthread

# --- Fichiers Sources ---
# Récupération récursive des .cpp dans src/
SOURCES  = $(shell find $(SRC_DIR) -name "*.cpp")

# Ajout manuel des sources ImGui
IMGUI_DIR = $(LIBS_DIR)/imgui
SOURCES += $(IMGUI_DIR)/imgui.cpp \
           $(IMGUI_DIR)/imgui_demo.cpp \
           $(IMGUI_DIR)/imgui_draw.cpp \
           $(IMGUI_DIR)/imgui_tables.cpp \
           $(IMGUI_DIR)/imgui_widgets.cpp \
           $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp \
           $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

# Ajout de la lib OpenGL système
LIBS += -lGL

# --- Objets ---
# On transforme les .cpp en .o et on les place dans le dossier de build spécifique
# Ex: src/main.cpp -> build/release/src/main.o
OBJECTS = $(SOURCES:%.cpp=$(BUILD_DIR)/%.o)

# --- Cible Finale ---
TARGET  = $(BIN_DIR)/$(TARGET_NAME)

# --- Règles ---

# Règle par défaut (lance la compilation release)
all: $(TARGET)

# Règle pour l'exécutable
$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)
	@echo "✅ Build [$(MODE)] réussi ! Exécutable : $(TARGET)"

# Règle générique pour compiler les .cpp en .o
# Le @mkdir -p $(dir $@) crée automatiquement les sous-dossiers (src, libs, etc.) dans build/
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# --- Commandes Utilisateur ---

# 1. Commande "make run" (Compile en Release et lance)
run: $(TARGET)
	@echo "🚀 Lancement en mode RELEASE..."
	./$(TARGET)

# 2. Commande "make debug" (Compile en Debug)
# On force la dépendance vers l'exécutable debug
debug: $(TARGET)
	@echo "🐞 Build DEBUG terminé."

# 3. Commande "make gdb" (Compile en Debug et lance GDB)
gdb: debug
	gdb ./$(TARGET)

# Nettoyage complet
clean:
	rm -rf build
	rm -rf bin
	@echo "🧹 Nettoyage effectué."

# Pour éviter les conflits avec des fichiers du même nom
.PHONY: all clean run debug gdb