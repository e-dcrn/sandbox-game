# --- Configuration du Compilateur ---
CXX      = g++
CXXFLAGS = -std=c++17 -g -Wall -Wextra

# --- Dossiers ---
BIN_DIR  = bin
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
# Tes sources
SOURCES  = $(SRC_DIR)/main.cpp

# Sources ImGui
IMGUI_DIR = $(LIBS_DIR)/imgui
SOURCES += $(IMGUI_DIR)/imgui.cpp \
           $(IMGUI_DIR)/imgui_demo.cpp \
           $(IMGUI_DIR)/imgui_draw.cpp \
           $(IMGUI_DIR)/imgui_tables.cpp \
           $(IMGUI_DIR)/imgui_widgets.cpp \
           $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp \
           $(IMGUI_DIR)/backends/imgui_impl_sdlrenderer2.cpp

# --- Objets ---
OBJECTS = $(SOURCES:.cpp=.o)

# --- Cible (Executable) ---
# L'exécutable sera maintenant bin/sandbox_app
TARGET  = $(BIN_DIR)/sandbox_app

# --- Règles ---

all: $(TARGET)

# Linkage (Création de l'exécutable)
$(TARGET): $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)
	@echo "✅ Build réussi ! Exécutable placé dans : $(TARGET)"

# Compilation (.cpp -> .o)
.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Nettoyage
clean:
	rm -f $(OBJECTS)
	rm -rf $(BIN_DIR)

# Lancer le jeu
run: all
	./$(TARGET)