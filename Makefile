# --- Configuration du Compilateur ---
CXX      = g++
CXXFLAGS_COMMON = -std=c++17 -Wall -Wextra

# --- Configuration des Modes ---
MODE ?= release

ifeq ($(MAKECMDGOALS),debug)
    MODE = debug
endif

# Définition explicite des chemins pour pouvoir nettoyer les deux modes
BUILD_ROOT    = build
BUILD_DEBUG   = build/debug
BUILD_RELEASE = build/release
BIN_DEBUG     = bin/debug
BIN_RELEASE   = bin/release

ifeq ($(MODE),debug)
    CXXFLAGS  = $(CXXFLAGS_COMMON) -g -O0
    BUILD_DIR = $(BUILD_DEBUG)
    BIN_DIR   = $(BIN_DEBUG)
    TARGET_NAME = sandbox_debug
else
    CXXFLAGS  = $(CXXFLAGS_COMMON) -O3 -DNDEBUG
    BUILD_DIR = $(BUILD_RELEASE)
    BIN_DIR   = $(BIN_RELEASE)
    TARGET_NAME = sandbox_app
endif

# --- Dossiers ---
SRC_DIR  = src
LIBS_DIR = libs

# --- Includes ---
INCLUDES = -I$(SRC_DIR) \
           -I$(LIBS_DIR)/imgui \
           -I$(LIBS_DIR)/imgui/backends \
           -I$(LIBS_DIR)/entt/src \
           $(shell sdl2-config --cflags)

# --- Librairies ---
LIBS     = $(shell sdl2-config --libs) -ldl -lpthread

# --- Sources ---

# 1. Points d'entrée
FILE_APP_MAIN  = src/main.cpp
FILE_TEST_MAIN = src/tests/main_test.cpp

# 2. ImGui
IMGUI_DIR = $(LIBS_DIR)/imgui
IMGUI_SRCS = $(IMGUI_DIR)/imgui.cpp \
             $(IMGUI_DIR)/imgui_demo.cpp \
             $(IMGUI_DIR)/imgui_draw.cpp \
             $(IMGUI_DIR)/imgui_tables.cpp \
             $(IMGUI_DIR)/imgui_widgets.cpp \
             $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp \
             $(IMGUI_DIR)/backends/imgui_impl_sdlrenderer2.cpp

# 3. Moteur (Tout src/ sauf les main)
ALL_SRC_FILES := $(shell find $(SRC_DIR) -name "*.cpp")
CORE_SRCS := $(filter-out $(FILE_APP_MAIN) $(FILE_TEST_MAIN), $(ALL_SRC_FILES))

# --- Objets ---
# Note: $(BUILD_DIR)/src/... contiendra ton code
#       $(BUILD_DIR)/libs/... contiendra imgui
APP_OBJS  = $(CORE_SRCS:%.cpp=$(BUILD_DIR)/%.o) \
            $(IMGUI_SRCS:%.cpp=$(BUILD_DIR)/%.o) \
            $(FILE_APP_MAIN:%.cpp=$(BUILD_DIR)/%.o)

TEST_OBJS = $(CORE_SRCS:%.cpp=$(BUILD_DIR)/%.o) \
            $(IMGUI_SRCS:%.cpp=$(BUILD_DIR)/%.o) \
            $(FILE_TEST_MAIN:%.cpp=$(BUILD_DIR)/%.o)

# --- Cibles ---
TARGET_APP  = $(BIN_DIR)/$(TARGET_NAME)
TARGET_TEST = $(BIN_DIR)/test_physics

# --- Règles de Compilation ---

all: $(TARGET_APP)

$(TARGET_APP): $(APP_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(APP_OBJS) -o $(TARGET_APP) $(LIBS)
	@echo "✅ Build App [$(MODE)] terminé -> $(TARGET_APP)"

$(TARGET_TEST): $(TEST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_OBJS) -o $(TARGET_TEST) $(LIBS)
	@echo "✅ Build Tests [$(MODE)] terminé -> $(TARGET_TEST)"

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# --- Commandes Utilisateur ---

run: $(TARGET_APP)
	@echo "🚀 Lancement..."
	./$(TARGET_APP)

test: $(TARGET_TEST)
	@echo "🧪 Tests..."
	./$(TARGET_TEST)

debug: $(TARGET_APP)

gdb: debug
	gdb ./$(TARGET_APP)

# --- Règles de Nettoyage (Ta demande spécifique) ---

# 1. Clean uniquement le CODE (src) du mode Debug. Garde les libs compilées.
clean_debug:
	rm -rf $(BIN_DEBUG)
	rm -rf $(BUILD_DEBUG)/src
	@echo "🧹 Debug Code cleaned (Libs kept)."

# 2. Clean uniquement le CODE (src) du mode Release. Garde les libs compilées.
clean_release:
	rm -rf $(BIN_RELEASE)
	rm -rf $(BUILD_RELEASE)/src
	@echo "🧹 Release Code cleaned (Libs kept)."

# 3. Supprime TOUT (Build + Bin). Force la recompilation des libs (ImGui, etc.)
clean_all:
	rm -rf build
	rm -rf bin
	@echo "💥 Full Clean Done (Everything deleted)."

.PHONY: all clean_debug clean_release clean_all clean_legacy run test debug gdb