#include "core/Game.hpp"
#include "utils/Logger.hpp"

int main(int, char**) {
    Game game;

    if (!game.init()) {
        log_error("Game failed to initialize.", "");
        return -1;
    }

    game.run();

    return 0;
}
