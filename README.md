# ECS Physics Sandbox

A 2D physics sandbox engine built from scratch using **C++**, **SDL2**, **EnTT**, and **Dear ImGui**. 

> **Note**: This project was originally intended to be an isometric sandbox game (as seen in `ROADMAP.md`). However, development focused primarily on building a robust, custom 2D physics engine and its ECS integration, which now serves as the core of this repository.

## 🚀 Key Features

- **Custom Physics Engine**: 
  - SAT (Separating Axis Theorem) based collision detection.
  - Supports Circles, Axis-Aligned Bounding Boxes (AABB), and Oriented Bounding Boxes (OBB).
  - Realistic collision resolution using Sequential Impulses.
  - Friction and restitution models.
- **ECS Architecture**: Leveraging `EnTT` for efficient entity management and decoupled systems.
- **Interactive Debugging**: 
  - Integrated `Dear ImGui` for real-time monitoring of engine state.
  - Performance tracking (FPS, entity count, energy conservation).
- **Time Manipulation**: 
  - Pause/Resume and Frame-by-frame stepping.
  - **History Rewind**: Ability to roll back the physics state to previous frames.
- **Serialization**: Lightweight snapshot system for state persistence and history.

## 🛠️ Tech Stack

- **Language**: C++17
- **Graphics & Input**: [SDL2](https://www.libsdl.org/)
- **Entity Component System**: [EnTT](https://github.com/skypjack/entt)
- **User Interface**: [Dear ImGui](https://github.com/ocornut/imgui)
- **Build System**: Makefile

## 📦 Project Structure

- `src/core`: Game loop and engine initialization.
- `src/physics`: Collision detection, impulse resolution, and spatial hashing.
- `src/render`: SDL2 and ImGui rendering systems.
- `src/common`: Component definitions and shared data structures.
- `src/utils`: Logging, serialization, and math helpers.
- `src/tests`: Comprehensive unit tests for physics and math.

## ⚙️ Building and Running

### Prerequisites

You need SDL2 installed on your system. On Ubuntu/Debian:
```bash
sudo apt-get install libsdl2-dev
```

### Build

To compile the application in release mode:
```bash
make
```

For debug mode with GDB symbols:
```bash
make debug
```

### Run

```bash
make run
```

### Tests

To run the physics test suite:
```bash
make test
```

## 🎮 Controls

- **Space**: Pause/Resume simulation.
- **Right Arrow**: Step one frame forward (when paused).
- **Left Arrow**: Rewind history (when paused).
- **R**: Reset scene.
- **Mouse**: Interactions (depending on entity configuration).

## 📄 License

This project is open-source. Feel free to use and modify it for your own learning or projects.
