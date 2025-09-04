# 🎮 REG Engine – A Python-Based 2D & 3D Game Engine

> *Learn. Build. Play. Explore.*  
> REG (Rapid Engine for Games) is a lightweight, educational, and modular game engine written in **Python**.  
It empowers students, researchers, and indie devs to create **2D & 3D games** while understanding the fundamentals of game engine design.

---

## ✨ Features

- 🕹️ **2D & 3D Game Support** – Pygame for 2D, PyOpenGL for 3D  
- ⚡ **Modular Architecture** – Clear separation of core systems (runtime, rendering, physics, AI, logic)  
- 🤖 **AI Integration** – Behavior Trees & Finite State Machines for smart NPCs  
- 🌍 **Physics Simulation** – PyBullet-powered 3D physics + custom 2D collision detection  
- 🎨 **Shader System** – GLSL-based lighting, materials, and visual effects  
- 🎮 **Sample Games**  
  - Classic **Pong** (2D)  
  - AI-driven **Patrol Simulation** (3D)  

---

## 💻 Tech Stack

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54) ![R](https://img.shields.io/badge/r-%23276DC3.svg?style=for-the-badge&logo=r&logoColor=white)

## 🏗️ Architecture Overview

REG is built on a **five-layer architecture**:

1. **Game Application Layer** – Your game logic & rules  
2. **Game Logic & AI Layer** – FSMs & behavior trees for AI  
3. **Physics & Collision Layer** – PyBullet + custom 2D physics  
4. **Rendering Layer** – Pygame (2D) + PyOpenGL (3D, shaders, cameras)  
5. **Core Runtime** – Engine loop, input polling, event handling  

This modularity makes REG both **educational** and **extensible**.

---

## 🚀 Getting Started

### 1. Clone the repo
```bash
git clone https://github.com/yourusername/REG_Engine.git
cd REG_Engine
````

### 2. Create a virtual environment (optional but recommended)

```bash
python -m venv venv
source venv/bin/activate   # Linux/Mac
venv\Scripts\activate      # Windows
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Run sample games

```bash
# Run 2D Pong
python game/pong.py

# Run 3D Patrol Simulation
python game/patrol3d.py
```

---

## 📂 Project Structure

```
REG_Engine/
│── reg/           # Core engine modules
│── game/          # Game-specific scripts
│── assets/        # Sprites, textures, sounds
│── prefabs/       # Reusable blueprints
│── requirements.txt
│── README.md
```

---

## 📊 Performance

| Application          | Avg FPS | CPU Usage | Memory |
| -------------------- | ------- | --------- | ------ |
| 2D Pong Game         | 240 FPS | 12%       | 80 MB  |
| 3D Patrol Simulation | 90 FPS  | 30%       | 210 MB |

Runs smoothly even on mid-range laptops with **integrated graphics**.

---

## 🎯 Roadmap

* [ ] GUI-based Scene Editor
* [ ] Advanced Physics (soft bodies, fluids)
* [ ] Built-in UI Toolkit
* [ ] Networking & Multiplayer
* [ ] Cross-platform deployment (WebGL, Android, iOS)
* [ ] Plugin ecosystem & scripting support
* [ ] Open-source community modules

---

## 👨‍💻 Authors
* **Mikiyas Aregawi Bahre**
* **Belard Mwambuka**
* **Godfrey Brew Ntiamoah**


Under the guidance of **Mr. Awadhesh Dixit**, SRM University-AP.

---

## 📝 License

This project is open-source under the **MIT License** – feel free to use, modify, and contribute!

---

## 🌟 Acknowledgments

* [Pygame](https://www.pygame.org/docs/)
* [PyOpenGL](http://pyopengl.sourceforge.net/)
* [PyBullet](https://github.com/bulletphysics/bullet3)
* Inspiration from Unity, Unreal, Godot, and countless indie devs 🚀

---

> 💡 *REG is not just a game engine — it’s a classroom, a playground, and a launchpad for your creativity.*

```

---

Would you like me to also **add badges (build, Python version, license, contributors)** at the top of the README to make it look more professional for GitHub?
```

