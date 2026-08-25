# itajuba_2026

Competition repository for **Itajubá 2026** — eVTOL ITA.

## Structure

```
itajuba_2026/
├── scripts/          ← build, simulate, agent scripts
│   ├── build.sh
│   ├── simulate.sh
│   └── agent.sh
├── .github/workflows/
│   └── build.yml     ← CI: compila contra o workspace pinado (Humble + Jazzy)
├── fase1/             ← pacote ROS2 da fase 1 (criado com new_mission.sh)
├── fase2/             ← pacote ROS2 da fase 2
├── ...
└── README.md
```

Cada fase é um pacote ROS2 independente, gerado com o `new_mission.sh` (veja abaixo)
para já sair compilável e voável (arma, sobe, pousa).

## Dependencies

This competition repo uses the following team packages (cloned side-by-side in `src/`):

| Package | What it provides |
|---|---|
| `fsm` | Finite state machine framework |
| `drone_lib` | PX4 drone abstraction (`Drone` class) |
| `stdstates` | Reusable states (takeoff, landing, PID, movement) |
| `stdbt` | Reusable Behavior Tree nodes (para fases modeladas com `--engine bt`) |
| `cv_nodes` | Computer vision ROS2 nodes |
| `custom_msgs` | Shared message/service definitions |

See [ARCHITECTURE.md](../../../ARCHITECTURE.md) for the full workspace guide.

## Criando uma nova fase

De dentro deste diretório (`src/itajuba_2026/`):

```bash
~/evtol/dev/templates/new_mission.sh fase1
~/evtol/dev/templates/new_mission.sh fase2 --engine bt   # ou fsm (padrão)
```

Isso gera o pacote completo (`package.xml`, `CMakeLists.txt`, `src/`, `include/`,
`config/`, `launch/`) já compilável, e atualiza as tasks do VS Code.

Depois, em `scripts/simulate.sh`, preencha o bloco `case` com o mundo, modelo e
pose inicial de cada fase.

## Quick Start

```bash
# From the workspace root (evtol/dev/):
# 1. Build dependencies first
bash src/itajuba_2026/scripts/build.sh deps

# 2. Build a mission package
bash src/itajuba_2026/scripts/build.sh fase1

# 3. Start simulation
bash src/itajuba_2026/scripts/agent.sh          # Terminal 1
bash src/itajuba_2026/scripts/simulate.sh fase1 # Terminal 2
source install/setup.bash && ros2 run fase1 fase1  # Terminal 3
```

## License

MIT
