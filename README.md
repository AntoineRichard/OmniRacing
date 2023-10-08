# OmniRacing

For now it only provides track generation.

How to install:
```bash
git clone --recurse-submodules https://github.com/AntoineRichard/OmniRacing.git
cd OmniLRS
git submodule init
git submodule update
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh opencv-python omegaconf hydra-core
```

Test the track generation:
```bash
python3 racetrack_generators.py
```


Run the sim:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh racetrack_demo_env.py
```