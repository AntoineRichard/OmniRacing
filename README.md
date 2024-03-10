# OmniRacing

For now it only provides track generation.

How to install:
```bash
git clone --recurse-submodules https://github.com/AntoineRichard/OmniRacing.git
cd OmniLRS
git submodule init
git submodule update
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh -m pip install opencv-python

# Takes 4Gb of space (can be stopped whenever, you only need one)
python3 utils/download_hdris.py

# Generates a bunch of tracks to act like a cache
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh utils/racetrack_pregenerate_tracks.py 
```




Test the track generation:
```bash
python3 racetrack_generators.py
```


Run the sim:
```bash
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh racetrack_demo_env.py
```
