from omni.isaac.kit import SimulationApp

# Start the simulation
simulation_app = SimulationApp({"headless": True})


from omni.isaac.core import World

track_cfg = {
    "name": "track",
    "num_tracks": 10000,
    "env_spacing": 0.0,
    "line_thickness": 0.001,
    "line_length": 0.8,
    "line_width": 0.05,
    "track_config": {
        "name": "random_angular",
        "max_corners": 20,
        "min_corners": 8,
        "min_radius": 0.25,
        "resampling_samples": 1000,
        "theta_noise": 0.0,
        "track_width": 24.0,
        "track_length": 24.0,
        "track_thickness": 1.5,
        "seed": 42,
        "resolution": 0.01,
    },
}

if __name__ == "__main__":
    from racetrack_track_builder_2 import TrackBuilderPreGen, TrackBuilderConfig
    from assets_managers import AssetsManagers

    track_cfg = TrackBuilderConfig(**track_cfg)

    TB = TrackBuilderPreGen(track_cfg, save_path="assets/tracks.pkl")
    TB.generate()
