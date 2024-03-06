from omni.isaac.kit import SimulationApp

# Start the simulation
simulation_app = SimulationApp({"headless": False})


from omni.isaac.core import World

track_cfg = {
    "name": "track",
    "num_tracks": 16,
    "env_spacing": 30.0,
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
    from racetrack_track_builder_2 import TrackBuilder, TrackBuilderConfig
    from assets_managers import AssetsManagers

    world = World(stage_units_in_meters=1.0)
    physics_ctx = world.get_physics_context()
    physics_ctx.set_solver_type("PGS")

    AM = AssetsManagers("/World")
    track_cfg = TrackBuilderConfig(**track_cfg)

    TB = TrackBuilder(track_cfg, AM)
    TB.build()

    # xy = generateOvalTrack(x_max = 15, y_max=8.5)

    i = 0
    while simulation_app.is_running():
        world.step(render=True)
        if i % 300 == 0:
            TB.randomize()
            AM.randomize_skydome()
            i = 0
        i += 1
    simulation_app.close()
