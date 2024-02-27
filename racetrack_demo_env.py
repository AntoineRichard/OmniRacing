from omni.isaac.kit import SimulationApp

# Start the simulation
simulation_app = SimulationApp({"headless": False})


from omni.isaac.core import World

track_cfg = {
    "name": "track",
    "line_thickness": 0.001,
    "line_length": 0.15,
    "line_width": 0.05,
    "track_config": {
        "name": "random_angular",
        "max_corners": 20,
        "min_corners": 8,
        "min_radius": 0.25,
        "resampling_samples": 1000,
        "theta_noise": 0.0,
        "track_width": 12.0,
        "track_length": 24.0,
        "track_thickness": 1.0,
        "seed": 42,
        "resolution": 0.01,
    },
}

env_cfg = {
    "seating_depth": 0.8,
    "room_length": 34,
    "num_seatings_layers": 10,
    "seatings_layer_height": 0.3,
    "seating_width": 1,
    "stairs_width": 1.5,
    "stairs_x_pos": 10,
    "panels_thickness": 0.1,
    "panels_height": 0.75,
    "panels_y_pos": 9,
    "seat_size": (0.8, 0.3, 0.04),
    "wall_height": 7.0,
    "wall_thickness": 0.1,
    "room_width": 20,
    "window_height": 5.0,
    "roof_width": 37.0,
    "ligth_intensity": 1000,
    "num_lights": 9,
}

if __name__ == "__main__":
    from racetrack_env_builder import EnvironmentBuilder, EnvironmentConfig
    from racetrack_track_builder import TrackBuilder, TrackBuilderConfig

    world = World(stage_units_in_meters=1.0)
    physics_ctx = world.get_physics_context()
    physics_ctx.set_solver_type("PGS")

    env_cfg = EnvironmentConfig(**env_cfg)
    EB = EnvironmentBuilder(env_cfg)
    EB.build()

    track_cfg = TrackBuilderConfig(**track_cfg)
    TB = TrackBuilder(track_cfg)
    TB.build()

    # xy = generateOvalTrack(x_max = 15, y_max=8.5)

    i = 0
    while simulation_app.is_running():
        world.step(render=True)
        if i % 30 == 0:
            TB.randomize()
            i = 0
        i += 1
    simulation_app.close()
