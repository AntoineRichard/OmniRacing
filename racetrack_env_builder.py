from racetrack_utils import (
    createCubeLikeObject,
    createPlaneLikeObject,
    createCylinderLikeObject,
    bindMaterial,
    loadAshPlankMaterial,
    loadCeilingMaterial,
    loadConcreteMaterial,
    loadGrassMaterial,
    loadSeatMaterial,
    loadStairsMaterial,
    loadWallMaterial,
    loadWhitePlasticMaterial,
)

from WorldBuilders.pxr_utils import (
    createStandaloneInstance,
    createInstancerAndCache4Variants,
    setInstancerParameters,
    addDefaultOps,
    setDefaultOps,
)

from pxr import UsdGeom, Sdf, Gf, UsdLux
import dataclasses
import numpy as np
import omni
import os

import dataclasses


@dataclasses.dataclass
class EnvironmentConfig:
    seating_depth: float = dataclasses.field(default_factory=float)
    room_length: float = dataclasses.field(default_factory=float)
    room_width: float = dataclasses.field(default_factory=float)
    num_seatings_layers: int = dataclasses.field(default_factory=int)
    seatings_layer_height: float = dataclasses.field(default_factory=float)
    seating_width: float = dataclasses.field(default_factory=float)
    stairs_width: float = dataclasses.field(default_factory=float)
    stairs_x_pos: float = dataclasses.field(default_factory=float)
    panels_thickness: float = dataclasses.field(default_factory=float)
    panels_height: float = dataclasses.field(default_factory=float)
    panels_y_pos: float = dataclasses.field(default_factory=float)
    seat_size: tuple = dataclasses.field(default_factory=tuple)
    wall_height: float = dataclasses.field(default_factory=float)
    wall_thickness: float = dataclasses.field(default_factory=float)
    window_height: float = dataclasses.field(default_factory=float)
    roof_width: float = dataclasses.field(default_factory=float)
    ligth_intensity: float = dataclasses.field(default_factory=float)
    num_lights: int = dataclasses.field(default_factory=int)


class EnvironmentBuilder:
    def __init__(self, cfg: EnvironmentConfig):
        self.stage = omni.usd.get_context().get_stage()
        self.settings = cfg

    def build(self):
        self.loadMaterials()
        self.buildStands("/stands")
        self.buildCourt("/court")
        self.buildCeiling("/ceiling")
        self.buildOutside("/outdoor")

    def createSeatings(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        for i in range(self.settings.num_seatings_layers):
            z = (
                (self.settings.num_seatings_layers - i)
                * self.settings.seatings_layer_height
                - self.settings.seatings_layer_height / 2
            )
            depth = (i + 1) * self.settings.seating_depth
            y = i * self.settings.seating_depth / 2 + self.settings.seating_depth / 2
            createCubeLikeObject(
                self.stage,
                os.path.join(root, "slab_" + str(i)),
                scale=(
                    self.settings.room_length,
                    depth,
                    self.settings.seatings_layer_height,
                ),
                translation=(0, y, z),
            )
            bindMaterial(self.stage, "/Looks/concrete", root + "/slab_" + str(i))

    def addStairs(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        x = self.settings.stairs_x_pos
        instancer = createStandaloneInstance(self.stage, root)
        depth = self.settings.seating_depth / 2
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "stair"),
            scale=(
                self.settings.stairs_width,
                depth,
                self.settings.seatings_layer_height / 2,
            ),
            translation=(0, 0, 0),
        )
        bindMaterial(self.stage, "/Looks/stairs", root + "/stair")
        instancer.GetPrototypesRel().AddTarget(os.path.join(root, "stair"))
        pos = []
        for i in range(self.settings.num_seatings_layers):
            z = (
                (self.settings.num_seatings_layers - i)
                * self.settings.seatings_layer_height
                - self.settings.seatings_layer_height * 3 / 4
            )
            y = (i + 1) * self.settings.seating_depth + depth / 2
            pos.append([x, y, z])
        x = -self.settings.stairs_x_pos
        for i in range(self.settings.num_seatings_layers):
            z = (
                (self.settings.num_seatings_layers - i)
                * self.settings.seatings_layer_height
                - self.settings.seatings_layer_height * 3 / 4
            )
            y = (i + 1) * self.settings.seating_depth + depth / 2
            pos.append([x, y, z])
        pos = np.array(pos)
        setInstancerParameters(self.stage, root, pos)

    def addFrontPanels(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        z = self.settings.seatings_layer_height / 2
        center_width = self.settings.stairs_x_pos * 2 - self.settings.stairs_width
        center_position = 0
        left_width = (
            self.settings.room_length / 2
            - self.settings.stairs_x_pos
            - self.settings.stairs_width / 2
        )
        left_position = -self.settings.room_length / 2 + left_width / 2
        right_width = (
            self.settings.room_length / 2
            - self.settings.stairs_x_pos
            - self.settings.stairs_width / 2
        )
        right_position = self.settings.room_length / 2 - right_width / 2
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "panel_left"),
            (left_width, self.settings.panels_thickness, self.settings.panels_height),
            (left_position, self.settings.panels_y_pos, z),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "panel_center"),
            (center_width, self.settings.panels_thickness, self.settings.panels_height),
            (center_position, self.settings.panels_y_pos, z),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "panel_right"),
            (right_width, self.settings.panels_thickness, self.settings.panels_height),
            (right_position, self.settings.panels_y_pos, z),
        )
        bindMaterial(self.stage, "/Looks/panels", root + "/panel_left")
        bindMaterial(self.stage, "/Looks/panels", root + "/panel_center")
        bindMaterial(self.stage, "/Looks/panels", root + "/panel_right")

    def addSeats(self, root):
        instancer = createStandaloneInstance(self.stage, root)
        createCubeLikeObject(
            self.stage, os.path.join(root, "seat"), (0.8, 0.3, 0.04), (0, 0, 0)
        )
        bindMaterial(self.stage, "/Looks/seats", root + "/seat")
        instancer.GetPrototypesRel().AddTarget(os.path.join(root, "seat"))

        pos = []
        for i in range(self.settings.num_seatings_layers):
            z = (
                self.settings.num_seatings_layers - i
            ) * self.settings.seatings_layer_height + self.settings.seat_size[2] / 2
            y = (i + 1) * self.settings.seating_depth - self.settings.seat_size[1] / 2
            for j in range(self.settings.room_length):
                x = j - self.settings.room_length / 2 + self.settings.seat_size[0] / 2
                if (x > -self.settings.stairs_x_pos - 0.75) and (
                    x < -self.settings.stairs_x_pos + 0.75
                ):
                    continue
                if (x > self.settings.stairs_x_pos - 0.75) and (
                    x < self.settings.stairs_x_pos + 0.75
                ):
                    continue
                pos.append([x, y, z])
        pos = np.array(pos)
        setInstancerParameters(self.stage, root, pos)

    def createStandWalls(self, root="/walls"):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        addDefaultOps(xformPrim)
        bindMaterial(self.stage, "/Looks/walls", root)
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "back_p1"),
            (self.settings.room_length * 3 / 4, self.settings.wall_thickness, 4.0),
            (0, -self.settings.wall_thickness / 2, 4.0 / 2),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "back_p2"),
            (self.settings.room_length * 3 / 4, self.settings.wall_thickness, 0.75),
            (
                0,
                -self.settings.wall_thickness / 2,
                self.settings.wall_height - 0.75 / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "back_p3"),
            (
                self.settings.room_length / 8,
                self.settings.wall_thickness,
                self.settings.wall_height,
            ),
            (
                self.settings.room_length * 7 / 16,
                -self.settings.wall_thickness / 2,
                self.settings.wall_height / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "back_p4"),
            (
                self.settings.room_length / 8,
                self.settings.wall_thickness,
                self.settings.wall_height,
            ),
            (
                -self.settings.room_length * 7 / 16,
                -self.settings.wall_thickness / 2,
                self.settings.wall_height / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "left"),
            (self.settings.wall_thickness, 9, self.settings.wall_height),
            (
                -self.settings.room_length / 2 - self.settings.wall_thickness / 2,
                4.5,
                self.settings.wall_height / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "right"),
            (self.settings.wall_thickness, 9, self.settings.wall_height),
            (
                self.settings.room_length / 2 + self.settings.wall_thickness / 2,
                4.5,
                self.settings.wall_height / 2,
            ),
        )
        instancer = createStandaloneInstance(self.stage, root + "/back_p5_i")
        createCubeLikeObject(
            self.stage,
            root + "/back_p5_i/part",
            (0.25, self.settings.wall_thickness, self.settings.wall_height - 4.75),
            (0, 0, 0),
        )
        instancer.GetPrototypesRel().AddTarget(os.path.join(root, "back_p5_i/part"))
        pos = []
        y = -self.settings.wall_thickness / 2
        z = 4.0 + (self.settings.wall_height - 4.75) / 2
        for i in range(1, 11):
            x = (
                i * self.settings.room_length * 3 / 4 * 1 / 11
                - self.settings.room_length * 3 / 4 * 1 / 2
            )
            pos.append([x, y, z])
        pos = np.array(pos)
        setInstancerParameters(self.stage, root + "/back_p5_i", pos)

    def createCourtWalls(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        addDefaultOps(xformPrim)
        bindMaterial(self.stage, "/Looks/walls", root)
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "right_p1"),
            (
                self.settings.wall_thickness,
                self.settings.room_width,
                self.settings.wall_height - self.settings.window_height - 0.75,
            ),
            (
                -self.settings.wall_thickness / 2 - self.settings.room_length / 2,
                0,
                (self.settings.wall_height - self.settings.window_height - 0.75) / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "right_p2"),
            (self.settings.wall_thickness, self.settings.room_width, 0.75),
            (
                -self.settings.wall_thickness / 2 - self.settings.room_length / 2,
                0,
                self.settings.wall_height - 0.75 / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "left_p1"),
            (
                self.settings.wall_thickness,
                self.settings.room_width,
                self.settings.wall_height - self.settings.window_height - 0.75,
            ),
            (
                self.settings.wall_thickness / 2 + self.settings.room_length / 2,
                0,
                (self.settings.wall_height - self.settings.window_height - 0.75) / 2,
            ),
        )
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "left_p2"),
            (self.settings.wall_thickness, self.settings.room_width, 0.75),
            (
                self.settings.wall_thickness / 2 + self.settings.room_length / 2,
                0,
                self.settings.wall_height - 0.75 / 2,
            ),
        )
        instancer = createStandaloneInstance(self.stage, root + "/wall_p3_i")
        createCubeLikeObject(
            self.stage,
            root + "/wall_p3_i/part",
            (self.settings.wall_thickness, 0.25, self.settings.window_height),
            (0, 0, 0),
        )
        instancer.GetPrototypesRel().AddTarget(root + "/wall_p3_i/part")
        pos = []
        x = -self.settings.wall_thickness / 2 - self.settings.room_length / 2
        z = self.settings.wall_height - 0.75 - self.settings.window_height / 2
        for i in range(1, 9):
            y = i * self.settings.room_width * 1 / 9 - self.settings.room_width * 1 / 2
            pos.append([x, y, z])
        x = self.settings.wall_thickness / 2 + self.settings.room_length / 2
        for i in range(1, 9):
            y = i * self.settings.room_width * 1 / 9 - self.settings.room_width * 1 / 2
            pos.append([x, y, z])
        pos = np.array(pos)
        setInstancerParameters(self.stage, root + "/wall_p3_i", pos)

    def buildStand(self, root="/stand", pos=(0, 0, 0), rotation=(0, 0, 0, 1)):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        addDefaultOps(xformPrim)
        setDefaultOps(xformPrim, pos, rotation, (1, 1, 1))
        self.createSeatings(root + "/seatings")
        self.addStairs(root + "/stairs_left")
        self.addStairs(root + "/stairs_right")
        self.addFrontPanels(root + "/front_panels")
        self.addSeats(root + "/seats")
        self.createStandWalls(root + "/walls")

    def createRoof(self, root="/roof", height=7.0, thickness=0.1):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "roof"),
            (self.settings.room_length, self.settings.roof_width, thickness),
            (0, 0, height + thickness / 2),
        )
        bindMaterial(self.stage, "/Looks/ceiling", root + "/roof")

    def createLights(self, root):
        #        stage, root="/lights", width=34, length=37, height=6.0, intensity=1000, num_lights=9
        #    ):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        x_min = -self.settings.room_length / 2 + 5
        x_max = self.settings.room_length / 2 - 5
        y_min = -self.settings.roof_width / 2 + 12
        y_max = self.settings.roof_width / 2 - 12
        num_lights = int(np.sqrt(self.settings.num_lights))
        x = np.linspace(x_min, x_max, num_lights)
        y = np.linspace(y_min, y_max, num_lights)
        x, y = np.meshgrid(x, y)
        for i, (x, y) in enumerate(zip(x.flatten(), y.flatten())):
            disk_light = UsdLux.DiskLight.Define(
                self.stage, root + "/projector_" + str(i)
            )
            disk_light.GetIntensityAttr().Set(1000)
            disk_light.GetRadiusAttr().Set(5)
            addDefaultOps(disk_light)
            setDefaultOps(
                disk_light,
                (x, y, self.settings.wall_height - 0.5),
                (0, 0, 0, 1),
                (1, 1, 1),
            )

    def createGround(self, root, width=400, length=400, height=-0.01):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        createCubeLikeObject(
            self.stage,
            os.path.join(root, "ground"),
            (width, length, 0.01),
            (0, 0, height),
        )
        bindMaterial(self.stage, "/Looks/grass", root + "/ground")

    def createSky(self, root):
        omni.kit.commands.execute(
            "CreatePayloadCommand",
            usd_context=omni.usd.get_context(),
            path_to=Sdf.Path(root),
            asset_path="omniverse://localhost/NVIDIA/Assets/Skies/Dynamic/ClearSky.usd",
            instanceable=False,
        )

    def buildOutside(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        self.createSky(root + "/sky")
        self.createGround(root + "/ground")

    def buildStands(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        self.buildStand(
            root=root + "/stand_left", pos=(0, -18.5, 0), rotation=(0, 0, 0, 1)
        )
        self.buildStand(
            root=root + "/stand_right", pos=(0, 18.5, 0), rotation=(0, 0, 1, 0)
        )

    def createCourt(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        createPlaneLikeObject(
            self.stage,
            os.path.join(root, "court"),
            scale=(self.settings.room_length, self.settings.roof_width, 1),
            translation=(0, 0, 0),
        )
        bindMaterial(self.stage, "/Looks/oak", root + "/court")

    def buildCourt(self, root):
        xformPrim = UsdGeom.Xform.Define(self.stage, root)
        self.createCourt(root + "/court")
        self.createCourtWalls(root + "/walls")

    def buildCeiling(self, root):
        self.createRoof(root=root + "/roof")
        self.createLights(root + "/lights")

    def loadMaterials(self):
        loadStairsMaterial(self.stage)
        loadSeatMaterial(self.stage)
        loadConcreteMaterial(self.stage)
        loadAshPlankMaterial(self.stage)
        loadWhitePlasticMaterial(self.stage)
        loadWallMaterial(self.stage)
        loadGrassMaterial(self.stage)
        loadCeilingMaterial(self.stage)
