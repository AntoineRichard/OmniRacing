from WorldBuilders.pxr_utils import createXform
from WorldBuilders.pxr_utils import (
    createStandaloneInstance,
    setInstancerParameters,
)
from pxr import Usd, UsdGeom, UsdLux
import random
import omni
import os


class AssetManager:
    def __init__(
        self,
        path_to_asset_folder: str = None,
        variant_type: str = None,
        instancer_path: str = None,
    ):
        self.stage = omni.usd.get_context().get_stage()
        self.path_to_asset_folder = path_to_asset_folder
        self.variant_type = variant_type
        self.instancer_path = instancer_path
        self.get_asset_pathes()
        self.instantiate_instancer()

    def get_asset_pathes(self):
        files = os.listdir(self.path_to_asset_folder)
        self.asset_pathes = [
            os.path.join(self.path_to_asset_folder, file)
            for file in files
            if file.endswith(".usd")
        ]

    def instantiate_instancer(self):
        self.instancer = createStandaloneInstance(self.stage, self.instancer_path)
        for asset_path in self.asset_pathes:
            asset_stage: Usd.Stage = Usd.Stage.Open(asset_path)
            default_prim = asset_stage.GetDefaultPrim()
            if self.variant_type is None:
                prim_path = omni.usd.get_stage_next_free_path(
                    self.stage, os.path.join(self.instancer_path, "asset"), False
                )
                obj_prim = self.stage.DefinePrim(prim_path, "Xform")
                obj_prim.GetReferences().AddReference(asset_path)
                obj_prim.SetInstanceable(True)
                self.instancer.GetPrototypesRel().AddTarget(prim_path)
            else:
                variants = default_prim.GetVariantSet(
                    self.variant_type
                ).GetVariantNames()
                for variant in variants:
                    prim_path = omni.usd.get_stage_next_free_path(
                        self.stage, os.path.join(self.instancer_path, "asset"), False
                    )
                    obj_prim = self.stage.DefinePrim(prim_path, "Xform")
                    obj_prim.GetReferences().AddReference(asset_path)
                    obj_prim.SetInstanceable(True)
                    prim = self.stage.GetPrimAtPath(prim_path)
                    prim.GetVariantSet(self.variant_type).SetVariantSelection(variant)
                    self.instancer.GetPrototypesRel().AddTarget(prim_path)

    def set_instancer_parameters(self, pos, **kwargs):
        setInstancerParameters(self.stage, self.instancer_path, pos, **kwargs)


class SkyDomeManager:
    def __init__(
        self,
        path_to_hdri_folder: str = "assets/hdris",
        skydome_path: str = "skydome",
    ):
        self.stage = omni.usd.get_context().get_stage()
        self.path_to_hdri_folder = path_to_hdri_folder
        self.skydome_path = skydome_path
        self.get_asset_pathes()
        self.create_dome_light()

    def get_asset_pathes(self):
        files = os.listdir(self.path_to_hdri_folder)
        self.hdr_pathes = [
            os.path.join(self.path_to_hdri_folder, file)
            for file in files
            if file.endswith(".hdr")
        ]

    def create_dome_light(self):
        dome_light_path = self.skydome_path + "/dome_light"
        self.usdLuxDomeLight = UsdLux.DomeLight.Define(self.stage, dome_light_path)
        self.usdLuxDomeLight.CreateIntensityAttr(1000)
        self.usdLuxDomeLight.CreateTextureFileAttr(self.hdr_pathes[0])

    def randomize(self):
        idx = random.randint(0, len(self.hdr_pathes) - 1)
        self.usdLuxDomeLight.GetTextureFileAttr().Set(self.hdr_pathes[idx])

    def set_skydome_id(self, id: int):
        self.usdLuxDomeLight.GetTextureFileAttr().Set(self.hdr_pathes[id])


class AssetsManagers:
    def __init__(self, root_path: str = "instancers"):
        self.cone_manager = AssetManager(
            path_to_asset_folder="assets/cones",
            variant_type="shadingVariant",
            instancer_path=os.path.join(root_path, "/cone_instancer"),
        )
        self.floor_manager = AssetManager(
            path_to_asset_folder="assets/floors",
            # variant_type="shadingVariant",
            instancer_path=os.path.join(root_path, "/floor_instancer"),
        )
        self.skydome_manager = SkyDomeManager(
            path_to_hdri_folder="assets/hdris",
            skydome_path=os.path.join(root_path, "/skydome"),
        )
        self.load_ground_plane(root_path)

    def load_ground_plane(self, root_path: str):
        stage = omni.usd.get_context().get_stage()
        obj_prim = stage.DefinePrim(root_path + "/ground_plane", "Xform")
        obj_prim.GetReferences().AddReference("assets/ground_plane.usd")

    def set_cone_parameters(self, pos, **kwargs):
        self.cone_manager.set_instancer_parameters(pos, **kwargs)

    def set_floor_parameters(self, pos, **kwargs):
        self.floor_manager.set_instancer_parameters(pos, **kwargs)

    def randomize_skydome(self):
        self.skydome_manager.randomize()
