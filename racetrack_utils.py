import omni
from WorldBuilders.pxr_utils import addDefaultOps, setDefaultOps
from pxr import Sdf, Gf, UsdShade


def createCubeLikeObject(
    stage, target_path, scale=[1, 1, 1], translation=[0, 0, 0], rotation=[0, 0, 0, 1]
):
    result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
    omni.kit.commands.execute("MovePrim", path_from=path, path_to=target_path)
    prim = stage.GetPrimAtPath(target_path)
    addDefaultOps(prim)
    setDefaultOps(prim, translation, rotation, scale)


def createPlaneLikeObject(
    stage, target_path, scale=[1, 1, 1], translation=[0, 0, 0], rotation=[0, 0, 0, 1]
):
    result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Plane")
    omni.kit.commands.execute("MovePrim", path_from=path, path_to=target_path)
    prim = stage.GetPrimAtPath(target_path)
    addDefaultOps(prim)
    setDefaultOps(prim, translation, rotation, scale)


def createCylinderLikeObject(
    stage, target_path, scale=[1, 1, 1], translation=[0, 0, 0], rotation=[0, 0, 0, 1]
):
    result, path = omni.kit.commands.execute(
        "CreateMeshPrimCommand", prim_type="Cylinder"
    )
    omni.kit.commands.execute("MovePrim", path_from=path, path_to=target_path)
    prim = stage.GetPrimAtPath(target_path)
    addDefaultOps(prim)
    setDefaultOps(prim, translation, rotation, scale)


def loadAshPlankMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Wood/Ash_Planks.mdl",
        mtl_name="Ash_Planks",
        mtl_path=Sdf.Path("/Looks/ash"),
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/ash")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(34, 37), Sdf.ValueTypeNames.TexCoord2f
    )

    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak_Planks.mdl",
        mtl_name="Oak_Planks",
        mtl_path=Sdf.Path("/Looks/oak"),
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/oak")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(34, 37), Sdf.ValueTypeNames.TexCoord2f
    )


def loadSeatMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="http://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/vMaterials_2/Leather/ABS_Hard_Leather.mdl",
        mtl_name="ABS_Hard_Leather_Peach",
        mtl_path="/Looks/seats",
    )


def loadStairsMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Plastics/Rubber_Smooth.mdl",
        mtl_name="Rubber_Smooth",
        mtl_path=Sdf.Path("/Looks/stairs"),
    )


def loadConcreteMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Masonry/Concrete_Polished.mdl",
        mtl_name="Concrete_Polished",
        mtl_path=Sdf.Path("/Looks/concrete"),
        select_new_prim=False,
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/concrete")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(1, 1), Sdf.ValueTypeNames.TexCoord2f
    )
    omni.usd.create_material_input(
        mtl_prim, "project_uvw", True, Sdf.ValueTypeNames.Bool
    )
    omni.usd.create_material_input(
        mtl_prim, "world_or_object", True, Sdf.ValueTypeNames.Bool
    )


def loadWhitePlasticMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Plastics/Plastic.mdl",
        mtl_name="Plastic",
        mtl_path="/Looks/panels",
    )


def loadWallMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Masonry/Stucco.mdl",
        mtl_name="Stucco",
        mtl_path=Sdf.Path("/Looks/walls"),
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/walls")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(1, 1), Sdf.ValueTypeNames.TexCoord2f
    )
    omni.usd.create_material_input(
        mtl_prim, "project_uvw", True, Sdf.ValueTypeNames.Bool
    )
    omni.usd.create_material_input(
        mtl_prim, "world_or_object", True, Sdf.ValueTypeNames.Bool
    )


def loadGrassMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Natural/Grass_Countryside.mdl",
        mtl_name="Grass_Countryside",
        mtl_path=Sdf.Path("/Looks/grass"),
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/grass")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(1, 1), Sdf.ValueTypeNames.TexCoord2f
    )
    omni.usd.create_material_input(
        mtl_prim, "project_uvw", True, Sdf.ValueTypeNames.Bool
    )
    omni.usd.create_material_input(
        mtl_prim, "world_or_object", True, Sdf.ValueTypeNames.Bool
    )


def loadCeilingMaterial(stage):
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="omniverse://localhost/NVIDIA/Materials/Base/Metals/CorrugatedMetal.mdl",
        mtl_name="CorrugatedMetal",
        mtl_path=Sdf.Path("/Looks/ceiling"),
    )
    mtl_prim = stage.GetPrimAtPath("/Looks/ceiling")
    omni.usd.create_material_input(
        mtl_prim, "texture_scale", Gf.Vec2f(0.5, 0.5), Sdf.ValueTypeNames.TexCoord2f
    )
    omni.usd.create_material_input(
        mtl_prim, "project_uvw", True, Sdf.ValueTypeNames.Bool
    )
    omni.usd.create_material_input(
        mtl_prim, "world_or_object", True, Sdf.ValueTypeNames.Bool
    )


def bindMaterial(stage, mtl_prim_path, prim_path):
    mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    shade = UsdShade.Material(mtl_prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        shade, UsdShade.Tokens.strongerThanDescendants
    )
