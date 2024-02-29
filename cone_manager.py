from WorldBuilders.pxr_utils import createXform
# Createis a bunch of cone primitives and provides an interface to an instancer to place them.


# I should instead overload my instancing stuff.
class ConeManager:
    def __init__(self):
        pass

    def add_reference(self):
        pass

    def instantiates_all(self):

        for og_cone in self.og_cones:
            
            
    

    print(axis)
     import omni
from pxr import Usd

path = "/World/container"
variant_type = "shadingVariant"

stage = omni.usd.get_context().get_stage()
prim_path = omni.usd.get_stage_next_free_path(stage, path, False)
obj_prim = stage.DefinePrim(prim_path, "Xform")
book_stage:Usd.Stage = Usd.Stage.Open("/home/antoine/Documents/OmniRacing/assets/cone_3.usd")
default_prim = book_stage.GetDefaultPrim()
variants = default_prim.GetVariantSet(variant_type).GetVariantNames() 

for variant in variants:
            cone_path = omni.usd.get_stage_next_free_path(self._stage,
                prototypes.GetPath().AppendPath("book_A"), 
                False
            )
            obj_prim = stage.DefinePrim(cone_path, "Xform")
            obj_prim.GetReferences().AddReference(path)

            omni.kit.commands.execute('CreateReference',
                path_to=book_path,
                asset_path=str(BOOK_A_USD),
                usd_context=omni.usd.get_context()
            )
            prim = self._stage.GetPrimAtPath(book_path)
            prim.GetVariantSet("color").SetVariantSelection(variant)
            asset_xform = UsdGeom.Xform(prim)
print(variants)
