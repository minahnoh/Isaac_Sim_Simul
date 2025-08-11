from pxr import Gf, UsdGeom
import omni.usd

def convert_path_to_position(prim_path):

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    matrix = xform.ComputeLocalToWorldTransform(0)
    position = Gf.Vec3f(matrix.ExtractTranslation())

    return position