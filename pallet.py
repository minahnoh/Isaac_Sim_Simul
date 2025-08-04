from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.materials import PhysicsMaterial
from pxr import Gf, UsdGeom
import omni.usd

class Pallet:
    def __init__(self, prim_path: str):
        """
        물리 팔레트 생성 (Prim의 실제 위치 사용)

        Args:
            prim_path: USD Stage 상의 경로
            size: 크기 (x, y, z)
        """
        self.prim_path = prim_path

        # 현재 Stage에서 prim 참조
        self.stage = omni.usd.get_context().get_stage()
        self.prim = self.stage.GetPrimAtPath(self.prim_path)

        # 위치는 Prim에서 직접 가져옴
        xform = UsdGeom.Xformable(self.prim)
        matrix = xform.ComputeLocalToWorldTransform(0)
        self.position = Gf.Vec3f(matrix.ExtractTranslation())

        # 물리 재질 생성 및 적용
        self.material = PhysicsMaterial(static_friction=1.0, dynamic_friction=0.8, restitution=0.1)

        # 팔레트 물리 설정
        self.cuboid = DynamicCuboid(
            prim_path=self.prim_path,
            position=self.position,
            mass=5.0
        )
        self.cuboid.apply_physics_material(self.material)
