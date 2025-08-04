from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.materials import PhysicsMaterial
from pxr import Gf, UsdGeom
import omni.usd

class Stacker:
    def __init__(self, prim_path: str, stacker_id):
        """
        고정 스태커 생성 (Prim의 실제 위치 사용)

        Args:
            prim_path: Stage 내 Prim 경로
            size: 크기
        """
        self.prim_path = prim_path
        self.id = stacker_id
        self.slots = [] # 팔레트를 저장

        # Stage에서 prim 위치 가져오기
        self.stage = omni.usd.get_context().get_stage()
        self.prim = self.stage.GetPrimAtPath(self.prim_path)

        xform = UsdGeom.Xformable(self.prim)
        matrix = xform.ComputeLocalToWorldTransform(0)
        self.position = Gf.Vec3f(matrix.ExtractTranslation())

        # 고정형 물리 바닥 생성
        self.material = PhysicsMaterial(static_friction=1.2, dynamic_friction=0.9, restitution=0.0)

        self.body = FixedCuboid(
            prim_path=self.prim_path,
            position=self.position
        )
        self.body.apply_physics_material(self.material)

    def store_pallet(self, pallet):
        self.slots.append(pallet)
        print(f"Stacker {self.id} 팔레트 {pallet.prim_path} 보관")

    def retrieve_pallet(self):
        if self.slots:
            pallet = self.slots.pop(0)
            print(f"Stacker {self.id} 팔레트 {pallet.prim_path} 꺼냄")
            return pallet
        return None
