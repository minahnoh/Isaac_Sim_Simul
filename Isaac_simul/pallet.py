# Isaac_simul/pallet.py (수정)
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.materials import PhysicsMaterial
from pxr import Gf, UsdGeom
import omni.usd
from Isaac_simul.usd_utils import convert_path_to_position

class Pallet:
    def __init__(self, prim_path: str, id_pallet):
        """
        물리 팔레트 생성 (Prim의 실제 위치 사용)
        """
        self.prim_path = prim_path
        self.id = id_pallet  # <-- 통일: id 로 사용

        # 현재 Stage에서 prim 참조
        self.position = convert_path_to_position(self.prim_path)

        # 물리 재질 생성 및 적용
        self.material = PhysicsMaterial(
            prim_path=self.prim_path,
            static_friction=1.0,
            dynamic_friction=0.8,
            restitution=0.1
        )

        # 팔레트 물리 설정
        self.cuboid = DynamicCuboid(
            prim_path=self.prim_path,
            position=self.position,
            mass=5.0
        )
        self.cuboid.apply_physics_material(self.material)
