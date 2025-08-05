import sys
import os
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from usd_utils import convert_path_to_position
from pxr import Gf, UsdGeom
import omni.usd

class Stacker:
    def __init__(self, prim_path: str, id_stacker: int):
        """
        고정 스태커 생성 (Prim의 실제 위치 사용)

        Attributes:
            prim_path: Stage 내 Prim 경로
            id_stacker(int): Unique stacker identifier
            slots : List of slots in the stacker for storing pallets 
        """
        self.prim_path = prim_path
        self.id_stacker = id_stacker
        self.slots = [] # 팔레트를 저장

        # prim_path를 위치벡터로 변환
        self.position = convert_path_to_position(self.prim_path)

        # 고정형 물리 바닥 생성
        self.material = PhysicsMaterial(prim_path = self.prim_path, static_friction=1.2, dynamic_friction=0.9, restitution=0.0)

        self.body = FixedCuboid(
            prim_path=self.prim_path,
            position=self.position
        )
        self.body.apply_physics_material(self.material)

    def fill_slot_with_pallet(self, pallet):
        self.slots.append(pallet)
        print(f"Stacker {self.id} 팔레트 {pallet.id_pallet} 보관")

    def empty_slot(self):
        if self.slots:
            pallet = self.slots.pop(0)
            print(f"Stacker {self.id} 팔레트 {pallet.id_pallet} 반환")
            return pallet
        print(f"Stacker {self.id} 비어 있음: 반환할 팔레트가 없습니다.")
        return None
