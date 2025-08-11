# Isaac_simul/stacker.py (수정)
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.materials import PhysicsMaterial
from Isaac_simul.usd_utils import convert_path_to_position

class Stacker:
    def __init__(self, prim_path: str, id_stacker: int):
        self.prim_path = prim_path
        self.id_stacker = id_stacker
        self.slots = []

        self.position = convert_path_to_position(self.prim_path)

        self.material = PhysicsMaterial(
            prim_path=self.prim_path,
            static_friction=1.2,
            dynamic_friction=0.9,
            restitution=0.0
        )

        self.body = FixedCuboid(
            prim_path=self.prim_path,
            position=self.position
        )
        self.body.apply_physics_material(self.material)

    def fill_slot_with_pallet(self, pallet):
        self.slots.append(pallet)
        print(f"Stacker {self.id_stacker} 팔레트 {pallet.id} 보관")

    def empty_slot(self):
        if self.slots:
            pallet = self.slots.pop(0)
            print(f"Stacker {self.id_stacker} 팔레트 {pallet.id} 반환")
            return pallet
        print(f"Stacker {self.id_stacker} 비어 있음: 반환할 팔레트가 없습니다.")
        return None
