# stacker.py
from omni.isaac.core.objects import DynamicCuboid
from pxr import Gf
from typing import List, Optional
from config_sim import STACKER_SIZE

class Stacker:
    def __init__(self, prim_path: str, base_pos: Gf.Vec3d, color=(0.3, 0.6, 0.9)):
        self.prim_path = prim_path
        self.base_pos = base_pos
        self.node = DynamicCuboid(
            prim_path=prim_path,
            name=prim_path.split("/")[-1],
            position=base_pos,
            size=STACKER_SIZE,
            color=color,
            fixed_base=True
        )
        self.slots: List[Optional[object]] = []  # Pallet 참조 저장
        self.capacity = 20

    def push(self, pallet) -> bool:
        if len(self.slots) < self.capacity:
            self.slots.append(pallet)
            pallet.at = "stacker"
            return True
        return False

    def pop(self):
        if self.slots:
            p = self.slots.pop(0)
            return p
        return None

    def has_pallet(self) -> bool:
        return len(self.slots) > 0
