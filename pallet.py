# pallet.py
from omni.isaac.core.objects import DynamicCuboid
from pxr import Gf
from typing import Optional
from config_sim import PALLET_SIZE

class Pallet:
    def __init__(self, prim_path: str, position: Gf.Vec3d, color=(0.6, 0.4, 0.2), items:int=0):
        self.prim_path = prim_path
        self.items = items
        self.node = DynamicCuboid(
            prim_path=prim_path,
            name=prim_path.split("/")[-1],
            position=position,
            size=PALLET_SIZE,
            color=color,
            fixed_base=False
        )
        self.carried_by = None  # AGV가 들고 있으면 AGV 객체 참조
        self.at = "stacker"     # "stacker" | "printer" | "agv" | "done"

    def set_world_pose(self, pos: Gf.Vec3d):
        self.node.set_world_pose(position=pos)
