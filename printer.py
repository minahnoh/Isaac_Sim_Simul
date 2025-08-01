# printer.py
from omni.isaac.core.objects import DynamicCuboid
from pxr import Gf
from config_sim import PRINTER_SIZE, PRINTER_LOAD_TIME, PRINTER_BUILD_TIME, PRINTER_UNLOAD_TIME
from logger_sim import SimLogger
from typing import Optional

class Printer:
    def __init__(self, prim_path: str, input_pos: Gf.Vec3d, output_pos: Gf.Vec3d, color=(0.7, 0.7, 0.7)):
        self.prim_path = prim_path
        self.input_pos = input_pos
        self.output_pos = output_pos
        self.node = DynamicCuboid(
            prim_path=prim_path,
            name=prim_path.split("/")[-1],
            position=input_pos,   # 본체 위치는 input 근처로
            size=PRINTER_SIZE,
            color=color,
            fixed_base=True
        )
        self.current_pallet = None
        self.busy = False

    async def run_cycle(self, world, logger: SimLogger):
        """AGV가 input_pos에 pallet을 놓아두면 실행됨"""
        if self.current_pallet is None:
            return
        self.busy = True
        logger.info(f"{self.prim_path}: LOAD 시작")
        await world.pause_and_wait(PRINTER_LOAD_TIME)

        logger.info(f"{self.prim_path}: BUILD 시작")
        await world.pause_and_wait(PRINTER_BUILD_TIME)

        logger.info(f"{self.prim_path}: UNLOAD 시작 (output_pos로 준비)")
        await world.pause_and_wait(PRINTER_UNLOAD_TIME)
        # 완료: 팔레트가 출력 위치로 이동했다고 가정
        self.current_pallet.at = "printer"  # 상태만 유지 (AGV가 찾아갈 것)
        self.busy = False

    def ready_for_input(self) -> bool:
        return (self.current_pallet is None) and (not self.busy)
