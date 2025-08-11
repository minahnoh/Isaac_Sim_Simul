# --- TEMP DEBUG header (put at the very top of workflow.py) ---
import sys, importlib.util
from pathlib import Path
PKG_PARENT = str(Path(__file__).resolve().parent.parent)  # e.g., c:\isaacsim
if PKG_PARENT not in sys.path:
    sys.path.insert(0, PKG_PARENT)

print("[DEBUG] __file__:", __file__)
print("[DEBUG] sys.path[0:3]:", sys.path[:3])
print("[DEBUG] spec Isaac_simul.config_isaac:", importlib.util.find_spec("Isaac_simul.config_isaac"))
print("[DEBUG] spec config_isaac:", importlib.util.find_spec("config_isaac"))
# --------------------------------------------------------------

import asyncio
from Isaac_simul.config_isaac import *
from Isaac_simul.pallet import Pallet
from Isaac_simul.stacker import Stacker
from Isaac_simul.printer import Printer
from Isaac_simul.agv import AGV
from Isaac_simul.logger_sim import SimLogger


class Workflow:
    def __init__(self, world):
        self.world = world
        self.logger = SimLogger(world)

        # 자산 초기화
        self.agv = AGV("/World/AGV___MaxMover__Fork_Over_Leg")
        self.stackers = [self.create_stacker(data["prim_path"], data["id"]) for data in STACKERS_PATH]
        self.printers = [self.create_printer(data["prim_path"], data["id"]) for data in PRINTERS_PATH]
        self.pallets = [self.create_pallet(data["prim_path"], data["id"]) for data in INITIAL_PALLET_PATH]

        # stacker1에 초기 팔레트 적재 (stacker가 존재할 때만)
        if self.stackers:
            for pallet in self.pallets:
                self.stackers[0].fill_slot_with_pallet(pallet)

    def create_stacker(self, prim_path: str, id: int):
        return Stacker(prim_path, id)

    def create_printer(self, prim_path: str, id: int):
        return Printer(prim_path, id)

    def create_pallet(self, prim_path: str, pallet_id: int):
        return Pallet(prim_path, pallet_id)

    async def _put_to_printer(self, printer: Printer):
        if printer.ready_for_input():
            # stacker1에서 꺼낼 수 없으면 패스
            if not self.stackers or not self.stackers[0].slots:
                return False

            pallet = self.stackers[0].empty_slot()
            if not pallet:
                return False

            await self.agv.pickup_from_stacker(self.stackers[0])
            await self.agv.drop_to_printer(printer)

            # world에 create_task가 없으면 asyncio.create_task로 폴백
            create_task = getattr(self.world, "create_task", asyncio.create_task)
            create_task(printer.start_printing(self.world, self.logger))
            return True
        return False

    async def _collect_from_printer(self, printer: Printer):
        if printer.current_pallet and printer.state != 'working':
            await self.agv.pickup_from_printer(printer)

            # stacker2가 없으면 stacker1에 되돌려두기
            if len(self.stackers) > 1:
                await self.agv.drop_to_stacker(self.stackers[1])
            else:
                await self.agv.drop_to_stacker(self.stackers[0])
            return True
        return False

    async def run_factory(self):
        # 초기 프린터 채우기
        if len(self.printers) > 0:
            await self._put_to_printer(self.printers[0])
        if len(self.printers) > 1:
            await self._put_to_printer(self.printers[1])

        while True:
            changed = False

            # 프린터 작업 완료 시 회수 및 stacker2(없으면 stacker1)에 저장
            for printer in self.printers:
                changed |= await self._collect_from_printer(printer)

            # 프린터가 비었으면 stacker1에서 pallet 공급
            for printer in self.printers:
                changed |= await self._put_to_printer(printer)

            # 종료 조건: stacker1 비었고, 모든 프린터가 비어있음
            if (not (self.stackers and self.stackers[0].slots) and
                all(p.current_pallet is None for p in self.printers)):
                break

            if not changed:
                # Isaac World API가 있으면 사용, 없으면 작은 sleep
                if hasattr(self.world, "step_async"):
                    await self.world.step_async()
                else:
                    await asyncio.sleep(0.0)

        self.logger.info("작업 완료, AGV 복귀")
        await self.agv.move_to(AGV_HOME_POS)
