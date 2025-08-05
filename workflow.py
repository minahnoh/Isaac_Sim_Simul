import math
import asyncio
from config_isaac import *
from pallet import Pallet
from stacker import Stacker
from printer import Printer
from agv import AGV
from logger_sim import SimLogger

class Workflow:
    def __init__(self, world):
        self.world = world
        self.logger = SimLogger(world)

        # 자산 초기화
        self.agv = AGV("/World/AGV___MaxMover__Fork_Over_Leg")
        self.stackers = [self.create_stacker(data["prim_path"], data["id"]) for data in STACKERS_PATH]
        self.printers = [self.create_printer(data["prim_path"], data["id"]) for data in PRINTERS_PATH]
        self.pallets = [self.create_pallet(data["prim_path"], data["id"]) for data in INITIAL_PALLET_PATH]

        # stacker1에 초기 팔레트 적재
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
            pallet = self.stackers[0].empty_slot()
            if not pallet:
                return False
            await self.agv.pickup_from_stacker(self.stackers[0])
            await self.agv.drop_to_printer(printer)
            self.world.create_task(printer.start_printing(self.world, self.logger))
            return True
        return False

    async def _collect_from_printer(self, printer: Printer):
        if printer.current_pallet and not printer.state == 'working':
            await self.agv.pickup_from_printer(printer)
            await self.agv.drop_to_stacker(self.stackers[1])
            return True
        return False

    async def run_factory(self):
        # 초기 프린터 채우기
        await self._put_to_printer(self.printers[0])
        await self._put_to_printer(self.printers[1])

        while True:
            # 작업상태 변경 여부 
            changed = False

            # 프린터 작업 완료 시 회수 및 stacker2에 저장
            for printer in self.printers:
                changed |= await self._collect_from_printer(printer)

            # 프린터가 비었으면 stacker1에서 pallet 공급
            for printer in self.printers:
                changed |= await self._put_to_printer(printer)

            # 종료 조건 확인
            if (not self.stackers[0].slots and
                all(p.current_pallet is None for p in self.printers)):
                break

            if not changed:
                await self.world.step_async()

        self.logger.info("작업 완료, AGV 복귀")
        await self.agv.move_to(AGV_HOME_POS)
