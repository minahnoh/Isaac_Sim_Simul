import math
import asyncio
from pxr import Gf
from config_sim import PALLET_CAPACITY, AGV_HOME_POS, AGV_PICK_DROP_TIME
from pallet import Pallet
from stacker import Stacker
from printer import Printer
from agv import AGV
from logger_sim import SimLogger

class CellController:
    def __init__(self, world):
        self.world = world
        self.logger = SimLogger(world)

        # 자산 초기화
        self.agv = AGV("/World/AGV___MaxMover__Fork_Over_Leg")
        self.stacker1 = Stacker("/World/Rack___lowpoly_gameready_01", stacker_id=1)
        self.stacker2 = Stacker("/World/Rack___lowpoly_gameready_02", stacker_id=2)
        self.printer1 = Printer("/World/_D_Printer_01", printer_id=1)
        self.printer2 = Printer("/World/_D_Printer_02", printer_id=2)

        self.pallet_id_counter = 1

    def _new_pallet(self, prim_path: str):
        """팔레트 객체 생성"""
        return Pallet(prim_path, self.pallet_id_counter)

    async def _put_to_printer(self, printer: Printer, stacker: Stacker):
        """프린터가 비었을 경우 스태커에서 팔레트 하나 공급"""
        if printer.ready_for_input():
            pallet = stacker.retrieve_pallet()
            if pallet is None:
                return False

            await self.agv.move_to(pallet.position)
            await asyncio.sleep(AGV_PICK_DROP_TIME)
            self.agv.pickup(pallet)

            await self.agv.move_to(printer.node.get_world_pose()[0])
            await asyncio.sleep(AGV_PICK_DROP_TIME)
            self.agv.drop(printer)
            printer.current_pallet = pallet

            # 비동기로 프린터 작동 시작
            self.world.create_task(printer.run_cycle(self.world, self.logger))
            return True
        return False

    async def _collect_pallet(self, printer: Printer, target_stacker: Stacker):
        """프린터 출력이 끝난 팔레트를 회수하여 스태커에 저장"""
        if (not printer.busy) and printer.current_pallet:
            pallet = printer.current_pallet
            await self.agv.move_to(printer.node.get_world_pose()[0])
            await asyncio.sleep(AGV_PICK_DROP_TIME)
            self.agv.pickup(pallet)

            await self.agv.move_to(target_stacker.position)
            await asyncio.sleep(AGV_PICK_DROP_TIME)
            self.agv.drop(target_stacker)
            target_stacker.store_pallet(pallet)
            printer.current_pallet = None
            return True
        return False

    async def run_factory(self):
        """공장 전체 흐름 실행"""
        # 초기 프린터 채우기
        await self._put_to_printer(self.printer1, self.stacker1)
        await self._put_to_printer(self.printer2, self.stacker1)

        while True:
            changed = False

            # 완료된 프린터에서 팔레트 수거
            changed |= await self._collect_pallet(self.printer1, self.stacker2)
            changed |= await self._collect_pallet(self.printer2, self.stacker2)

            # 빈 프린터에 새 팔레트 공급
            changed |= await self._put_to_printer(self.printer1, self.stacker1)
            changed |= await self._put_to_printer(self.printer2, self.stacker1)

            # 변화가 없으면 한 틱 대기
            if not changed:
                await self.world.step_async()

            # 종료 조건: 모든 프린터 비어있고, 스태커도 비어있을 때
            if not any([self.stacker1.slots,
                        self.printer1.current_pallet, self.printer2.current_pallet]):
                break

        self.logger.info("작업 완료, AGV 복귀")
        await self.agv.move_to(AGV_HOME_POS)
