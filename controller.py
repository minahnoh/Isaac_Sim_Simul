# controller.py
import math
from logger_sim import SimLogger
from config_sim import PALLET_CAPACITY, STACKER1_POS, STACKER2_POS, PRINTER1_INPUT_POS, PRINTER1_OUTPUT_POS, PRINTER2_INPUT_POS, PRINTER2_OUTPUT_POS, AGV_HOME_POS
from pallet import Pallet
from stacker import Stacker
from printer import Printer
from agv import AGV
from pxr import Gf

class CellController:
    def __init__(self, world):
        self.world = world
        self.logger = SimLogger(world)

        # 자산 생성
        self.agv = AGV("/World/AGV", AGV_HOME_POS)
        self.stacker1 = Stacker("/World/Stacker01", STACKER1_POS)
        self.stacker2 = Stacker("/World/Stacker02", STACKER2_POS)
        self.printer1 = Printer("/World/Printer01", PRINTER1_INPUT_POS, PRINTER1_OUTPUT_POS)
        self.printer2 = Printer("/World/Printer02", PRINTER2_INPUT_POS, PRINTER2_OUTPUT_POS)

        self.pallet_id_counter = 1

    def _new_pallet(self, pos: Gf.Vec3d, items: int) -> Pallet:
        p = Pallet(f"/World/Pallet_{self.pallet_id_counter:03d}", pos, items=items)
        self.pallet_id_counter += 1
        return p

    def plan_pallets_for_order(self, total_items: int):
        """주문 아이템 수 → 팔레트 수 및 각 팔레트 적재량 분배"""
        pallets = []
        n = math.ceil(total_items / PALLET_CAPACITY)
        remain = total_items
        for i in range(n):
            take = min(PALLET_CAPACITY, remain)
            pos = STACKER1_POS if i % 2 == 0 else STACKER2_POS
            p = self._new_pallet(pos, take)
            pallets.append(p)
            # 스태커에 적재
            if i % 2 == 0:
                self.stacker1.push(p)
            else:
                self.stacker2.push(p)
            remain -= take
        self.logger.info(f"주문 분해 → 팔레트 {len(pallets)}개 생성 (총 {total_items} 아이템)")
        return pallets

    async def _feed_printer_if_available(self, printer, stacker):
        """프린터가 비어있고 스태커에 팔레트가 있으면 하나 공급"""
        if printer.ready_for_input() and stacker.has_pallet():
            p = stacker.pop()
            # 스태커 → 프린터 투입
            await self.agv.move_to(self.world, self.logger, self.agv.get_world_pos() if self.agv.carrying else p.node.get_world_pose()[0])
            if self.agv.carrying is None:
                # 픽업 위치까지
                await self.agv.move_to(self.world, self.logger, p.node.get_world_pose()[0])
                await self.agv.pick(self.world, self.logger, p)

            await self.agv.move_to(self.world, self.logger, printer.input_pos)
            await self.agv.drop(self.world, self.logger, "printer_input")
            printer.current_pallet = p
            # 프린터 사이클 시작(백그라운드)
            self.world.create_task(printer.run_cycle(self.world, self.logger))
            return True
        return False

    async def _collect_from_printer(self, printer, target_stacker):
        """프린터 완료 팔레트 회수 → 다른 스태커로 반출"""
        if (not printer.busy) and (printer.current_pallet is not None):
            p = printer.current_pallet
            # 출력 위치로 AGV 이동(간단히 input/출력 근접 동일 취급)
            await self.agv.move_to(self.world, self.logger, printer.output_pos)
            await self.agv.pick(self.world, self.logger, p)
            printer.current_pallet = None

            # 빈 스태커로 이동하여 드롭
            await self.agv.move_to(self.world, self.logger, target_stacker.base_pos)
            await self.agv.drop(self.world, self.logger, "stacker")
            target_stacker.push(p)
            p.at = "stacker"
            return True
        return False

    async def run_order_flow(self, total_items: int):
        pallets = self.plan_pallets_for_order(total_items)

        # 초기: 두 프린터를 우선 채우기
        await self._feed_printer_if_available(self.printer1, self.stacker1 if self.stacker1.has_pallet() else self.stacker2)
        await self._feed_printer_if_available(self.printer2, self.stacker2 if self.stacker2.has_pallet() else self.stacker1)

        # 메인 루프: 프린터 완료 수거 → 빈 프린터에 새 팔레트 투입 (2대 제한)
        while any([self.stacker1.has_pallet(), self.stacker2.has_pallet()]) or \
              (self.printer1.current_pallet is not None) or \
              (self.printer2.current_pallet is not None):

            # 1) 완료 프린터 회수
            changed = False
            changed |= await self._collect_from_printer(self.printer1, self.stacker2)
            changed |= await self._collect_from_printer(self.printer2, self.stacker1)

            # 2) 빈 프린터에 새 팔레트 투입
            changed |= await self._feed_printer_if_available(self.printer1, self.stacker1 if self.stacker1.has_pallet() else self.stacker2)
            changed |= await self._feed_printer_if_available(self.printer2, self.stacker2 if self.stacker2.has_pallet() else self.stacker1)

            if not changed:
                # 변화 없으면 한 틱 진행
                await self.world.step_async()

        self.logger.info("주문 처리 완료. AGV 복귀.")
        # 홈 복귀
        await self.agv.move_to(self.world, self.logger, AGV_HOME_POS)
