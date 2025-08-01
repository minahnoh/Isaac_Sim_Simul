# main.py
import asyncio
from omni.isaac.kit import SimulationApp
# Isaac Sim 실행 (필요 시 headless=True)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

from controller import CellController
from logger_sim import SimLogger
from config_sim import SIM_DT

class WorldWrapper:
    """World + 편의 async 유틸리티"""
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0, physics_dt=SIM_DT, rendering_dt=SIM_DT)
        self.world.scene.add_default_ground_plane()

        # Isaac Sim의 Task 스케줄용 컨테이너
        self._tasks = []

    @property
    def current_time(self) -> float:
        return self.world.current_time

    def get_physics_dt(self) -> float:
        return self.world.get_physics_dt()

    async def step_async(self):
        self.world.step(render=True)
        await asyncio.sleep(0)

    async def pause_and_wait(self, seconds: float):
        target = self.current_time + seconds
        while self.current_time < target:
            self.world.step(render=True)
            await asyncio.sleep(0)

    def create_task(self, coro):
        self._tasks.append(asyncio.create_task(coro))

async def main_async():
    w = WorldWrapper()
    logger = SimLogger(w)

    # 셀 컨트롤러 구성
    cell = CellController(w)

    # 예시: 주문 총 아이템 수 (나중에 SimPy 연동 시 여기로 전달)
    total_items = 180  # → 팔레트 4개 (50,50,50,30)
    await cell.run_order_flow(total_items)

    logger.info("시뮬레이션 종료 대기(2s)")
    await w.pause_and_wait(2.0)

if __name__ == "__main__":
    try:
        asyncio.get_event_loop().run_until_complete(main_async())
    finally:
        simulation_app.close()
