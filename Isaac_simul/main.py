# Isaac_simul/main.py
# VS Code Edition 내부에서 실행하는 러너 (SimulationApp 생성 금지)

import sys
from pathlib import Path

# --- ensure package imports work no matter where we run from ---
PKG_PARENT = str(Path(__file__).resolve().parent.parent)  # e.g., c:\isaacsim
if PKG_PARENT not in sys.path:
    sys.path.insert(0, PKG_PARENT)
# --------------------------------------------------------------

import asyncio
from omni.isaac.core import World
from Isaac_simul.workflow import Workflow
from Isaac_simul.logger_sim import SimLogger

class WorldWrapper:
    def __init__(self):
        # 앱이 이미 떠 있으므로 바로 World 생성 가능
        self.world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
        self.world.scene.add_ground_plane()
        self._tasks = []

    @property
    def current_time(self) -> float:
        return self.world.current_time

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
    cell = Workflow(w)
    await cell.run_factory()
    logger.info("시뮬레이션 종료 대기(2s)")
    await w.pause_and_wait(2.0)

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main_async())
