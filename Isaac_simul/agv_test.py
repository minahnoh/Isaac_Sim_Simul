# Isaac_simul/test_agv_move_to_target_fix.py
import sys, asyncio
from pathlib import Path
import numpy as np

# 패키지 경로 추가
PKG_PARENT = str(Path(__file__).resolve().parent.parent)
if PKG_PARENT not in sys.path:
    sys.path.insert(0, PKG_PARENT)

from omni.isaac.core import World
from omni.isaac.core.prims import RigidPrim
import omni.usd
from pxr import UsdPhysics

from Isaac_simul.logger_sim import SimLogger

AGV_ROOT = "/World/AGV___MaxMover__Fork_Over_Leg"   # AGV 루트(Xform) 경로
TARGET_PATH = "/World/agv_target_pos/stacker01_pos" # 타겟 프림 경로
SPEED = 1.0                                        # m/s
ARRIVE_EPS = 0.05                                  # 5cm

def find_first_rigidbody_under(stage, root_path: str, exclude_prefixes=("/World/defaultGroundPlane",)):
    """root_path 하위에서 UsdPhysics.RigidBodyAPI가 붙은 첫 프림을 찾는다."""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    # DFS
    stack = [root]
    while stack:
        p = stack.pop()
        p_path = p.GetPath().pathString
        if not any(p_path.startswith(pref) for pref in exclude_prefixes):
            if p and p.IsValid():
                if UsdPhysics.RigidBodyAPI.CanApply(p) or p.HasAPI(UsdPhysics.RigidBodyAPI):
                    return p
        stack.extend(reversed(p.GetChildren()))
    return None

async def main():
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    world.scene.add_ground_plane()  # ground는 static collider로 남게 됨

    logger = SimLogger(world)
    logger.info("AGV Target 이동(고정: 섀시 rigid body 지정) 시작")

    stage = omni.usd.get_context().get_stage()

    # 타겟 래핑
    target = RigidPrim(prim_path=TARGET_PATH, name="target")
    target.initialize()
    if not target.is_valid():
        logger.error(f"Target 프림을 찾을 수 없습니다: {TARGET_PATH}")
        return

    # AGV 섀시(진짜 rigid body) 프림 찾기
    rb_prim = find_first_rigidbody_under(stage, AGV_ROOT)
    if rb_prim is None:
        logger.error(f"Rigid Body를 찾지 못했습니다(경로 확인/AGV에 RigidBody 적용 필요): {AGV_ROOT}")
        return

    chassis_path = rb_prim.GetPath().pathString
    logger.info(f"AGV 섀시 프림: {chassis_path}")

    agv = RigidPrim(prim_path=chassis_path, name="agv_chassis")
    agv.initialize()
    if not agv.is_valid():
        logger.error(f"AGV 섀시 프림 래핑 실패: {chassis_path}")
        return

    # 시작/타겟 포즈
    start_pos, _ = agv.get_world_pose()
    target_pos, _ = target.get_world_pose()
    logger.info(f"Start Pos: {tuple(round(v,3) for v in start_pos)}")
    logger.info(f"Target Pos: {tuple(round(v,3) for v in target_pos)}")

    # 이동 루프
    while True:
        pos, _ = agv.get_world_pose()
        to_target = np.array(target_pos) - np.array(pos)
        dist = float(np.linalg.norm(to_target))
        if dist < ARRIVE_EPS:
            logger.info(f"목표 지점 도착! (남은 거리 {dist:.3f} m)")
            break

        direction = (to_target / dist) if dist > 1e-6 else np.zeros(3)
        agv.set_linear_velocity(tuple(direction * SPEED))

        world.step(render=True)
        await asyncio.sleep(0)

    # 정지
    agv.set_linear_velocity((0.0, 0.0, 0.0))
    for _ in range(int(0.3 / world.physics_dt)):
        world.step(render=True)
        await asyncio.sleep(0)
    logger.info("AGV 정지 완료")

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
