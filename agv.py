# agv.py
import asyncio
import omni.usd
from pxr import Gf, UsdGeom
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path


class AGV:
    def __init__(self, prim_path: str, max_speed: float = 100.0, stop_distance: float = 1.0):
        """
        물리 기반 AGV 생성자

        Args:
            prim_path: AGV의 Prim 경로
            max_speed: 최대 속도 (cm/s)
            stop_distance: 목표 위치와 이 거리 이내로 가까워지면 정지
        """
        self.prim_path = prim_path
        self.stage = get_current_stage()
        self.prim = get_prim_at_path(self.prim_path)
        self.body = RigidPrim(prim_path=self.prim_path)
        self.max_speed = max_speed
        self.stop_distance = stop_distance
        self.carrying = None

    def get_position(self):
        """현재 AGV의 월드 위치 반환"""
        xform = UsdGeom.Xformable(self.prim)
        matrix = xform.ComputeLocalToWorldTransform(0)
        return Gf.Vec3f(matrix.ExtractTranslation())

    def stop(self):
        """AGV 멈춤"""
        self.body.set_linear_velocity(Gf.Vec3f(0, 0, 0))

    async def move_to(self, target_pos: Gf.Vec3f):
        """
        물리 기반 이동 - 목표 위치까지 속도 적용

        Args:
            target_pos: 이동 목표 위치 (Gf.Vec3f)
        """
        print(f"[AGV] 이동 시작 → {target_pos}")
        reached = False
        update_dt = 1 / 60.0  # 시뮬레이션 주기

        while not reached:
            current_pos = self.get_position()
            direction = target_pos - current_pos
            distance = direction.GetLength()

            if distance < self.stop_distance:
                self.stop()
                print(f"[AGV] 도착 완료. 남은 거리: {distance:.2f}")
                reached = True
                break

            velocity = direction.GetNormalized() * self.max_speed
            self.body.set_linear_velocity(velocity)
            await asyncio.sleep(update_dt)

    def pickup(self, pallet):
        """팔레트 연결"""
        self.carrying = pallet
        print(f"[AGV] 팔레트 {pallet.prim_path} 집어올림 (추후 joint 연결 가능)")

    def drop(self, stacker):
        """팔레트 해제"""
        if self.carrying:
            print(f"[AGV] 팔레트 {self.carrying.prim_path} 내려놓음")
            self.carrying = None
