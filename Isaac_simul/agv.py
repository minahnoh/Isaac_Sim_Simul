import asyncio
import omni.usd
from pxr import Gf, UsdGeom
from omni.isaac.core.prims import RigidPrim # RigidPrim: Isaac Sim에서 물리기반 오브젝트(Rigid Body)를 제어하기 위한 도우미 클래스
from omni.isaac.core.utils.stage import get_current_stage # 현재 USD Stage를 갖고옴
from omni.isaac.core.utils.prims import get_prim_at_path  # 특정 경로에 있는 Prim(객체)를 갖고옴
from Isaac_simul.printer import Printer
from Isaac_simul.stacker import Stacker
from Isaac_simul.config_isaac import *


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
        self.body = RigidPrim(prim_paths_expr=self.prim_path) # RigidPrim 객체를 생성해서 해당 AGV에 물리엔진 기능 부여
        self.max_speed = max_speed
        self.stop_distance = stop_distance
        self.carrying = None # AGV가 팔레트를 들고 있을 경우 어떤 팔레트를 들고 있는지

    def get_position(self):
        """현재 AGV의 월드 위치 반환"""
        xform = UsdGeom.Xformable(self.prim)
        matrix = xform.ComputeLocalToWorldTransform(0)
        return Gf.Vec3f(matrix.ExtractTranslation())

    def stop(self):
        """AGV 멈춤"""
        self.body.set_linear_velocity(Gf.Vec3f(0, 0, 0)) # 현재 속도를 (0,0,0)으로 설정해서 이동 정지, set_linear_velocity(): 여러 개의 물리 객체(prim)의 속도를 한꺼번에 제어할때 사용하는 함수

    async def move_to(self, target_pos: Gf.Vec3f): # async을 통해 agv를 비동기적으로 움직임
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

            velocity = direction.GetNormalized() * self.max_speed # 목표 방향으로 속도 벡터 계산(정규화된 방향 * 속도)
            self.body.set_linear_velocity(velocity) # 물리 객체에 적용
            await asyncio.sleep(update_dt)

    def pickup(self, pallet):
        """팔레트 연결"""
        self.carrying = pallet
        print(f"[AGV] 팔레트 {pallet.id} 집어올림 ")

    def drop(self, pallet):
        """팔레트 해제"""
        if self.carrying:
            print(f"[AGV] 팔레트 {pallet.id} 내려놓음")
            self.carrying = None

    async def pickup_from_stacker(self, stacker: Stacker):
        """stacker로부터 pallet을 들어 올림"""
        pallet = stacker.empty_slot()
        if pallet is None:
            return None
        await self.move_to(stacker.position)
        await asyncio.sleep(AGV_PICK_DROP_TIME)
        self.pickup(pallet)
        print(f"[AGV] Stacker{stacker.id_stacker}에서 Pallet{pallet.id}를 꺼냄")
        return pallet

    async def drop_to_stacker(self, stacker: Stacker):
        if self.carrying is None:
            return False
        await self.move_to(stacker.position)
        await asyncio.sleep(AGV_PICK_DROP_TIME)
        stacker.fill_slot_with_pallet(self.carrying)
        self.drop(self.carrying)
        print(f"[AGV] Stacker {stacker.id_stacker}에 Pallet {self.carrying.id} 드롭 완료")
        return True

    async def pickup_from_printer(self, printer: Printer):
        if printer.current_pallet is None or printer.state == 'working':
            return None
        pallet = printer.current_pallet
        await self.move_to(printer.node.get_world_pose()[0])
        await asyncio.sleep(AGV_PICK_DROP_TIME)
        self.pickup(pallet)
        printer.current_pallet = None
        return pallet

    async def drop_to_printer(self, printer: Printer):
        if self.carrying is None or not printer.ready_for_input():
            return False
        await self.move_to(printer.node.get_world_pose()[0])
        await asyncio.sleep(AGV_PICK_DROP_TIME)
        printer.current_pallet = self.carrying
        self.drop(self.carrying)
        return True
    
