from omni.isaac.core.objects import FixedCuboid
from pxr import Gf, UsdGeom
import omni.usd
from config_sim import PRINTER_LOAD_TIME, PRINTER_BUILD_TIME, PRINTER_UNLOAD_TIME
from logger_sim import SimLogger
import asyncio
from pallet import Pallet

class Printer:
    """
        Class representing an 3D Printer in the system

        Attributes:
            prim_path (str): USD Stage 상의 프린터 위치 경로
            printer_id (int | str): 프린터 고유 ID
            state: 프린터 현재 상태: 'idle', 'working', 또는 'error
            current_pallet: 현재 프린터에 적재된 팔레트 객체 (없으면 None)
        """
    
    def __init__(self, prim_path: str, printer_id):
        self.prim_path = prim_path
        self.id = printer_id
        self.state = 'idle' # state: 'idle', 'working', 'error'
        self.current_pallet = None  # 현재 프린터에 놓인 팔레트
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)
        matrix = xform.ComputeLocalToWorldTransform(0)
        position = Gf.Vec3f(matrix.ExtractTranslation())
        
        # 프린터 본체는 고정형으로 생성
        self.node = FixedCuboid(
            prim_path=prim_path,
            position=position
        )

    async def start_printing(self, world, logger: SimLogger):
        if self.current_pallet is None:
            return

        self.state = 'working'
        logger.info(f"3D 프린터 {self.id}: LOAD")
        await asyncio.sleep(PRINTER_LOAD_TIME)

        logger.info(f"3D 프린터 {self.id}: BUILD")
        await asyncio.sleep(PRINTER_BUILD_TIME)

        logger.info(f"3D 프린터 {self.id}: UNLOAD")
        await asyncio.sleep(PRINTER_UNLOAD_TIME)

        self.state = 'idle'
        self.current_pallet = None

    def ready_for_input(self):
        return (self.current_pallet is None) and (self.state == 'idle')