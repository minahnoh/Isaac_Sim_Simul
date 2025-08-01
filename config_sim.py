# config_sim.py
from pxr import Gf

# ---- 시뮬레이션 파라미터 ----
SIM_DT = 1.0 / 60.0          # 물리 dt
AGV_SPEED = 1.0              # m/s
AGV_PICK_DROP_TIME = 1.5     # s (집기/놓기 시간)
PRINTER_LOAD_TIME = 1.0      # s
PRINTER_BUILD_TIME = 6.0     # s
PRINTER_UNLOAD_TIME = 1.0    # s

PALLET_CAPACITY = 50         # 한 팔레트당 아이템 최대 개수
PALLET_SIZE = (0.6, 0.4, 0.2)  # meters (x,y,z)
AGV_SIZE = (0.8, 0.6, 0.3)
STACKER_SIZE = (0.7, 0.7, 1.2)
PRINTER_SIZE = (1.2, 1.0, 1.2)

# ---- 위치 정의 (m) ----
# 좌표계: X-우/좌, Y-앞/뒤, Z-위/아래
STACKER1_POS = Gf.Vec3d(-2.0, 0.0, 0.0)
STACKER2_POS = Gf.Vec3d( 2.0, 0.0, 0.0)
PRINTER1_INPUT_POS = Gf.Vec3d(0.0,  3.0, 0.0)
PRINTER1_OUTPUT_POS= Gf.Vec3d(0.8, 3.0, 0.0)
PRINTER2_INPUT_POS = Gf.Vec3d(0.0, -3.0, 0.0)
PRINTER2_OUTPUT_POS= Gf.Vec3d(0.8,-3.0, 0.0)

AGV_HOME_POS = Gf.Vec3d(-2.0, -2.0, 0.0)

# 팔레트 시각 오프셋(AGV에 실었을 때)
AGV_PALLET_OFFSET = Gf.Vec3d(0.0, 0.0, PALLET_SIZE[2] * 0.6)
