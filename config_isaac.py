from pxr import UsdGeom, Gf
import omni.usd

# ---- 시뮬레이션 파라미터 ----
SIM_DT = 1.0 / 60.0          # 물리 dt
AGV_SPEED = 1.0              # m/s
AGV_PICK_DROP_TIME = 1.5     # s (집기/놓기 시간)
PRINTER_LOAD_TIME = 1.0      # s
PRINTER_BUILD_TIME = 6.0     # s
PRINTER_UNLOAD_TIME = 1.0    # s

PALLET_CAPACITY = 50         # 한 팔레트당 아이템 최대 개수

# ---- 위치 정의 (m) ----
# 좌표계: X-우/좌, Y-앞/뒤, Z-위/아래
stage = omni.usd.get_context().get_stage()
STACKER1_POS = Gf.Vec3d(496.9256, 0.0,-908.65663)
STACKER2_POS = Gf.Vec3d( 496.9256, 0.0, 318.41164)
PRINTER1_POS = Gf.Vec3d(0.0,  3.0, 0.0)
PRINTER2_POS = Gf.Vec3d(0.0, -3.0, 0.0)


AGV_HOME_POS = Gf.Vec3d(142.0, 0.0, -350.0)

INITIAL_PALLET_PATH=[ {"prim_path": "/World/Plastic_Pallet_01", "id": 1},
                      {"prim_path": "/World/Plastic_Pallet_02", "id": 2},
                      {"prim_path": "/World/Plastic_Pallet_03", "id": 3} ]

PRINTERS_PATH=[ {"prim_path": "/World/_D_Printer_01", "id": 1},
                {"prim_path": "/World/_D_Printer_02", "id": 2}] 

STACKERS_PATH=[ {"prim_path": "/World/Rack___lowpoly_gameready_01", "id": 1},
                {"prim_path": "/World/Rack___lowpoly_gameready_02", "id": 2} ]

AGV_PATH=["/World/AGV___MaxMover__Fork_Over_Leg"]