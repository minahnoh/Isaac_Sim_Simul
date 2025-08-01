# logger_sim.py
class SimLogger:
    def __init__(self, world):
        self.world = world

    def ts(self) -> str:
        t = self.world.current_time
        return f"{t:7.2f}s"

    def info(self, msg: str):
        print(f"[{self.ts()}] {msg}")
