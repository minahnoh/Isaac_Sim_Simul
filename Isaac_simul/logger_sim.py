class SimLogger:
    def __init__(self, world):
        self.world = world

    def _get_timestamp(self):
        current_time = self.world.current_time  # 시간 단위: 분
        days = int(current_time // (24 * 60))
        hours = int((current_time % (24 * 60)) // 60)
        minutes = int(current_time % 60)
        timestamp = f"{days:02d}:{hours:02d}:{minutes:02d}"
        total_minutes = int(current_time)
        return timestamp, total_minutes

    def info(self, message: str, event_type: str = "INFO"):
        timestamp, total_minutes = self._get_timestamp()
        print(f"[{timestamp}] [{total_minutes}] | {event_type}: {message}")
