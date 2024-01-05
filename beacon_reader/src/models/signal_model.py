from pydantic import BaseModel

class Signal(BaseModel):
    beacon_id: str
    power_received: float