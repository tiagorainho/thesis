
from pydantic import BaseModel
from models.location_model import Location

class Beacon(BaseModel):
    location: Location
    description: str
    power: float
    beacon_id: str