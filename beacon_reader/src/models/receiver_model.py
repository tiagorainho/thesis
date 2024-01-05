from pydantic import BaseModel
from models.location_model import Location

class Receiver(BaseModel):
    locations: list[Location]