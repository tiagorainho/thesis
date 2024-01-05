from pydantic import BaseModel

class Location(BaseModel):
    latitude: float
    longitude: float
    height: float