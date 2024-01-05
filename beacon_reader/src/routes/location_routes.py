
from fastapi import APIRouter, status
from typing import List, Tuple
from models.signal_model import Signal
from config.db import receivers_collection, beacons_collection
from bson.objectid import ObjectId
from services.location_service import triangulate_position
from models.beacons_model import Beacon
from models.signal_model import Signal
from typing import Union
from models.location_model import Location

location_route = APIRouter(prefix="/location")


@location_route.post("/register/{receiver_id}", status_code=status.HTTP_201_CREATED)
async def register(receiver_id: str, signals: list[Signal]) -> Union[dict, None]:
    receiver = receivers_collection.find_one({"_id": ObjectId(receiver_id)})

    connected_beacons: List[Tuple[Beacon, Signal]] = list()
    for signal in signals:
        beacon = beacons_collection.find_one({"beacon_id": signal.beacon_id})
        if beacon is None:
            continue
        connected_beacons.append((beacon, signal))
    
    new_position = triangulate_position(connected_beacons)

    if new_position is None:
        return None
    
    locations: List[dict] = receiver["locations"]
    
    coordinates = {
        "latitude": new_position[0],
        "longitude": new_position[1],
        "height": new_position[2],
    }
    locations.insert(0, coordinates)

    receivers_collection.update_one({"_id": ObjectId(receiver_id)}, {"$set": {"locations": locations}}, True)

    
    return {"location": coordinates}
