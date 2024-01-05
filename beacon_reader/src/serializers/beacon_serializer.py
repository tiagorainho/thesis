from models.beacons_model import Beacon
from typing import List
from serializers.location_serializer import location_serializer

def beacon_serializer(beacon: Beacon) -> dict:
    return {
        "beacon_id": str(beacon["beacon_id"]),
        'location': location_serializer(beacon["location"]),
        'description': beacon["description"],
        "power": beacon["power"]
    }

def beacons_serializer(beacons: List[Beacon]) -> list:
    return [beacon_serializer(beacon) for beacon in beacons]