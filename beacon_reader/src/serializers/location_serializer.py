from models.location_model import Location
from typing import List

def location_serializer(location: Location) -> dict:
    return {
        'latitude': location["latitude"],
        'longitude': location["longitude"],
        'height': location["height"],
    }

def locations_serializer(locations: List[Location]) -> list:
    return [location_serializer(location) for location in locations]