
from fastapi import APIRouter, status
from serializers.receiver_serializer import receiver_serializer, receivers_serializer
from config.db import receivers_collection
from typing import Union
from bson import ObjectId

receiver_route = APIRouter(prefix="/receiver")

@receiver_route.post("/", status_code=status.HTTP_201_CREATED)
async def register() -> Union[dict, None]:

    _id = receivers_collection.insert_one({
        "locations": list()
    })

    receiver = receivers_collection.find_one({"_id": _id.inserted_id})
    return receiver_serializer(receiver)

@receiver_route.get("/")
async def receivers() -> list:
    receivers = list(receivers_collection.find({}))
    return receivers_serializer(receivers)

@receiver_route.get("/now")
async def now(receiver_id: str) -> Union[dict, None]:
    receiver = receivers_collection.find_one({'_id': ObjectId(receiver_id)})
    if receiver is None:
        return None
    
    locations = receiver["locations"]

    location = receiver["locations"][0] if len(locations) > 0 else None

    return {
        "id": str(receiver["_id"]),
        "last_location": location
    }