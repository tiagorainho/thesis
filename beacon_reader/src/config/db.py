from pymongo import MongoClient

db_connection = MongoClient("mongodb://localhost:27017")
db = db_connection.safe_tracker

receivers_collection = db["receivers"]
beacons_collection = db["beacons"]