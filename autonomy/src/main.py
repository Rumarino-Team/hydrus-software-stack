from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Tuple

from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],  
    allow_headers=["*"], 
)

# ==============================
# Pydantic Models (For API Schema)
# ==============================

class Point3D(BaseModel):
    x: float
    y: float
    z: float

class ColorFilter(BaseModel):
    tolerance: float
    min_confidence: float
    min_area: float
    rgb_range: Tuple[int,int,int] = (0,0,0)

class Detection(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float
    cls: int
    conf: float
    depth: float = 0
    point: Optional[Point3D] = None
    colorFilter: Optional[ColorFilter] = None


class Detections(BaseModel):
    detections: List[Detection]
    detector_name: str

# ==============================
# Temporary In-Memory Storage
# ==============================
detections_db: List[Detection] = []

# ==============================
# API Endpoints
# ==============================

@app.get("/")
def read_root():
    return {"message": "Welcome to the Detection API"}

# 1️⃣ Get all detections
@app.get("/detections", response_model=List[Detection])
def get_detections():
    return detections_db

# 2️⃣ Get a single detection by index
@app.get("/detections/{index}", response_model=Detection)
def get_detection(index: int):
    if index < 0 or index >= len(detections_db):
        raise HTTPException(status_code=404, detail="Detection not found")
    return detections_db[index]

# 3️⃣ Add new detections
@app.post("/detections", response_model=Detection)
def add_detection(detection: Detection):
    detections_db.append(detection)
    return detection

# 4️⃣ Add multiple detections at once
@app.post("/detections/batch", response_model=Detections)
def add_multiple_detections(detections: Detections):
    detections_db.extend(detections.detections)
    return detections
