from typing import List

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from .database import Database
from .docker_manager import DockerManager

# Tailscale IP constant
TAILSCALE_IP = "100.76.98.95"

app = FastAPI()

db = Database()
docker_manager = DockerManager()

class UserRequest(BaseModel):
    name: str | None = None
    email: str
    password: str

class ContainerRequest(BaseModel):
    email: str
    password: str
    image_tag: str = "linuxserver/openssh-server"

class ContainerInfo(BaseModel):
    id: str
    port: int
    password: str
    image_tag: str
    ssh_command: str
    web_urls: dict = {}

class ImageInfo(BaseModel):
    id: str
    tag: str
    size: int
    created: str

@app.post("/register")
def register(req: UserRequest):
    if not req.name:
        raise HTTPException(status_code=400, detail="Name required")
    if not db.add_user(req.name, req.email, req.password):
        raise HTTPException(status_code=400, detail="Email already registered")
    return {"message": "registered"}

@app.post("/login")
def login(req: UserRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    return {"message": "success"}

@app.get("/images", response_model=List[ImageInfo])
def list_images(email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    try:
        images = docker_manager.list_available_images()
        return [ImageInfo(**img) for img in images]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list images: {str(e)}")

@app.get("/images/{image_tag:path}")
def get_image_info(image_tag: str, email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    try:
        info = docker_manager.get_image_info(image_tag)
        if not info:
            raise HTTPException(status_code=404, detail="Image not found")
        return info
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get image info: {str(e)}")

@app.post("/container", response_model=ContainerInfo)
def create_container(req: ContainerRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    try:
        cid, docker_id, port, password, image_tag = docker_manager.create_container(user_id, req.image_tag)
        db.add_container(cid, user_id, docker_id, port, password, image_tag)
        
        # Generate SSH command and web URLs
        ssh_command = f"ssh dev@{TAILSCALE_IP} -p {port}"
        web_urls = {}
        
        if "hydrus" in image_tag.lower():
            web_urls = {
                "web_interface": f"http://{TAILSCALE_IP}:8000",
                "detection_viewer": f"http://{TAILSCALE_IP}:5000"
            }
        
        return ContainerInfo(
            id=cid, 
            port=port, 
            password=password, 
            image_tag=image_tag,
            ssh_command=ssh_command,
            web_urls=web_urls
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/container", response_model=List[ContainerInfo])
def list_containers(email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    rows = db.list_containers(user_id)
    
    containers = []
    for r in rows:
        ssh_command = f"ssh dev@{TAILSCALE_IP} -p {r[2]}"
        web_urls = {}
        
        if "hydrus" in r[4].lower():
            web_urls = {
                "web_interface": f"http://{TAILSCALE_IP}:8000",
                "detection_viewer": f"http://{TAILSCALE_IP}:5000"
            }
        
        containers.append(ContainerInfo(
            id=r[0], 
            port=r[2], 
            password=r[3], 
            image_tag=r[4],
            ssh_command=ssh_command,
            web_urls=web_urls
        ))
    
    return containers

@app.delete("/container/{cid}")
def delete_container(cid: str, email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    rows = db.list_containers(user_id)
    for r in rows:
        if r[0] == cid:
            docker_manager.remove_container(r[1])
            db.delete_container(user_id, cid)
            return {"message": "deleted"}
    raise HTTPException(status_code=404, detail="Not found")