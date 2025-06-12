from typing import List

import os
from fastapi import FastAPI, HTTPException, Request, Form
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from starlette.middleware.sessions import SessionMiddleware
from pydantic import BaseModel

from .database import Database
from .docker_manager import DockerManager

# Tailscale IP constant
TAILSCALE_IP = "100.76.98.95"

app = FastAPI()

templates = Jinja2Templates(directory=os.path.join(os.path.dirname(__file__), "templates"))
app.add_middleware(SessionMiddleware, secret_key="ssh-docker-server-secret")

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

class ComposeRequest(BaseModel):
    email: str
    password: str
    compose_name: str = "openssh"

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


@app.get("/", response_class=HTMLResponse)
def home(request: Request):
    return templates.TemplateResponse("login.html", {"request": request, "message": None})


@app.post("/login", response_class=HTMLResponse)
def login_web(request: Request, email: str = Form(...), password: str = Form(...)):
    user_id = db.verify_user(email, password)
    if not user_id:
        return templates.TemplateResponse(
            "login.html",
            {"request": request, "message": "Invalid credentials"},
            status_code=401,
        )
    request.session["email"] = email
    request.session["password"] = password
    return RedirectResponse("/dashboard", status_code=303)


@app.post("/register", response_class=HTMLResponse)
def register_web(
    request: Request,
    name: str = Form(...),
    email: str = Form(...),
    password: str = Form(...),
):
    if not db.add_user(name, email, password):
        return templates.TemplateResponse(
            "login.html",
            {"request": request, "message": "Email already registered"},
            status_code=400,
        )
    request.session["email"] = email
    request.session["password"] = password
    return RedirectResponse("/dashboard", status_code=303)


@app.post("/logout")
def logout(request: Request):
    request.session.clear()
    return RedirectResponse("/", status_code=303)


@app.get("/dashboard", response_class=HTMLResponse)
def dashboard(request: Request):
    email = request.session.get("email")
    password = request.session.get("password")
    if not email or not password:
        return RedirectResponse("/", status_code=303)
    user_id = db.verify_user(email, password)
    if not user_id:
        return RedirectResponse("/", status_code=303)

    rows = db.list_containers(user_id)
    containers = [
        {
            "id": r[0],
            "docker_id": r[1],
            "port": r[2],
            "password": r[3],
            "image_tag": r[4],
        }
        for r in rows
    ]
    images = docker_manager.list_available_images()
    return templates.TemplateResponse(
        "dashboard.html",
        {
            "request": request,
            "email": email,
            "containers": containers,
            "images": images,
            "tailscale_ip": TAILSCALE_IP,
        },
    )


@app.post("/create")
def create_container_web(request: Request, image_tag: str = Form(...)):
    email = request.session.get("email")
    password = request.session.get("password")
    if not email or not password:
        return RedirectResponse("/", status_code=303)
    user_id = db.verify_user(email, password)
    if not user_id:
        return RedirectResponse("/", status_code=303)
    try:
        cid, docker_id, port, passwd, img_tag, info = docker_manager.create_container(
            user_id, image_tag
        )
        db.add_container(
            cid,
            user_id,
            docker_id,
            port,
            passwd,
            img_tag,
            info["ssh_ready"],
            info.get("funnel_port"),
        )
    except Exception:
        pass
    return RedirectResponse("/dashboard", status_code=303)


@app.post("/delete")
def delete_container_web(request: Request, cid: str = Form(...)):
    email = request.session.get("email")
    password = request.session.get("password")
    if not email or not password:
        return RedirectResponse("/", status_code=303)
    user_id = db.verify_user(email, password)
    if not user_id:
        return RedirectResponse("/", status_code=303)
    rows = db.list_containers(user_id)
    for r in rows:
        if r[0] == cid:
            docker_manager.remove_container(r[1])
            db.delete_container(user_id, cid)
            break
    return RedirectResponse("/dashboard", status_code=303)

@app.post("/api/register")
def register(req: UserRequest):
    if not req.name:
        raise HTTPException(status_code=400, detail="Name required")
    if not db.add_user(req.name, req.email, req.password):
        raise HTTPException(status_code=400, detail="Email already registered")
    return {"message": "registered"}

@app.post("/api/login")
def login(req: UserRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    return {"message": "success"}

@app.get("/api/images", response_model=List[ImageInfo])
def list_images(email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    try:
        images = docker_manager.list_available_images()
        return [ImageInfo(**img) for img in images]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list images: {str(e)}")

@app.get("/compose", response_model=List[str])
def list_compose(email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    return docker_manager.list_available_compose()

@app.get("/api/images/{image_tag:path}")
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

@app.post("/api/container", response_model=ContainerInfo)
def create_container(req: ContainerRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    
    try:
        cid, docker_id, port, password, image_tag, info = docker_manager.create_container(user_id, req.image_tag)
        db.add_container(cid, user_id, docker_id, port, password, image_tag, info["ssh_ready"], info.get("funnel_port"))
        
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

@app.post("/compose", response_model=ContainerInfo)
def create_compose(req: ComposeRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")

    try:
        cid, docker_id, port, password, name = docker_manager.create_container_from_compose(
            user_id, req.compose_name
        )
        db.add_container(cid, user_id, docker_id, port, password, name)

        ssh_command = f"ssh dev@{TAILSCALE_IP} -p {port}" if port else "ssh access unavailable"
        web_urls = {}

        return ContainerInfo(
            id=cid,
            port=port or 0,
            password=password,
            image_tag=name,
            ssh_command=ssh_command,
            web_urls=web_urls,
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/container", response_model=List[ContainerInfo])
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

@app.delete("/api/container/{cid}")
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

