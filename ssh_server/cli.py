from .database import Database
from .docker_manager import DockerManager, TAILSCALE_IP, TAILSCALE_FUNNEL_DOMAIN


def cli() -> None:
    """Interactive command-line interface for your Docker-SSH service."""
    db = Database()
    docker_manager = DockerManager()

    print("=== SSH Docker Server CLI ===")
    if (choice := input("1) Register\n2) Login\nSelect: ")) == "1":
        if db.add_user(input("Name: "), input("Email: "), input("Password: ")):
            print("Registered successfully")
        else:
            print("Email already registered")
        return
    elif choice != "2":
        return

    email, pwd = input("Email: "), input("Password: ")
    if not (user_id := db.verify_user(email, pwd)):
        print("Invalid credentials")
        return

    while True:
        print("\n1) Create instance"
              "\n2) List instances"
              "\n3) Delete instance"
              "\n4) List available images"
              "\n5) Create from compose"
              "\n6) Funnel instructions"
              "\n7) Exit")
        opt = input("Select: ")

        # -------------------- 1) Create instance --------------------------- #
        if opt == "1":
            try:
                # ----- let user pick an image --------------------------------
                imgs = docker_manager.list_available_images()
                if not imgs:
                    print("No Docker images found.")
                    continue

                for i, img in enumerate(imgs, 1):
                    print(f"{i}) {img['tag']} (ID {img['id']})")
                print(f"{len(imgs)+1}) Enter custom image name")

                while True:
                    try:
                        idx = int(input("Select image: ")) - 1
                        if idx == len(imgs):
                            image_tag = input("Image tag: ").strip()
                            break
                        if 0 <= idx < len(imgs):
                            image_tag = imgs[idx]["tag"]
                            break
                        print("Invalid selection")
                    except ValueError:
                        print("Please enter a number")

                # ----- create container --------------------------------------
                print(f"Creating container with {image_tag}…")
                cid, docker_id, port, passwd, used_img, info = \
                    docker_manager.create_container(user_id, image_tag)

                funnel_path = info["funnel_path"]
                funnel_port = info["funnel_port"]

                db.add_container(cid, user_id, docker_id, port, passwd,
                                 used_img, info["ssh_ready"], funnel_port)

                print(f"\n✅ Container {cid} created (image: {used_img})")
                print(f"SSH (Tailscale): ssh "
                      f"{'root' if 'hydrus' in used_img.lower() else 'dev'}@"
                      f"{TAILSCALE_IP} -p {port}")
                print(f"Password: {passwd}")

                if funnel_path:
                    print("\n🌐 Public HTTPS via Tailscale Funnel:")
                    print(f"   https://{TAILSCALE_FUNNEL_DOMAIN}{funnel_path}")
                else:
                    print("\n⚠️  Funnel auto-setup failed or disabled.")
                    print(f"   Run manually: "
                          f"sudo tailscale serve https /{cid} tcp://localhost:{port}")

            except Exception as exc:
                print(f"[Error] {exc}")

        # -------------------- 2) List instances ---------------------------- #
        elif opt == "2":
            containers = db.list_containers(user_id)
            if not containers:
                print("No containers found")
            else:
                print("\n📋 Your containers:")
                print(f"{'ID':<12} {'Port':<8} {'SSH':<5} {'Password':<12} {'Image':<25} {'Access'}")
                print("-" * 100)
                for r in containers:
                    container_id, docker_id, port, password, image_tag, ssh_ready, funnel_port = r
                    ssh_status = "✅" if ssh_ready else "⚠️"
                    if "hydrus" in image_tag.lower():
                        access_info = f"ssh root@{TAILSCALE_IP} -p {port}"
                    else:
                        access_info = f"ssh dev@{TAILSCALE_IP} -p {port}"
                    print(f"{container_id:<12} {port:<8} {ssh_status:<5} {password:<12} {image_tag:<25} {access_info}")

        # -------------------- 3) Delete instance --------------------------- #
        elif opt == "3":
            cid = input("Container id: ").strip()
            for r in db.list_containers(user_id):
                if r[0] == cid:
                    docker_manager.remove_container(r[1])
                    db.delete_container(user_id, cid)
                    print("Deleted.")
                    break
            else:
                print("Not found.")

        # -------------------- 4) List Docker images ------------------------ #
        elif opt == "4":
            imgs = docker_manager.list_available_images()
            if not imgs:
                print("No Docker images.")
            else:
                print(f"\n{'Image Tag':<45}{'ID':<15}{'Size (MB)'}")
                print("-" * 70)
                for img in imgs:
                    mb = img['size'] // (1024 * 1024)
                    print(f"{img['tag']:<45}{img['id']:<15}{mb}")

        # -------------------- 5) Create from compose ----------------------- #
        elif opt == "5":
            names = docker_manager.list_available_compose()
            if not names:
                print("No compose configurations available.")
                continue
            for i, name in enumerate(names, 1):
                print(f"{i}) {name}")
            try:
                idx = int(input("Select compose file: ")) - 1
                if not 0 <= idx < len(names):
                    print("Invalid selection")
                    continue
                chosen = names[idx]
            except ValueError:
                print("Invalid selection")
                continue

            print(f"Launching compose '{chosen}'…")
            try:
                cid, docker_id, port, passwd, name, info = docker_manager.create_container_from_compose(user_id, chosen)
                funnel_path = info["funnel_path"]
                funnel_port = info["funnel_port"]

                db.add_container(cid, user_id, docker_id, port, passwd, name, info["ssh_ready"], funnel_port)

                if port:
                    print(f"SSH: ssh dev@{TAILSCALE_IP} -p {port}")
                print(f"Password: {passwd}")
                if funnel_path:
                    print(f"Funnel: https://{TAILSCALE_FUNNEL_DOMAIN}{funnel_path}")
            except Exception as exc:
                print(f"[Error] {exc}")

        # -------------------- 6) Quick Funnel help ------------------------- #
        elif opt == "6":
            print("\n📡  Tailscale Funnel Quick-start")
            print("--------------------------------")
            print("Expose any TCP port publicly:")
            print("  sudo tailscale serve https /myapp tcp://localhost:<port>")
            print("Visit:")
            print(f"  https://{TAILSCALE_FUNNEL_DOMAIN}/myapp\n"
                  "SSH itself can’t be wrapped in HTTPS; use Tailscale IP+port "
                  "for SSH, and funnel web UIs instead.")

        # -------------------- 7) Exit -------------------------------------- #
        else:
            break
