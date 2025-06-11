import argparse
import uvicorn

from .api import app
from .cli import cli


def main() -> None:
    parser = argparse.ArgumentParser(description="SSH Docker server")
    parser.add_argument("--cli", action="store_true", help="run CLI interface")
    parser.add_argument("--host", default="0.0.0.0", help="host for API server")
    parser.add_argument("--port", type=int, default=8000, help="port for API")
    args = parser.parse_args()

    if args.cli:
        cli()
    else:
        uvicorn.run("ssh_server.api:app", host=args.host, port=args.port)


if __name__ == "__main__":
    main()