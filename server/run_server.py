#!/usr/bin/env python3
import argparse
import time

from server_app import ServerApp


def main() -> None:
    parser = argparse.ArgumentParser(description="Finav Web Debug Server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8010)
    args = parser.parse_args()

    app = ServerApp(host=args.host, port=args.port)
    app.start()

    print(f"[server] running at http://{args.host}:{args.port}")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        app.stop()
        print("\n[server] stopped")


if __name__ == "__main__":
    main()
