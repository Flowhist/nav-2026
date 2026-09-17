#!/usr/bin/env python3
import argparse
import signal
import time

from server_app import ServerApp


def main() -> None:
    parser = argparse.ArgumentParser(description="Finav Web Debug Server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8010)
    args = parser.parse_args()

    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    app = ServerApp(host=args.host, port=args.port)
    try:
        app.start()
        print(f"[server] running at http://{args.host}:{args.port}")
        while not stopping:
            time.sleep(0.2)
    finally:
        app.stop()
        print("\n[server] stopped")


if __name__ == "__main__":
    main()
