from src.services.http_service import app
import uvicorn
import json
import os


def main():
    cfg_path = "config/config.json"
    host = "0.0.0.0"
    port = 8025
    if os.path.exists(cfg_path):
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
            host = cfg.get("server_host", host)
            port = int(cfg.get("server_port", port))
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    main()



