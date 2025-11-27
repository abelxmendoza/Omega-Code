# File: servers/control_api.py
import os, time, pathlib
from flask import Blueprint, Response, request
from typing import Callable

def create_control_blueprint(
    camera_present: Callable[[], bool],
    get_frame: Callable[[], "np.ndarray | None"],
    cv2
) -> Blueprint:
    bp = Blueprint("control", __name__)

    CAPTURE_DIR = pathlib.Path(os.getenv("CAPTURE_DIR", "/home/omega1/Omega-Code/captures/backend"))
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)

    @bp.get("/snapshot")
    def snapshot():
        if not camera_present():
            return "❌ Camera not available", 400
        frame = get_frame()
        if frame is None:
            return "❌ No frame available", 503

        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            return "❌ JPEG encode failed", 500
        jpg = buf.tobytes()

        if request.args.get("save", "0").lower() in ("1","true","yes"):
            ts = int(time.time())
            (CAPTURE_DIR / f"snapshot_{ts}.jpg").write_bytes(jpg)

        return Response(
            jpg, mimetype="image/jpeg",
            headers={"Cache-Control":"no-store, no-cache, must-revalidate, max-age=0"}
        )

    return bp
