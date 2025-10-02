#!/usr/bin/env python3
# RECEIVER_v2_ignore_first5s.py
from DualArmController_trigger_v2 import DualArmController
import socket
import time

HOST = "0.0.0.0"
PORT = 5050
INTERVAL = 0.5 

# --- Controller initialize ---
controller = DualArmController()
controller.set_sync_mode('allow_single')  # single hand is allowed
res = controller.start()
print(res["message"])  

label_to_gripper = {
    "TypeA": "open",
    "TypeB": "close",
    "TypeC": "half",
    "None": None,
}

def to_number(s: str):
    s = s.strip()
    if s and s.lstrip("+-").isdigit():
        try:
            return int(s)   
        except ValueError:
            pass
    try:
        return float(s)     
    except ValueError:
        return s            

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"listening on {HOST}:{PORT} ...")

    conn, addr = s.accept()
    print("connected from", addr)
    conn.setblocking(False)

    with conn:
        buf = b""
        last_text = None
        next_tick = time.perf_counter() + INTERVAL

        
        start_time = time.perf_counter()
        IGNORE_SECONDS = 1.0

        while True:
            # non blocking
            while True:
                try:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("client closed")
                        raise SystemExit
                    buf += chunk
                except BlockingIOError:
                    break  

            
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.rstrip(b"\r")
                if not line:
                    continue
                last_text = line.decode("utf-8", errors="replace").strip()

            
            now = time.perf_counter()
            if now >= next_tick:
                if last_text is not None:
                    
                    elapsed = now - start_time
                    if elapsed < IGNORE_SECONDS:
                        print(f"⏭️ 仍在啟動期（{elapsed:.2f}s < {IGNORE_SECONDS:.0f}s），忽略這筆：{last_text}")
                        last_text = None
                    else:
                        parts = [to_number(p) for p in last_text.split(",") if p.strip() != ""]
                        if len(parts) < 14:
                            print("⚠️ 資料長度不足:", parts)

                        else:
                            ts    = parts[0]
                            r1    = parts[1:7]# robot1 J1..J6
                            r1label = parts[7]
                            r2    = parts[8:14]   # robot2 J1..J6
                            r2label = parts[14]
                            g1 = label_to_gripper.get(r1label, None)
                            g2 = label_to_gripper.get(r2label, None)

                            
                            if (getattr(controller, "executor_thread", None) is None or
                                not controller.executor_thread.is_alive()):
                                controller.start()

                            
                            try:
                                controller.append_joint("robot1", r1, gripper=g1)
                                controller.append_joint("robot2", r2, gripper=g2)
                                print(f"[1Hz] ts={ts} | r1={r1} |right_label={r1label}| gripper={g1}| r2={r2} | left_label={r2label}| gripper={g2} ")
                            except Exception as e:
                                print(f"❌ append_joint error: {e}")

                        last_text = None

                
                while next_tick <= now:
                    next_tick += INTERVAL

            time.sleep(0.01)  