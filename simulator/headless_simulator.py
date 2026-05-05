import sys
import socket
import json
import threading
import math
import random
import time

# --- Configuration ---
MAP_SIZE = 10.0
GRID_DIVISIONS = 10
FPS = 30

# --- Hardware Specs ---
MAX_VAL = 127
RVC_RADIUS = 0.3
LINEAR_SPEED = 1.0
ANGULAR_SPEED = math.radians(90)
INTERRUPT_THRESHOLD = 10 

class HeadlessSimulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.clients = [] 
        self.prev_front_blocked = False
        self.reset_environment()

    def reset_environment(self):
        with self.lock:
            self.dust_map = [[random.randint(0, MAX_VAL) if random.random() < 0.4 else 0 for _ in range(GRID_DIVISIONS)] for _ in range(GRID_DIVISIONS)]
            self.pos_x, self.pos_y = 1.5, 1.5
            self.angle = 0.0
            self.moving, self.turning, self.cleaning = False, 0, False
            self.cleaner_mode = "OFF"
            self.prev_front_blocked = False
            print("Headless Simulator: Environment Reset.")

    def broadcast_event(self, event_data):
        msg = (json.dumps(event_data) + "\n").encode()
        with self.lock:
            for conn in self.clients[:]:
                try:
                    conn.sendall(msg)
                except:
                    if conn in self.clients: self.clients.remove(conn)

    def update_physics(self, dt):
        with self.lock:
            if self.turning != 0: self.angle += self.turning * ANGULAR_SPEED * dt
            if self.moving:
                nx = self.pos_x + math.cos(self.angle) * LINEAR_SPEED * dt
                ny = self.pos_y + math.sin(self.angle) * LINEAR_SPEED * dt
                if RVC_RADIUS <= nx <= MAP_SIZE - RVC_RADIUS: self.pos_x = nx
                if RVC_RADIUS <= ny <= MAP_SIZE - RVC_RADIUS: self.pos_y = ny
            
            sensors = self.get_sensor_data_internal()
            is_blocked = (sensors['front'] <= INTERRUPT_THRESHOLD)
            if is_blocked and not self.prev_front_blocked:
                self.broadcast_event({"type": "INTERRUPT", "source": "FRONT_SENSOR"})
            self.prev_front_blocked = is_blocked

            if self.cleaning:
                gx, gy = int(self.pos_x), int(self.pos_y)
                if 0 <= gx < GRID_DIVISIONS and 0 <= gy < GRID_DIVISIONS:
                    pwr = 150 if self.cleaner_mode == "UP" else 50
                    self.dust_map[gy][gx] = max(0, self.dust_map[gy][gx] - pwr * dt)

    def get_sensor_data_internal(self):
        def raycast_surface(ray_angle):
            dx, dy = math.cos(ray_angle), math.sin(ray_angle)
            dists = []
            if dx > 0: dists.append((MAP_SIZE - self.pos_x) / dx)
            elif dx < 0: dists.append((0 - self.pos_x) / dx)
            if dy > 0: dists.append((MAP_SIZE - self.pos_y) / dy)
            elif dy < 0: dists.append((0 - self.pos_y) / dy)
            d_surface = max(0, min([d for d in dists if d >= 0]) - RVC_RADIUS)
            return min(int(d_surface * 127), MAX_VAL)

        gx, gy = int(self.pos_x), int(self.pos_y)
        dust_val = self.dust_map[gy][gx] if (0 <= gx < 10 and 0 <= gy < 10) else 0
        return {
            "front": raycast_surface(self.angle),
            "left": raycast_surface(self.angle - math.radians(45)),
            "right": raycast_surface(self.angle + math.radians(45)),
            "dust": int(dust_val),
            "pos": (round(self.pos_x, 2), round(self.pos_y, 2))
        }

    def handle_client(self, conn, addr):
        conn.settimeout(None)
        with self.lock: self.clients.append(conn)
        buffer = ""
        try:
            while self.running:
                data = conn.recv(4096)
                if not data: break
                buffer += data.decode()
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip(): continue
                    try:
                        req = json.loads(line)
                        if req['type'] == 'GET_SENSORS':
                            conn.sendall((json.dumps(self.get_sensor_data_internal()) + "\n").encode())
                        elif req['type'] == 'SET_CONTROL':
                            with self.lock:
                                self.moving = req.get('move', self.moving)
                                self.turning = req.get('turn', self.turning)
                                self.cleaning = req.get('clean', self.cleaning)
                                self.cleaner_mode = req.get('mode', self.cleaner_mode)
                            conn.sendall(b"{\"status\":\"ok\"}\n")
                        elif req['type'] == 'TRIGGER_EVENT':
                            print(f"Triggering Event: {req['name']}")
                            if req['name'] == 'INTERRUPT':
                                self.broadcast_event({"type": "INTERRUPT"})
                            else:
                                self.broadcast_event({"type": "EVENT", "name": req['name']})
                            conn.sendall(b"{\"status\":\"ok\"}\n")
                    except Exception as e:
                        print(f"Simulator: Request Error {e}")
        except: pass
        finally:
            with self.lock: 
                if conn in self.clients: self.clients.remove(conn)
            conn.close()

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('localhost', 12345)); server.listen(10)
        print("Headless Server listening on 12345")
        while self.running:
            try:
                conn, addr = server.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
            except: break

    def main_loop(self):
        while self.running:
            self.update_physics(1.0/FPS)
            time.sleep(1.0/FPS)

if __name__ == "__main__":
    sim = HeadlessSimulator()
    threading.Thread(target=sim.run_server, daemon=True).start()
    sim.main_loop()
