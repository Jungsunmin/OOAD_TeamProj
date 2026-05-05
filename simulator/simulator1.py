import pygame
print("Specialized Simulator (simulator1.py) starting...")
import sys
import socket
import json
import threading
import math
import random

# --- Configuration ---
MAP_SIZE = 10.0
GRID_DIVISIONS = 10
CELL_SIZE = 60
WINDOW_SIZE = GRID_DIVISIONS * CELL_SIZE
FPS = 30

# --- Hardware Specs ---
MAX_VAL = 127
RVC_RADIUS = 0.3
LINEAR_SPEED = 1.0
ANGULAR_SPEED = math.radians(90)
INTERRUPT_THRESHOLD = 30
FRONT_SENSOR_DRAW_LENGTH = 1.67
SIDE_SENSOR_DRAW_LENGTH = 1.5

class RVCSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE + 200, WINDOW_SIZE))
        pygame.display.set_caption("RVC - 'ㅗ' Shape Test Map")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)
        
        self.lock = threading.Lock()
        self.running = True
        self.clients = [] 
        
        self.btn_on_rect = pygame.Rect(WINDOW_SIZE + 20, 50, 160, 40)
        self.btn_off_rect = pygame.Rect(WINDOW_SIZE + 20, 100, 160, 40)
        self.btn_reset_rect = pygame.Rect(WINDOW_SIZE + 20, 150, 160, 40)
        
        self.prev_front_blocked = False
        self.reset_environment()

    def reset_environment(self):
        with self.lock:
            self.dust_map = [[random.randint(0, MAX_VAL) if random.random() < 0.4 else 0 for _ in range(GRID_DIVISIONS)] for _ in range(GRID_DIVISIONS)]
            self.walls = [[0 for _ in range(GRID_DIVISIONS)] for _ in range(GRID_DIVISIONS)]
            
            # "ㅗ" shaped pocket at center
            self.walls[4][4] = 1 
            self.walls[4][5] = 1 
            self.walls[4][6] = 1 
            self.walls[5][4] = 1 
            self.walls[5][6] = 1 
            
            self.pos_x, self.pos_y = 5.5, 6.5
            self.angle = -math.pi/2 
            
            self.moving, self.turning, self.cleaning = 0, 0, False
            self.cleaner_mode = "OFF"
            self.prev_front_blocked = False
            
        self.broadcast_event({"type": "EVENT", "name": "RESET"})
        print("Simulator: Environment Reset (ㅗ Shape Loaded).")

    def broadcast_event(self, event_data):
        msg = (json.dumps(event_data) + "\n").encode()
        clients_copy = []
        with self.lock:
            clients_copy = self.clients[:]
        for conn in clients_copy:
            try:
                conn.sendall(msg)
            except:
                with self.lock:
                    if conn in self.clients: self.clients.remove(conn)

    def is_wall(self, x, y):
        gx, gy = int(x), int(y)
        if 0 <= gx < GRID_DIVISIONS and 0 <= gy < GRID_DIVISIONS:
            return self.walls[gy][gx] == 1
        return True 

    def update_physics(self, dt):
        trigger_interrupt = False
        with self.lock:
            if self.turning != 0: self.angle += self.turning * ANGULAR_SPEED * dt
            if self.moving != 0:
                nx = self.pos_x + math.cos(self.angle) * LINEAR_SPEED * dt * self.moving
                ny = self.pos_y + math.sin(self.angle) * LINEAR_SPEED * dt * self.moving
                can_move = True
                for ox, oy in [(-RVC_RADIUS, -RVC_RADIUS), (RVC_RADIUS, -RVC_RADIUS), (-RVC_RADIUS, RVC_RADIUS), (RVC_RADIUS, RVC_RADIUS)]:
                    if self.is_wall(nx + ox, ny + oy):
                        can_move = False
                        break
                if can_move:
                    self.pos_x, self.pos_y = nx, ny
            
            sensors = self.get_sensor_data_internal()
            is_blocked = (sensors['front'] <= INTERRUPT_THRESHOLD)
            if is_blocked and not self.prev_front_blocked:
                trigger_interrupt = True
            self.prev_front_blocked = is_blocked

            if self.cleaning:
                gx, gy = int(self.pos_x), int(self.pos_y)
                if 0 <= gx < GRID_DIVISIONS and 0 <= gy < GRID_DIVISIONS:
                    pwr = 150 if self.cleaner_mode == "UP" else 50
                    self.dust_map[gy][gx] = max(0, self.dust_map[gy][gx] - pwr * dt)
        
        if trigger_interrupt:
            self.broadcast_event({"type": "INTERRUPT", "source": "FRONT_SENSOR"})

    def get_sensor_data_internal(self):
        def raycast_all(ray_angle, origin_x=None, origin_y=None):
            if origin_x is None:
                origin_x = self.pos_x
            if origin_y is None:
                origin_y = self.pos_y

            dx, dy = math.cos(ray_angle), math.sin(ray_angle)
            dists = []
            if dx > 0: dists.append((MAP_SIZE - origin_x) / dx)
            elif dx < 0: dists.append((0 - origin_x) / dx)
            if dy > 0: dists.append((MAP_SIZE - origin_y) / dy)
            elif dy < 0: dists.append((0 - origin_y) / dy)
            d_boundary = min([d for d in dists if d >= 0]) if dists else MAP_SIZE
            
            d_wall = MAP_SIZE
            step = 0.05
            for d in [i * step for i in range(1, int(d_boundary/step))]:
                if self.is_wall(origin_x + dx * d, origin_y + dy * d):
                    d_wall = d
                    break
            
            dist = min(d_boundary, d_wall)
            return min(int(max(0, dist - RVC_RADIUS) * 127 / 2.0), MAX_VAL)

        gx, gy = int(self.pos_x), int(self.pos_y)
        dust_val = self.dust_map[gy][gx] if (0 <= gx < 10 and 0 <= gy < 10) else 0

        side_sensor_forward_offset = 0.5
        side_origin_x = self.pos_x + math.cos(self.angle) * side_sensor_forward_offset
        side_origin_y = self.pos_y + math.sin(self.angle) * side_sensor_forward_offset

        return {
            "front": raycast_all(self.angle),
            "left": raycast_all(self.angle - math.radians(90), side_origin_x, side_origin_y),
            "right": raycast_all(self.angle + math.radians(90), side_origin_x, side_origin_y),
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
        while self.running:
            try:
                conn, addr = server.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
            except: break

    def draw(self):
        self.screen.fill((255,255,255))
        for y in range(GRID_DIVISIONS):
            for x in range(GRID_DIVISIONS):
                rect = pygame.Rect(x*60, y*60, 60, 60)
                if self.walls[y][x] == 1:
                    pygame.draw.rect(self.screen, (50, 50, 50), rect)
                else:
                    d = self.dust_map[y][x]
                    alpha = d / float(MAX_VAL)
                    color = (int(255*(1-alpha)+240*alpha), int(255*(1-alpha)+230*alpha), int(255*(1-alpha)+140*alpha))
                    pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, (220, 220, 220), rect, 1)

        sx, sy = int(self.pos_x*60), int(self.pos_y*60)
        rad = int(RVC_RADIUS*60)
        sensors = self.get_sensor_data_internal()

        side_sensor_forward_offset = 0.5
        side_sx = int((self.pos_x + math.cos(self.angle) * side_sensor_forward_offset) * CELL_SIZE)
        side_sy = int((self.pos_y + math.sin(self.angle) * side_sensor_forward_offset) * CELL_SIZE)
        # Change color only if FRONT sensor is blocked
        pygame.draw.circle(self.screen, (255,255,0) if sensors['front'] <= INTERRUPT_THRESHOLD else (255,0,0), (sx, sy), rad)
        
        # Draw Sensors (All consistent length and width)
        # Front
        pygame.draw.line(self.screen, (0,255,0), (sx, sy), (sx + int(math.cos(self.angle)*rad*FRONT_SENSOR_DRAW_LENGTH), sy + int(math.sin(self.angle)*rad*FRONT_SENSOR_DRAW_LENGTH)), 3)
        # Left and right sensors start slightly forward in the front sensor direction.
        pygame.draw.line(self.screen, (0,255,0), (side_sx, side_sy), (side_sx + int(math.cos(self.angle-math.radians(90))*rad*SIDE_SENSOR_DRAW_LENGTH), side_sy + int(math.sin(self.angle-math.radians(90))*rad*SIDE_SENSOR_DRAW_LENGTH)), 3)
        pygame.draw.line(self.screen, (0,255,0), (side_sx, side_sy), (side_sx + int(math.cos(self.angle+math.radians(90))*rad*SIDE_SENSOR_DRAW_LENGTH), side_sy + int(math.sin(self.angle+math.radians(90))*rad*SIDE_SENSOR_DRAW_LENGTH)), 3)
        
        pygame.draw.rect(self.screen, (200,200,200), (600, 0, 200, 600))
        pygame.draw.rect(self.screen, (0,255,0), self.btn_on_rect); self.screen.blit(self.font.render("POWER ON", True, (255,255,255)), (660, 62))
        pygame.draw.rect(self.screen, (255,0,0), self.btn_off_rect); self.screen.blit(self.font.render("POWER OFF", True, (255,255,255)), (658, 112))
        pygame.draw.rect(self.screen, (255,165,0), self.btn_reset_rect); self.screen.blit(self.font.render("RESET MAP", True, (255,255,255)), (658, 162))
        
        self.screen.blit(self.font.render(f"Cleaner: {self.cleaner_mode}", True, (0,0,0)), (620, 230))
        self.screen.blit(self.font.render(f"Front Sensor: {sensors['front']}", True, (0,0,0)), (620, 260))
        self.screen.blit(self.font.render(f"Left Sensor: {sensors['left']}", True, (0,0,0)), (620, 290))
        self.screen.blit(self.font.render(f"Right Sensor: {sensors['right']}", True, (0,0,0)), (620, 320))
        self.screen.blit(self.font.render(f"Dust Sensor: {sensors['dust']}", True, (0,0,0)), (620, 350))
        pygame.display.flip()

    def main_loop(self):
        while self.running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT: self.running = False
                if ev.type == pygame.MOUSEBUTTONDOWN:
                    if self.btn_on_rect.collidepoint(ev.pos): self.broadcast_event({"type": "EVENT", "name": "BUTTON_ON"})
                    if self.btn_off_rect.collidepoint(ev.pos): self.broadcast_event({"type": "EVENT", "name": "BUTTON_OFF"})
                    if self.btn_reset_rect.collidepoint(ev.pos): self.reset_environment()
            self.update_physics(1.0/FPS); self.draw(); self.clock.tick(FPS)
        pygame.quit()

if __name__ == "__main__":
    print("Simulator: Starting Server Thread...")
    sim = RVCSimulator()
    threading.Thread(target=sim.run_server, daemon=True).start()
    print("Simulator: Server Thread Started. Entering Main Loop (GUI)...")
    sim.main_loop()
