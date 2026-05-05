import pygame
import sys
import socket
import json
import threading
import math
import random

# --- Configuration ---
MAP_SIZE = 10.0      # 10.0m x 10.0m
GRID_DIVISIONS = 10  # 10x10 blocks
CELL_SIZE = 60       # 60px per block
WINDOW_SIZE = GRID_DIVISIONS * CELL_SIZE
FPS = 30

# --- Hardware Specs ---
MAX_VAL = 127        # 7-bit resolution (0~127)
RVC_RADIUS = 0.3     # 30cm radius
LINEAR_SPEED = 1.0   # 1.0 m/s
ANGULAR_SPEED = math.radians(90) # 90 deg/s

# --- Colors ---
WHITE, BLACK, RED, GREEN, YELLOW = (255,255,255), (0,0,0), (255,0,0), (0,255,0), (255,255,0)
BLUE, GRAY, ORANGE, DUST_COLOR = (0,0,255), (200,200,200), (255,165,0), (240,230,140)

class RVCSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE + 200, WINDOW_SIZE))
        pygame.display.set_caption("RVC Final Simulator (Calibrated)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)
        
        self.lock = threading.Lock()
        self.running = True
        
        # UI Elements
        self.btn_on_rect = pygame.Rect(WINDOW_SIZE + 20, 50, 160, 40)
        self.btn_off_rect = pygame.Rect(WINDOW_SIZE + 20, 100, 160, 40)
        self.btn_reset_rect = pygame.Rect(WINDOW_SIZE + 20, 150, 160, 40)
        
        self.reset_environment()

    def reset_environment(self):
        with self.lock:
            # Random Dust (40% Density, 0-127 Level)
            self.dust_map = [[random.randint(0, MAX_VAL) if random.random() < 0.4 else 0 for _ in range(GRID_DIVISIONS)] for _ in range(GRID_DIVISIONS)]
            # Robot Initial State (Inside a square)
            self.pos_x, self.pos_y = 1.5, 1.5
            self.angle = 0.0
            self.moving, self.turning, self.cleaning = False, 0, False
            self.cleaner_mode = "OFF"
            self.event_queue = []
            print("Simulator: Environment Reset.")

    def update_physics(self, dt):
        with self.lock:
            if self.turning != 0: self.angle += self.turning * ANGULAR_SPEED * dt
            if self.moving:
                nx = self.pos_x + math.cos(self.angle) * LINEAR_SPEED * dt
                ny = self.pos_y + math.sin(self.angle) * LINEAR_SPEED * dt
                if RVC_RADIUS <= nx <= MAP_SIZE - RVC_RADIUS: self.pos_x = nx
                if RVC_RADIUS <= ny <= MAP_SIZE - RVC_RADIUS: self.pos_y = ny
            if self.cleaning:
                gx, gy = int(self.pos_x), int(self.pos_y)
                if 0 <= gx < GRID_DIVISIONS and 0 <= gy < GRID_DIVISIONS:
                    # Scaling cleaning power
                    pwr = 150 if self.cleaner_mode == "UP" else 50
                    self.dust_map[gy][gx] = max(0, self.dust_map[gy][gx] - pwr * dt)

    def get_sensor_data(self):
        with self.lock:
            def raycast_surface(ray_angle):
                dx, dy = math.cos(ray_angle), math.sin(ray_angle)
                dists = []
                if dx > 0: dists.append((MAP_SIZE - self.pos_x) / dx)
                elif dx < 0: dists.append((0 - self.pos_x) / dx)
                if dy > 0: dists.append((MAP_SIZE - self.pos_y) / dy)
                elif dy < 0: dists.append((0 - self.pos_y) / dy)
                # Distance from robot surface = (Distance from center) - RADIUS
                d_surface = max(0, min([d for d in dists if d >= 0]) - RVC_RADIUS)
                # SCALE: 1.0m = 127. Limit to 0~127.
                return min(int(d_surface * 127), MAX_VAL)

            # Center coordinates for dust
            gx, gy = int(self.pos_x), int(self.pos_y)
            dust_val = self.dust_map[gy][gx] if (0 <= gx < 10 and 0 <= gy < 10) else 0

            return {
                "front": raycast_surface(self.angle),
                "left": raycast_surface(self.angle - math.radians(45)),
                "right": raycast_surface(self.angle + math.radians(45)),
                "dust": int(dust_val),
                "pos": (round(self.pos_x, 2), round(self.pos_y, 2)),
                "angle": int(math.degrees(self.angle)) % 360
            }

    def draw(self):
        self.screen.fill(WHITE)
        # Grid & Dust
        for y in range(GRID_DIVISIONS):
            for x in range(GRID_DIVISIONS):
                rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                d = self.dust_map[y][x]
                alpha = d / float(MAX_VAL)
                color = (int(255*(1-alpha)+DUST_COLOR[0]*alpha), int(255*(1-alpha)+DUST_COLOR[1]*alpha), int(255*(1-alpha)+DUST_COLOR[2]*alpha))
                pygame.draw.rect(self.screen, color, rect); pygame.draw.rect(self.screen, (220, 220, 220), rect, 1)

        # RVC Visualization
        sx, sy = int(self.pos_x*CELL_SIZE), int(self.pos_y*CELL_SIZE)
        rad = int(RVC_RADIUS*CELL_SIZE)
        sensors = self.get_sensor_data()
        # Visual emergency alert
        body_color = YELLOW if sensors['front'] <= 10 else RED
        pygame.draw.circle(self.screen, body_color, (sx, sy), rad)
        pygame.draw.line(self.screen, GREEN, (sx, sy), (sx + int(math.cos(self.angle)*rad*1.5), sy + int(math.sin(self.angle)*rad*1.5)), 3)

        # UI Panel
        pygame.draw.rect(self.screen, GRAY, (WINDOW_SIZE, 0, 200, WINDOW_SIZE))
        pygame.draw.rect(self.screen, GREEN, self.btn_on_rect); self.screen.blit(self.font.render("POWER ON", True, WHITE), (WINDOW_SIZE+60, 62))
        pygame.draw.rect(self.screen, RED, self.btn_off_rect); self.screen.blit(self.font.render("POWER OFF", True, WHITE), (WINDOW_SIZE+58, 112))
        pygame.draw.rect(self.screen, ORANGE, self.btn_reset_rect); self.screen.blit(self.font.render("RESET MAP", True, WHITE), (WINDOW_SIZE+58, 162))

        # Real-time Status
        sy_off = 230
        self.screen.blit(self.font.render(f"Cleaner: {self.cleaner_mode}", True, BLACK), (WINDOW_SIZE+20, sy_off))
        self.screen.blit(self.font.render(f"Front Sensor: {sensors['front']}", True, BLACK if sensors['front']>10 else RED), (WINDOW_SIZE+20, sy_off+25))
        self.screen.blit(self.font.render(f"Dust Sensor: {sensors['dust']}", True, BLACK), (WINDOW_SIZE+20, sy_off+50))
        self.screen.blit(self.font.render(f"Pos: {sensors['pos']}", True, BLACK), (WINDOW_SIZE+20, sy_off+75))
        
        pygame.display.flip()

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('localhost', 12345)); server.listen(1); server.settimeout(0.1)
        while self.running:
            try:
                conn, addr = server.accept()
                with conn:
                    while self.running:
                        data = conn.recv(1024)
                        if not data: break
                        req = json.loads(data.decode())
                        if req['type'] == 'GET_SENSORS':
                            resp = self.get_sensor_data()
                            with self.lock:
                                if self.event_queue: resp["event"] = self.event_queue.pop(0)
                            conn.sendall(json.dumps(resp).encode())
                        elif req['type'] == 'SET_CONTROL':
                            with self.lock:
                                self.moving, self.turning, self.cleaning = req.get('move', self.moving), req.get('turn', self.turning), req.get('clean', self.cleaning)
                                self.cleaner_mode = req.get('mode', self.cleaner_mode)
                            conn.sendall(json.dumps({"status": "ok"}).encode())
            except socket.timeout: continue

    def main_loop(self):
        threading.Thread(target=self.run_server, daemon=True).start()
        last = pygame.time.get_ticks()
        while self.running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT: self.running = False
                if ev.type == pygame.MOUSEBUTTONDOWN:
                    if self.btn_on_rect.collidepoint(ev.pos): self.event_queue.append("BUTTON_ON")
                    if self.btn_off_rect.collidepoint(ev.pos): self.event_queue.append("BUTTON_OFF")
                    if self.btn_reset_rect.collidepoint(ev.pos): self.reset_environment()
            dt = (pygame.time.get_ticks() - last) / 1000.0
            last = pygame.time.get_ticks()
            self.update_physics(dt); self.draw(); self.clock.tick(FPS)
        pygame.quit()

if __name__ == "__main__":
    RVCSimulator().main_loop()
