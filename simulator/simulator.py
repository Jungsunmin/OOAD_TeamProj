import pygame
import sys
import socket
import json
import threading
import math
import random

# Configuration
MAP_SIZE = 10.0 # 10x10 meters
GRID_DIVISIONS = 10
CELL_SIZE = 60
WINDOW_SIZE = GRID_DIVISIONS * CELL_SIZE
FPS = 30
DUST_DENSITY = 0.4 
MAX_SENSOR_VALUE = 127

# RVC Spec based on src/rvc_controller.h
OBSTACLE_THRESHOLD = 10 # Value <= 10 is BLOCKED
DUST_THRESHOLD = 60     # Value >= 60 is DUSTY

# Physics
RVC_RADIUS = 0.3
LINEAR_SPEED = 1.0 
ANGULAR_SPEED = math.radians(90)

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
DUST_COLOR = (240, 230, 140)

class RVCSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption("RVC Simulator (Boundary Interface Ready)")
        self.clock = pygame.time.Clock()
        
        self.dust_map = [[0 for _ in range(GRID_DIVISIONS)] for _ in range(GRID_DIVISIONS)]
        for y in range(GRID_DIVISIONS):
            for x in range(GRID_DIVISIONS):
                if random.random() < DUST_DENSITY:
                    self.dust_map[y][x] = random.randint(1, MAX_SENSOR_VALUE)
        
        self.pos_x, self.pos_y = 1.0, 1.0
        self.angle = 0.0
        self.moving = False
        self.turning = 0
        self.cleaning = False
        
        self.is_emergency = False # For visual feedback
        self.running = True
        self.lock = threading.Lock()

    def update_physics(self, dt):
        with self.lock:
            if self.turning != 0:
                self.angle += self.turning * ANGULAR_SPEED * dt
            
            if self.moving:
                new_x = self.pos_x + math.cos(self.angle) * LINEAR_SPEED * dt
                new_y = self.pos_y + math.sin(self.angle) * LINEAR_SPEED * dt
                if RVC_RADIUS <= new_x <= MAP_SIZE - RVC_RADIUS:
                    self.pos_x = new_x
                if RVC_RADIUS <= new_y <= MAP_SIZE - RVC_RADIUS:
                    self.pos_y = new_y
            
            if self.cleaning:
                gx, gy = int(self.pos_x), int(self.pos_y)
                if 0 <= gx < GRID_DIVISIONS and 0 <= gy < GRID_DIVISIONS:
                    self.dust_map[gy][gx] = max(0, self.dust_map[gy][gx] - 50 * dt)

    def get_sensor_data(self):
        with self.lock:
            def check_dist(ray_angle):
                dx, dy = math.cos(ray_angle), math.sin(ray_angle)
                dists = []
                if dx > 0: dists.append((MAP_SIZE - self.pos_x) / dx)
                elif dx < 0: dists.append((0 - self.pos_x) / dx)
                if dy > 0: dists.append((MAP_SIZE - self.pos_y) / dy)
                elif dy < 0: dists.append((0 - self.pos_y) / dy)
                return min([d for d in dists if d >= 0])

            raw_front = check_dist(self.angle)
            raw_left = check_dist(self.angle - math.radians(45))
            raw_right = check_dist(self.angle + math.radians(45))
            
            # --- MAPPING for src logic ---
            # src expects: Value <= 10 means Blocked.
            # So: Value = distance_in_cm. 
            # 0.1m (10cm) -> 10. 
            def map_dist_to_sensor(dist):
                val = int(dist * 100) # 1m = 100 units
                return min(max(val, 0), MAX_SENSOR_VALUE)

            front = map_dist_to_sensor(raw_front)
            left = map_dist_to_sensor(raw_left)
            right = map_dist_to_sensor(raw_right)
            
            # Emergency status check (Visual only)
            self.is_emergency = (front <= OBSTACLE_THRESHOLD)

            gx, gy = int(self.pos_x), int(self.pos_y)
            dust = self.dust_map[gy][gx] if (0 <= gx < 10 and 0 <= gy < 10) else 0

            return {
                "front": front,
                "left": left,
                "right": right,
                "dust": int(dust),
                "pos": (self.pos_x, self.pos_y),
                "angle": math.degrees(self.angle)
            }

    def draw(self):
        self.screen.fill(WHITE)
        for y in range(GRID_DIVISIONS):
            for x in range(GRID_DIVISIONS):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                dust = self.dust_map[y][x]
                alpha = dust / float(MAX_SENSOR_VALUE) if dust > 0 else 0
                color = (
                    int(255 * (1 - alpha) + DUST_COLOR[0] * alpha),
                    int(255 * (1 - alpha) + DUST_COLOR[1] * alpha),
                    int(255 * (1 - alpha) + DUST_COLOR[2] * alpha)
                )
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, (220, 220, 220), rect, 1)

        screen_x, screen_y = int(self.pos_x * CELL_SIZE), int(self.pos_y * CELL_SIZE)
        radius = int(RVC_RADIUS * CELL_SIZE)
        
        # Color changes if front is blocked (Visualizing Interrupt Condition)
        rvc_color = YELLOW if self.is_emergency else RED
        pygame.draw.circle(self.screen, rvc_color, (screen_x, screen_y), radius)
        
        end_x = screen_x + int(math.cos(self.angle) * radius * 1.5)
        end_y = screen_y + int(math.sin(self.angle) * radius * 1.5)
        pygame.draw.line(self.screen, GREEN, (screen_x, screen_y), (end_x, end_y), 3)
        pygame.display.flip()

    def run_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('localhost', 12345))
        server_socket.listen(1)
        server_socket.settimeout(0.1)
        while self.running:
            try:
                conn, addr = server_socket.accept()
                with conn:
                    while self.running:
                        data = conn.recv(1024)
                        if not data: break
                        req = json.loads(data.decode())
                        if req['type'] == 'GET_SENSORS':
                            conn.sendall(json.dumps(self.get_sensor_data()).encode())
                        elif req['type'] == 'SET_CONTROL':
                            with self.lock:
                                self.moving = req.get('move', False)
                                self.turning = req.get('turn', 0)
                                self.cleaning = req.get('clean', False)
                            conn.sendall(json.dumps({"status": "ok"}).encode())
            except socket.timeout: continue

    def main_loop(self):
        server_thread = threading.Thread(target=self.run_server)
        server_thread.start()
        last_time = pygame.time.get_ticks()
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.running = False
            dt = (pygame.time.get_ticks() - last_time) / 1000.0
            last_time = pygame.time.get_ticks()
            self.update_physics(dt)
            self.draw()
            self.clock.tick(FPS)
        pygame.quit()
        server_thread.join()

if __name__ == "__main__":
    RVCSimulator().main_loop()
