import sys
import socket
import json
import threading
import math
import random
import time

# Minimal Mock of RVCSimulator for Headless testing
class MockSimulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.clients = []
        self.pos_x, self.pos_y = 1.5, 1.5
        self.angle = 0.0
        self.moving, self.turning, self.cleaning = False, 0, False
        self.cleaner_mode = "OFF"
        self.dust_map = [[0 for _ in range(10)] for _ in range(10)]

    def get_sensor_data_internal(self):
        return {
            "front": 127, "left": 127, "right": 127, "dust": 0,
            "pos": (self.pos_x, self.pos_y)
        }

    def handle_client(self, conn, addr):
        print(f"Connected by {addr}")
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
                    req = json.loads(line)
                    if req['type'] == 'GET_SENSORS':
                        conn.sendall((json.dumps(self.get_sensor_data_internal()) + "\n").encode())
                    elif req['type'] == 'SET_CONTROL':
                        print(f"Control: {line}")
                        conn.sendall(b"{\"status\":\"ok\"}\n")
                    elif req['type'] == 'TRIGGER_EVENT':
                        print(f"Triggering Event: {req['name']}")
                        if req['name'] == 'INTERRUPT':
                            self.broadcast_event({"type": "INTERRUPT"})
                        else:
                            self.broadcast_event({"type": "EVENT", "name": req['name']})
                        conn.sendall(b"{\"status\":\"ok\"}\n")
        except Exception as e: print(f"Error: {e}")
        finally:
            with self.lock: 
                if conn in self.clients: self.clients.remove(conn)
            conn.close()

    def broadcast_event(self, event_data):
        msg = (json.dumps(event_data) + "\n").encode()
        with self.lock:
            for conn in self.clients:
                try: conn.sendall(msg)
                except: pass

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('localhost', 12345))
        server.listen(10)
        print("Mock Server listening on 12345")
        while self.running:
            conn, addr = server.accept()
            threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    sim = MockSimulator()
    sim.run_server()
