import socket
import json
import time

def test_client():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('localhost', 12345))
    
    def set_control(move=False, turn=0, clean=False):
        req = {"type": "SET_CONTROL", "move": move, "turn": turn, "clean": clean}
        client.sendall((json.dumps(req) + "\n").encode())
        return json.loads(client.recv(1024).decode())

    def get_sensors():
        req = {"type": "GET_SENSORS"}
        client.sendall((json.dumps(req) + "\n").encode())
        return json.loads(client.recv(1024).decode())

    print("Initial State:", get_sensors())
    
    print("\nStarting Move & Turn Right & Clean for 2 seconds...")
    set_control(move=True, turn=1, clean=True)
    time.sleep(2)
    
    print("State after 2s:", get_sensors())
    
    print("\nStopping...")
    set_control(move=False, turn=0, clean=False)
    print("Final State:", get_sensors())

    client.close()

if __name__ == "__main__":
    test_client()
