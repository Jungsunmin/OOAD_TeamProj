import socket
import json

def push_button():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('localhost', 12345))
    
    # We can't directly 'push' a button via socket if the simulator doesn't have a command for it.
    # But wait, our simulator sends events in the 'GET_SENSORS' response.
    # So we need to make the simulator think the button was pressed.
    # Actually, the simulator UI handles the button press and sets self.button_event.
    # I'll just tell the user to press it.
    
    print("Please press the 'POWER ON' button in the Pygame window.")

if __name__ == "__main__":
    push_button()
