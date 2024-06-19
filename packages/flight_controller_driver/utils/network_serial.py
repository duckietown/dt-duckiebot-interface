import socket
import serial

# Configuration
HOST = '0.0.0.0'  # Listen on all network interfaces
PORT = 12345  # Port to listen on for incoming connections
SERIAL_PORT = '/dev/ttyACM{}'  # Serial port to which data will be sent
BAUD_RATE = 1e6  # Baud rate for the serial communication
BAUD_RATE = 115200  # Baud rate for the serial communication

# Function to handle incoming connections
def handle_connection(client_socket, serial_port):
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        try:
            serial_port.write(data)
            response = serial_port.read_all()  # Read response from serial port
            client_socket.send(response)  # Send response back to client
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            client_socket.close()
            serial_port.close()
            return

# Main function
def main():
    while True:
        # Open serial port
        for i in range(10):
            try:
                serial_port = serial.Serial(SERIAL_PORT.format(i), BAUD_RATE)
                print(f"Serial port {SERIAL_PORT.format(i)} opened successfully.")
                break
            except serial.SerialException as e:
                print(f"Failed to open serial port: {e}")

        # Set up server socket
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((HOST, PORT))
            server_socket.listen(5)
            print(f"Server listening on {HOST}:{PORT}")
        except socket.error as e:
            print(f"Failed to create server socket: {e}")
            return

        try:
            while True:
                # Accept incoming connection
                client_socket, address = server_socket.accept()
                print(f"Accepted connection from {address}")

                # Handle incoming data from client and pipe it to serial port
                handle_connection(client_socket, serial_port)

                # Close client socket
                client_socket.close()

        except KeyboardInterrupt:
            print("Shutting down...")
            # Close sockets
            server_socket.close()
            serial_port.close()

if __name__ == "__main__":
    main()
