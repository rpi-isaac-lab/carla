import csv
import socket
import logging
import json
import signal
import threading

class CSVHandler(logging.Handler):
    def __init__(self, filename, headers):
        super().__init__()
        self.filename = filename
        self.headers = headers

        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.headers)
        
        if self.file.tell() == 0:
            self.writer.writeheader()

    def emit(self, record):
        log_entry = self.format(record)
        self.writer.writerow(log_entry)
        self.file.flush() # flushes immediately in case of crash

    def close(self):
        self.file.close()
        super().close()

class CSVFormatter(logging.Formatter):
    def __init__(self, headers):
        super().__init__()
        self.headers = headers

    def format(self, record):
        log_data = {header: getattr(record, header, '') for header in self.headers}
        return log_data

# Initializes and sets up the CSV logger
# Returns the logger
def setup_logger(filename, headers):
    logger = logging.getLogger('csv_logger')
    logger.setLevel(logging.INFO)

    csv_handler = CSVHandler(filename, headers)
    csv_formatter = CSVFormatter(headers)

    csv_handler.setFormatter(csv_formatter)
    logger.addHandler(csv_handler)
    
    return logger

'''
Currently, the logger must take in these parameters as strings and
also creates the file in the directory where the file was run. It
also prints out a log message to console. 
'''

# Starts the logger server
# Listening happens in a separate daemon
# End server with KeyboardInterrupt
def start_logger_server(host='localhost', port=5000):
    # Initialize logger with headers and filename
    headers = headers = ['time elapsed', 'collision', 'lane breach', 'lap count'] #add headers for additional data; or leave blank if don't need header
    csv_filename = 'log.csv'

    logger = setup_logger(csv_filename, headers)

    # Listen for connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f'Logger server listening on {host}:{port}')
        
        # Listen for data
        def listen():
            while True:
                client_socket, addr = server_socket.accept()
                with client_socket:
                    print(f'Connected to {addr}')
                    # receive data
                    while True:
                        data = client_socket.recv(1024).decode('utf-8')
                        if not data:
                            break
                        data_dict = json.loads(data)
                        print(data_dict)
                        logger.info("log", extra=data_dict)

        # Create daemon and run
        listener_thread = threading.Thread(target=listen)
        listener_thread.daemon = True
        listener_thread.start()
        

        # Exit condition: end on KeyboardInterrupt
        try:
            while True:
                pass 
        except KeyboardInterrupt:
            print("Shutting down the server...")

if __name__ == '__main__':
    start_logger_server()

