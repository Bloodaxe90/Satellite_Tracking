import queue
import threading
import time
import serial


class FSM:
    """
    Controls the FSM using serial communication with commands being sent using a background thread
    """

    def __init__(self, port: str, baudrate: int, timeout: int):
        """
        Initializes the controller with serial port settings

        Args:
            port (str): The serial port to connect to
            baudrate (int): The baudrate
            timeout (int): Timeout for serial read operations (in seconds)
        """
        self.port: str = port
        self.baudrate: int = baudrate
        self.timeout: float = timeout
        self.serial: serial.Serial = None
        self.write_thread: threading.Thread = threading.Thread(
            target=self.write_loop, daemon=True
        )
        self.command_buffer: queue.Queue = (
            queue.Queue()
        )  # Thread safe queue to store outgoing commands
        self.writing: bool = False  # Flag to control the write thread

    def connect(self):
        """
        Connects to the FSM device via the specified serial port
        and starts the background thread
        """
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.writing = True
            self.write_thread.start()
            print(f"Connected to FSM on {self.port} & write thread started \n")
        except serial.SerialException as e:
            print(f"Error: {e}, Failed to connect to port {self.port} \n")

    def disconnect(self):
        """
        Stops the write thread and closes the serial connection
        """
        if self.serial and self.serial.is_open:
            self.writing = False
            self.write_thread.join()
            self.serial.close()
            print("Disconnected from FSM\n")

    def read_response(self) -> list:
        """
        Reads all available lines from the FSMs serial response buffer

        Returns:
            list: A list of response lines from the FSM
        """
        response_lines = [""]
        while True:
            if self.serial.in_waiting > 0:
                line = self.serial.readline()
                if line:
                    response_lines.append(line.decode("utf-8").strip())
            else:
                time.sleep(0.1)

            if self.serial.in_waiting == 0:
                break

        return response_lines

    def write_loop(self):
        """
        Background thread loop that sends commands from the buffer
        to the FSM device
        """
        while self.writing:
            try:
                # Wait for a command to appear in the queue
                command = self.command_buffer.get(block=True, timeout=0.1)

                self.serial.reset_input_buffer()

                command_bytes: bytes = command.encode("utf-8") + b"\r\n"  # Add CR+LF
                self.serial.write(command_bytes)
            except queue.Empty:
                continue

    def send_command(
        self, command: str, print_sent: bool = True, print_received: bool = True
    ):
        """
        Sends a command to the FSM device and optionally waits for a response

        Args:
            command (str): The command to send (Refer to manual for valid FSM instructions)
            print_sent (bool): Whether to print the sent command (Added latency should be false during main fsm tracking loop)
            print_received (bool): Whether to read and print the response after sending (Added latency should be false during main fsm tracking loop)
        """

        if not self.serial or not self.serial.is_open:
            print("Not connected to FSM")
            return

        if print_sent:
            print(f"Sent: {command}")

        self.serial.reset_input_buffer()  # Clear old data before sending new command ensuring input buffer doesn't overflow when receive is False for extended periods
        self.command_buffer.put(command)

        if print_received:
            response: str = "\n".join(self.read_response())
            print(f"Received: {response}")
