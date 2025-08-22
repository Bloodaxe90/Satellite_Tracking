from src.fsm.fsm import FSM


def setup_fsm(port: str, baudrate: int, timeout: int = 1) -> FSM:
    """
    Initialize and configure the FSM (Fast Steering Mirror).

    Args:
        port (str): Serial port to which the FSM is connected (e.g. COM3 or /dev/ttyUSB0)
        baudrate (int): Baudrate for the serial communication
        timeout (int): Timeout value in seconds for the serial connection

    Returns:
        FSM: Configured and initialized FSM ready for commands
    """

    print("Setting up FSM/n")
    fsm = FSM(port, baudrate, timeout)
    fsm.connect()

    print("Status: (Should be 0000000000 signalling no issues)")
    fsm.send_command("status")
    fsm.send_command(
        "acknowledge"
    )  # Acknowledges and dismisses and non 0 status issues
    fsm.send_command("start")

    # Reset FSM
    fsm.send_command("xy=0;0")

    return fsm
