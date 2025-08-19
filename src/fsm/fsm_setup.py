from src.fsm.fsm import FSM


def setup_fsm(port: str, baudrate: int, timeout: int) -> FSM:
    fsm = FSM(port, baudrate, timeout)
    fsm.connect()
    print(f"Status: (Should be 0000000000 signalling no issues)")
    fsm.send_command("status")
    fsm.send_command("acknowledge")
    fsm.send_command("start")
    fsm.send_command("xy=0;0")
    return fsm