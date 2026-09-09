import os
import socket
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.sim_arm import SimArmClient
from core.sim_gripper import SimGripperClient
from core.tcp_protocol import JsonStreamReader, send_json_compat


class FakeSimServer:
    def __init__(self, port, value=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.listen(1)
        self.received = []
        self.value = value
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        conn, _ = self.sock.accept()
        with conn:
            reader = JsonStreamReader(conn)
            while True:
                try:
                    req = reader.read()
                except ConnectionError:
                    break
                self.received.append(req)
                send_json_compat(
                    conn, {"value": self.value, "info": {}}, reader.mode
                )


def test_sim_arm_execute_trajectory_sends_degrees():
    server = FakeSimServer(18331)
    c = SimArmClient(host="127.0.0.1", port=18331)
    assert c.connect() is True
    c.execute_trajectory([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]], speed=20)
    req = server.received[-1]
    assert req["cmd"] == "execute_trajectory"
    assert req["trajectory"] == [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]]
    print("PASS: test_sim_arm_execute_trajectory_sends_degrees")


def test_sim_gripper_open_sends_value_1000():
    server = FakeSimServer(18332)
    c = SimGripperClient(host="127.0.0.1", port=18332,
                         src="/left_gripper/movement_control")
    c.connect()
    c.open()
    req = server.received[-1]
    assert req["cmd"] == "gripper"
    assert req["action"] == "open"
    assert req["value"] == 1000
    print("PASS: test_sim_gripper_open_sends_value_1000")


def test_sim_arm_propagates_service_failure():
    server = FakeSimServer(18333, value=False)
    c = SimArmClient(host="127.0.0.1", port=18333)
    assert c.connect() is True
    assert c.execute_trajectory([[1, 2, 3, 4, 5, 6, 7]]) is False


if __name__ == "__main__":
    test_sim_arm_execute_trajectory_sends_degrees()
    test_sim_gripper_open_sends_value_1000()
    print("All sim client tests passed.")
