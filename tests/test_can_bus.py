import can

from control_lib import can_bus


class FakeBus:
    def __init__(self):
        self.sent_messages = []
        self.responses = []
        self.shutdown_called = False

    def send(self, msg):
        self.sent_messages.append(msg)

    def recv(self, timeout=None):
        if self.responses:
            return self.responses.pop(0)
        return None

    def queue_response(self, msg):
        self.responses.append(msg)

    def shutdown(self):
        self.shutdown_called = True


def test_send_sdo_write_and_read_round_trip():
    fake_buses = {}

    def provider(channel: str):
        bus = FakeBus()
        fake_buses[channel] = bus
        return bus

    can_bus.set_bus_provider(provider)
    try:
        can_bus.ensure_buses(["canX"])
        bus = fake_buses["canX"]

        can_bus.send_sdo_write("canX", 1, 0x1234, 0, 42, 2, sleep=0)
        assert bus.sent_messages, "SDO write did not send any CAN frame"
        write_msg = bus.sent_messages[-1]
        assert write_msg.arbitration_id == 0x601

        response_data = [0x43, 0x34, 0x12, 0x00, 0x2A, 0x00, 0x00, 0x00]
        bus.queue_response(
            can.Message(arbitration_id=0x581, data=response_data, is_extended_id=False)
        )

        value = can_bus.send_sdo_read("canX", 1, 0x1234, 0, timeout=0.01)
        assert value == 42
    finally:
        can_bus.shutdown_buses()
        can_bus.reset_bus_provider()

    assert fake_buses["canX"].shutdown_called
