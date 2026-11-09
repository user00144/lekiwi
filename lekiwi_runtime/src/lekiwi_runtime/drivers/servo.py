"""One serial connection shared by arm reads and wheel writes."""


class ServoBus:
    def __init__(self, config: dict, motor_ids=range(1, 7)):
        self.config = config
        self.motor_ids = motor_ids
        self.port = None
        self.port_handler = None
        self.packet_handler = None

    def _open(self, port):
        import scservo_sdk as sdk

        bus = sdk.PortHandler(port)
        try:
            if not bus.openPort() or not bus.setBaudRate(self.config["baudrate"]):
                raise OSError(f"Cannot open servo port: {port}")
            return bus
        except BaseException:
            bus.closePort()
            raise

    def connect(self) -> None:
        if self.port_handler is not None:
            return
        import scservo_sdk as sdk
        from serial.tools import list_ports

        self.packet_handler = sdk.PacketHandler(0)
        port = self.config.get("port")
        if not port:
            candidates = []
            # Restrict discovery to servo USB adapters, excluding TI radar UARTs.
            for info in sorted(list_ports.comports(), key=lambda item: item.device):
                if info.vid not in (0x1A86, 0x0403, 0x10C4):
                    continue
                bus = None
                try:
                    bus = self._open(info.device)
                    replies = [self.packet_handler.ping(bus, mid) for mid in self.motor_ids]
                    if all(comm == sdk.COMM_SUCCESS and error == 0 for _, comm, error in replies):
                        candidates.append(info.device)
                except OSError:
                    continue
                finally:
                    if bus is not None:
                        bus.closePort()
            if len(candidates) != 1:
                raise OSError("Set arm.port explicitly: expected one servo board, "
                              f"found {len(candidates)}")
            port = candidates[0]
        self.port_handler = self._open(port)
        self.port = port

    def close(self) -> None:
        if self.port_handler is not None:
            try:
                self.port_handler.closePort()
            finally:
                self.port_handler = None

