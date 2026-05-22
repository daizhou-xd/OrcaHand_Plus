import os
import sys
import tempfile
import serial
import time
import threading
import contextlib


def _ocra_temp_dir() -> str:
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


_configure_pycache_prefix()


class ServoProtocol:
    @staticmethod
    def checksum(data):
        return (~sum(data) & 0xFF)

    @staticmethod
    def make_write_pos_cmd(servo_id, position, time_ms=0, speed=1000):
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF

        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF

        spd_l = speed & 0xFF
        spd_h = (speed >> 8) & 0xFF

        packet = [
            0xFF, 0xFF,
            servo_id,
            0x09,
            0x03,
            0x2A,
            pos_l, pos_h,
            time_l, time_h,
            spd_l, spd_h
        ]
        packet.append(ServoProtocol.checksum(packet[2:]))
        return bytes(packet)

    @staticmethod
    def make_read_pos_cmd(servo_id):
        packet = [
            0xFF, 0xFF,
            servo_id,
            0x04,
            0x02,
            0x38,
            0x02
        ]
        packet.append(ServoProtocol.checksum(packet[2:]))
        return bytes(packet)

    @staticmethod
    def make_write_data_cmd(servo_id, start_addr, data: bytes | bytearray | list[int]):
        """通用写控制表（WRITEDATA 0x03）。

        包格式：FF FF, ID, Length, Instruction=0x03, Param1=addr, Param2..=data
        其中 Length = (1 + len(data)) + 2
        """
        start_addr = int(start_addr) & 0xFF
        if isinstance(data, (bytes, bytearray)):
            payload = bytes(data)
        else:
            payload = bytes(int(x) & 0xFF for x in data)

        params = bytes([start_addr]) + payload
        length = len(params) + 2
        packet = [0xFF, 0xFF, int(servo_id) & 0xFF, int(length) & 0xFF, 0x03]
        packet.extend(list(params))
        packet.append(ServoProtocol.checksum(packet[2:]))
        return bytes(packet)

    @staticmethod
    def make_sync_read_cmd(servo_ids, start_addr, data_len):
        """同步读 SYNCREAD（指令 0x82）。

        手册格式：
        头 FF FF, ID=0xFE, Length=N+4, Instruction=0x82,
        Param1=首地址, Param2=读取长度, Param3..=要查询的舵机ID列表。
        """
        ids = [int(i) for i in servo_ids]
        ids = [i for i in ids if 0 <= i <= 253]
        start_addr = int(start_addr) & 0xFF
        data_len = int(data_len) & 0xFF

        packet = [0xFF, 0xFF, 0xFE, len(ids) + 4, 0x82, start_addr, data_len]
        packet.extend(ids)
        packet.append(ServoProtocol.checksum(packet[2:]))
        return bytes(packet)

    @staticmethod
    def make_sync_write_cmd(servo_data, start_addr, data_len):
        """同步写 SYNCWRITE（指令 0x83）。

        手册格式：
        头 FF FF, ID=0xFE, Length=(data_len+1)*N + 4, Instruction=0x83,
        Param1=首地址, Param2=写入长度, Param3..= [ID1, Data..., ID2, Data..., ...]

        servo_data: iterable[(id:int, payload:bytes|bytearray|list[int])]
        """
        start_addr = int(start_addr) & 0xFF
        data_len = int(data_len) & 0xFF

        blocks = []
        for sid, payload in servo_data:
            sid = int(sid)
            if not (0 <= sid <= 253):
                continue
            if isinstance(payload, (bytes, bytearray)):
                b = bytes(payload)
            else:
                b = bytes(int(x) & 0xFF for x in payload)
            if len(b) != int(data_len):
                continue
            blocks.append((sid, b))

        if not blocks:
            return b""

        length = (int(data_len) + 1) * len(blocks) + 4
        if length > 0xFF:
            raise ValueError("SYNCWRITE packet too long")

        packet = [0xFF, 0xFF, 0xFE, int(length), 0x83, start_addr, data_len]
        for sid, b in blocks:
            packet.append(int(sid) & 0xFF)
            packet.extend(b)
        packet.append(ServoProtocol.checksum(packet[2:]))
        return bytes(packet)


class ServoBus:
    def __init__(self, port, baudrate=1_000_000):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.02
        )
        self.lock = threading.Lock()

    def write_position(self, servo_id, position, time_ms=0, speed=1000):
        position = max(0, min(4095, int(position)))
        cmd = ServoProtocol.make_write_pos_cmd(
            servo_id, position, time_ms, speed
        )
        with self.lock:
            self.ser.write(cmd)

    def write_data(self, servo_id: int, start_addr: int, data: bytes | bytearray | list[int]):
        cmd = ServoProtocol.make_write_data_cmd(int(servo_id), int(start_addr), data)
        with self.lock:
            self.ser.write(cmd)

    def write_u8(self, servo_id: int, start_addr: int, value: int):
        self.write_data(int(servo_id), int(start_addr), [int(value) & 0xFF])

    @contextlib.contextmanager
    def _temp_timeout(self, max_wait: float):
        old_timeout = getattr(self.ser, "timeout", None)
        try:
            if old_timeout is not None:
                self.ser.timeout = min(float(old_timeout), float(max_wait))
        except Exception:
            old_timeout = None
        try:
            yield
        finally:
            if old_timeout is not None:
                try:
                    self.ser.timeout = old_timeout
                except Exception:
                    pass

    def _read_some(self) -> bytes:
        return self.ser.read(self.ser.in_waiting or 1)

    def _read_until(self, *, max_wait: float, min_bytes: int) -> bytes:
        start = time.time()
        buf = b""
        while time.time() - start < float(max_wait):
            buf += self._read_some()
            if len(buf) >= int(min_bytes):
                break
        return buf

    @staticmethod
    def _checksum_ok(pkt: bytes) -> bool:
        if len(pkt) < 6:
            return False
        calc = ServoProtocol.checksum(list(pkt[2:-1]))
        return calc == pkt[-1]

    @classmethod
    def _drain_packets(cls, buffer: bytes):
        out = []
        i = 0
        while True:
            idx = buffer.find(b"\xFF\xFF", i)
            if idx < 0:
                break
            if len(buffer) < idx + 4:
                break
            length = buffer[idx + 3]
            total = int(length) + 4
            if len(buffer) < idx + total:
                break
            pkt = buffer[idx:idx + total]
            if cls._checksum_ok(pkt):
                out.append(pkt)
                i = idx + total
            else:
                i = idx + 2
        return out, buffer[i:]

    def read_position(self, servo_id, max_wait=0.03):
        cmd = ServoProtocol.make_read_pos_cmd(servo_id)
        with self.lock:
            with self._temp_timeout(max_wait):
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                buf = self._read_until(max_wait=float(max_wait), min_bytes=8)

        if len(buf) < 8:
            return None
        if buf[0] != 0xFF or buf[1] != 0xFF:
            return None
        if buf[2] != servo_id or buf[4] != 0x00:
            return None

        return buf[5] | (buf[6] << 8)

    def sync_read(self, servo_ids, start_addr=0x38, data_len=2, max_wait=0.05):
        """同步读多个舵机的控制表内容，返回 dict[id] = bytes 或 None。

        典型用法：读取当前位置：start_addr=0x38, data_len=2。
        """
        ids = [int(i) for i in servo_ids]
        ids = [i for i in ids if 0 <= i <= 253]
        if not ids:
            return {}

        cmd = ServoProtocol.make_sync_read_cmd(ids, start_addr, data_len)
        expected_len = int(data_len) + 2  # status包 length 字段 = data_len + 2
        results = {sid: None for sid in ids}

        with self.lock:
            with self._temp_timeout(max_wait):
                self.ser.reset_input_buffer()
                self.ser.write(cmd)

                start = time.time()
                buf = b""
                got = set()
                while time.time() - start < float(max_wait):
                    buf += self._read_some()
                    packets, buf = self._drain_packets(buf)
                    for pkt in packets:
                        sid = pkt[2]
                        length = pkt[3]
                        if sid not in results:
                            continue
                        if int(length) != expected_len:
                            continue
                        status = pkt[4]
                        if status != 0x00:
                            results[sid] = None
                            got.add(sid)
                            continue
                        payload = pkt[5:-1]
                        if len(payload) != int(data_len):
                            continue
                        results[sid] = payload
                        got.add(sid)
                    if len(got) == len(results):
                        break

        return results

    def sync_read_u16(self, servo_ids, start_addr: int, *, max_wait: float = 0.05) -> dict[int, int | None]:
        """同步读取 16-bit 无符号值（小端）并返回 dict[id] -> int|None。"""
        raw = self.sync_read(servo_ids, start_addr=int(start_addr) & 0xFF, data_len=2, max_wait=max_wait)
        out: dict[int, int | None] = {}
        for sid, b in raw.items():
            if not b or len(b) != 2:
                out[int(sid)] = None
            else:
                out[int(sid)] = int(b[0] | (b[1] << 8))
        return out

    # ---- Current feedback (SRAM 0x45, len=2) ----
    # Doc: 6.5mA per LSB, max 500 * 6.5mA = 3250mA.
    CURRENT_ADDR = 0x45
    CURRENT_LSB_MA = 6.5

    # Best-effort torque switch (common for SMS/STS/Feetech-like tables).
    # If your model uses a different address, adjust here.
    try:
        TORQUE_ENABLE_ADDR = int(str(os.environ.get("OCRA_TORQUE_ENABLE_ADDR", "0x28")), 0)
    except Exception:
        TORQUE_ENABLE_ADDR = 0x28

    def sync_read_current_raw(self, servo_ids, *, max_wait: float = 0.05) -> dict[int, int | None]:
        return self.sync_read_u16(servo_ids, start_addr=self.CURRENT_ADDR, max_wait=max_wait)

    def sync_read_current_ma(self, servo_ids, *, max_wait: float = 0.05) -> dict[int, float | None]:
        raw = self.sync_read_current_raw(servo_ids, max_wait=max_wait)
        out: dict[int, float | None] = {}
        for sid, v in raw.items():
            out[int(sid)] = None if v is None else float(v) * float(self.CURRENT_LSB_MA)
        return out

    def sync_write(self, servo_data, start_addr, data_len, *, max_wait=0.02):
        """同步写多个舵机控制表（SYNCWRITE 0x83）。

        注意：SYNCWRITE 没有应答包，这里只负责发送。
        """
        cmd = ServoProtocol.make_sync_write_cmd(servo_data, start_addr, data_len)
        if not cmd:
            return

        with self.lock:
            with self._temp_timeout(float(max_wait)):
                # 同步写无回包，清空输入避免旧数据干扰后续 read
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                self.ser.write(cmd)

    def sync_write_positions(self, moves, *, time_ms=0, speed=1000, max_wait=0.02):
        """同步写位置（首地址 0x2A，长度 6：pos2 + time2 + speed2）。"""
        data = []
        for sid, position in moves:
            position = max(0, min(4095, int(position)))
            t = max(0, min(0xFFFF, int(time_ms)))
            spd = max(0, min(0xFFFF, int(speed)))
            payload = bytes([
                position & 0xFF,
                (position >> 8) & 0xFF,
                t & 0xFF,
                (t >> 8) & 0xFF,
                spd & 0xFF,
                (spd >> 8) & 0xFF,
            ])
            data.append((int(sid), payload))
        self.sync_write(data, start_addr=0x2A, data_len=6, max_wait=max_wait)


    def close(self):
        self.ser.close()
