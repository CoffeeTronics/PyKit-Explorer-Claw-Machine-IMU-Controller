# SPDX-FileCopyrightText: 2024 Microchip Technology Inc.
# SPDX-License-Identifier: MIT

"""
RNBD451 Bluetooth Low Energy Module - CircuitPython Driver
==========================================================

Targets: Microchip Curiosity CircuitPython board (SAMD51)
         board.BLE_TX / BLE_RX / BLE_CLR

Default UART: 115200 8N1 (module factory default)

Two operating modes
-------------------
* DATA mode  – module advertises / transfers transparent UART bytes.
               This is the power-on default.
* COMMAND mode – ASCII command interface.  Enter with "$$$", exit with "---".

Typical central (scanner/connector) flow
-----------------------------------------
    ble = RNBD451(uart, reset_pin=board.BLE_CLR)
    ble.hard_reset()
    ble.enter_command_mode()
    devices = ble.scan()
    addr, addr_type = devices[0]["address"], devices[0]["addr_type"]
    ble.connect(addr, addr_type)
    ble.exit_command_mode()        # back to data mode / transparent UART
    ble.write(b"hello")
    data = ble.read_available()

Typical peripheral (advertiser) flow
--------------------------------------
    ble = RNBD451(uart, reset_pin=board.BLE_CLR)
    ble.hard_reset()
    # Module starts advertising automatically in data mode.
    # Wait for %CONNECT% async status.
    ble.wait_for_connection(timeout=30)
    data = ble.read_available()
    ble.write(b"world")
"""

import time
import busio


# ── Response / status tokens ───────────────────────────────────────────────────
_CMD_PROMPT     = b"CMD>"
_END            = b"END"
_AOK            = b"AOK"
_ERR            = b"Err"
_SCANNING       = b"Scanning"
_TRYING         = b"Trying"
_REBOOTING      = b"Rebooting"

# Async status strings (wrapped in % by default delimiter)
_S_REBOOT       = b"%REBOOT%"
_S_CONNECT      = b"%CONNECT"       # prefix – rest carries address + handle
_S_DISCONNECT   = b"%DISCONNECT%"
_S_SECURED      = b"%SECURED%"
_S_BONDED       = b"%BONDED%"
_S_STREAM_OPEN  = b"%STREAM_OPEN%"

# Timeouts (seconds)
_T_CMD          = 2.0    # normal command round-trip
_T_SCAN         = 10.0   # maximum scan window
_T_CONNECT      = 15.0   # TCP-like connection attempt
_T_REBOOT       = 5.0    # module reboot
_T_READLINE     = 0.05   # polling interval


class RNBD451Error(Exception):
    """Raised when the module returns an error or times out."""


class RNBD451:
    """
    Driver for the RNBD451 BLE module over UART.

    Parameters
    ----------
    uart : busio.UART
        Pre-constructed UART peripheral connected to the module.
        Must be 115200 baud, 8N1 (module factory default).
    reset_pin : digitalio.DigitalInOut, optional
        Active-low hardware reset pin (board.BLE_CLR on the Curiosity board).
        If supplied, ``hard_reset()`` drives it.
    status_delimiter : str
        Character used to wrap asynchronous status messages.
        Default ``"%"`` matches module factory setting.
    """

    def __init__(self, uart, reset_pin=None, status_delimiter="%"):
        self._uart = uart
        self._reset_pin = reset_pin
        self._delim = status_delimiter.encode()
        self._in_command_mode = False
        self._connected = False
        self._peer_address = None

    # ── Low-level I/O ─────────────────────────────────────────────────────────

    def _write_raw(self, data: bytes) -> None:
        self._uart.write(data)

    def _readline(self, timeout: float = _T_READLINE) -> bytes:
        """
        Return one CRLF-terminated line or b"" on timeout.
        Strips trailing whitespace / CRLF.
        """
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self._uart.read(1)
            if b is None:
                time.sleep(0.005)
                continue
            buf += b
            if buf.endswith(b"\r\n") or buf.endswith(b"\n"):
                return bytes(buf).strip()
        return bytes(buf).strip()

    def _read_until(
        self,
        *tokens: bytes,
        timeout: float = _T_CMD,
        collect_lines: bool = False,
    ):
        """
        Read lines until one contains any of *tokens*.

        Returns
        -------
        tuple (matched_token, collected_lines_list)
        matched_token is None on timeout.
        """
        deadline = time.monotonic() + timeout
        lines = []
        while time.monotonic() < deadline:
            line = self._readline(timeout=min(0.1, deadline - time.monotonic()))
            if not line:
                continue
            # Dispatch async status strings encountered mid-command
            self._handle_async(line)
            if collect_lines:
                lines.append(line)
            for tok in tokens:
                if tok in line:
                    return tok, lines
        return None, lines

    def _handle_async(self, line: bytes) -> None:
        """
        Update internal state from asynchronous status strings.
        Called transparently from ``_read_until``.
        """
        if _S_CONNECT in line:
            self._connected = True
            # Parse %CONNECT,<addr_type>,<addr>,<handle>%
            try:
                inner = line.strip(self._delim)
                parts = inner.split(b",")
                if len(parts) >= 3:
                    self._peer_address = parts[2].decode().strip()
            except Exception:
                pass
        elif _S_DISCONNECT in line:
            self._connected = False
            self._peer_address = None
        elif _S_STREAM_OPEN in line:
            self._stream_open = True

    def _send_cmd(self, cmd: str, timeout: float = _T_CMD) -> str:
        """
        Send an ASCII command (CR appended automatically) and wait for
        AOK or Err.  Returns the full response block as a string.

        Raises RNBD451Error on Err response or timeout.
        """
        if not self._in_command_mode:
            raise RNBD451Error("Not in command mode. Call enter_command_mode() first.")
        payload = (cmd + "\r\n").encode()
        self._write_raw(payload)
        matched, lines = self._read_until(
            _AOK, _ERR, _CMD_PROMPT, timeout=timeout, collect_lines=True
        )
        text = b"\n".join(lines).decode()
        if matched == _ERR or (matched is None):
            if matched is None:
                raise RNBD451Error(f"Timeout waiting for response to: {cmd!r}")
            raise RNBD451Error(f"Module returned error for: {cmd!r}\n{text}")
        return text

    # ── Mode control ──────────────────────────────────────────────────────────

    def hard_reset(self, delay: float = 0.1, settle: float = 1.5) -> None:
        """
        Perform a hardware reset via the RST / BLE_CLR pin (active-low).
        Falls back to software reboot (R,1) if no pin was supplied.

        Parameters
        ----------
        delay   : float  Seconds to hold reset asserted.
        settle  : float  Seconds to wait after release for firmware boot.
        """
        if self._reset_pin is not None:
            self._reset_pin.value = False
            time.sleep(delay)
            self._reset_pin.value = True
        else:
            # Software path – only works if already in command mode
            if not self._in_command_mode:
                self.enter_command_mode()
            self._write_raw(b"R,1\r\n")

        self._in_command_mode = False
        self._connected = False
        self._peer_address = None

        # Wait for %REBOOT%
        matched, _ = self._read_until(_S_REBOOT, timeout=settle)
        # Allow extra settle time regardless
        time.sleep(0.3)

    def enter_command_mode(self, retries: int = 3) -> None:
        """
        Enter RNBD451 Command mode by sending the ``$$$`` sequence.

        The spec requires ``$$$`` arrive in a clean 1-second window with
        no other bytes before or after.  We flush the RX buffer first,
        wait a quiet gap, then send.

        Raises RNBD451Error if CMD> prompt is not received.
        """
        if self._in_command_mode:
            return

        for attempt in range(retries):
            # Drain anything pending
            while self._uart.in_waiting:
                self._uart.read(self._uart.in_waiting)
            time.sleep(0.1)

            # Send $$$ with no trailing CR (spec: just the three chars)
            self._write_raw(b"$$$")
            # Wait for CMD>
            matched, _ = self._read_until(_CMD_PROMPT, timeout=2.0)
            if matched == _CMD_PROMPT:
                self._in_command_mode = True
                return
            time.sleep(0.2)

        raise RNBD451Error("Failed to enter command mode after $$$")

    def exit_command_mode(self) -> None:
        """
        Exit Command mode and return to Data mode using ``---``.
        The module responds with ``END``.
        """
        if not self._in_command_mode:
            return
        self._write_raw(b"---\r\n")
        self._read_until(_END, timeout=_T_CMD)
        self._in_command_mode = False

    # ── System commands ───────────────────────────────────────────────────────

    def get_firmware_version(self) -> str:
        """Return firmware version string (V command)."""
        self._write_raw(b"V\r\n")
        _, lines = self._read_until(_CMD_PROMPT, _AOK, timeout=_T_CMD, collect_lines=True)
        return b"\n".join(lines).decode().strip()

    def get_device_info(self) -> str:
        """Return device info dump (D command): MAC, name, auth, features."""
        return self._send_cmd("D")

    def get_mac_address(self) -> str:
        """
        Return the module's Bluetooth MAC address.
        Parses the D command output – first non-blank line after the header.
        """
        info = self.get_device_info()
        for line in info.splitlines():
            line = line.strip()
            if len(line) == 12 and all(c in "0123456789ABCDEFabcdef" for c in line):
                return line
        return info.splitlines()[0].strip() if info else ""

    def get_connection_status(self):
        """
        Return connection info dict or None if not connected.

        Dict keys: ``address``, ``addr_type``, ``transparent_uart``
        """
        self._write_raw(b"GK\r\n")
        _, lines = self._read_until(_CMD_PROMPT, _AOK, timeout=_T_CMD, collect_lines=True)
        for line in lines:
            line = line.strip()
            if b"None" in line or b"none" in line:
                return None
            parts = line.split(b",")
            if len(parts) >= 3:
                return {
                    "address":         parts[0].decode().strip(),
                    "addr_type":       int(parts[1]),
                    "transparent_uart": int(parts[2]) == 1,
                }
        return None

    def reboot(self) -> None:
        """Software reboot (R,1). Waits for %REBOOT% status."""
        self._write_raw(b"R,1\r\n")
        self._in_command_mode = False
        self._connected = False
        self._read_until(_S_REBOOT, timeout=_T_REBOOT)
        time.sleep(0.3)

    def factory_reset(self, keep_private_services: bool = True) -> None:
        """
        Factory reset.  Module reboots immediately.

        Parameters
        ----------
        keep_private_services : bool
            True  → SF,1 (preserves private GATT services)
            False → SF,2 (clears everything including private services)
        """
        param = "1" if keep_private_services else "2"
        self._write_raw(f"SF,{param}\r\n".encode())
        self._in_command_mode = False
        self._connected = False
        self._read_until(_S_REBOOT, timeout=_T_REBOOT)
        time.sleep(0.3)

    # ── GAP / Device name ─────────────────────────────────────────────────────

    def set_device_name(self, name: str) -> None:
        """Set the BLE device name (SN command, up to 20 chars)."""
        if len(name) > 20:
            raise ValueError("Device name must be 20 characters or fewer")
        self._send_cmd(f"SN,{name}")

    def set_serialized_name(self, name: str) -> None:
        """
        Set device name with last-2-byte MAC appended (S- command).
        Useful for unique workshop / hackathon demo naming.
        Up to 15 chars for the base name.
        """
        if len(name) > 15:
            raise ValueError("Base name must be 15 characters or fewer")
        self._send_cmd(f"S-,{name}")

    # ── Advertising ──────────────────────────────────────────────────────────

    def start_advertising(
        self,
        interval_ms: int = 100,
        duration_ms: int = 0,
    ) -> None:
        """
        Start BLE advertising (A command).

        Parameters
        ----------
        interval_ms : int
            Advertisement interval in ms.  Range 20–40959 ms.
            Converted to 0.625 ms units internally.
        duration_ms : int
            Total advertising time in ms (0 = indefinite).
        """
        interval_units = max(0x20, min(0xFFFF, int(interval_ms / 0.625)))
        if duration_ms == 0:
            self._send_cmd(f"A,{interval_units:04X},{interval_units:04X}")
        else:
            dur_units = min(0xFFFF, int(duration_ms / 0.625))
            self._send_cmd(f"A,{interval_units:04X},{interval_units:04X},{dur_units:04X}")

    def stop_advertising(self) -> None:
        """Stop advertising (Y command)."""
        self._send_cmd("Y")

    def set_default_services(self, device_info: bool = True, transparent_uart: bool = True) -> None:
        """
        Configure supported GATT services bitmap (SS command).

        Parameters
        ----------
        device_info      : bool  Enable Device Information service (0x80)
        transparent_uart : bool  Enable Transparent UART service (0x40)
        """
        bitmap = 0x00
        if device_info:
            bitmap |= 0x80
        if transparent_uart:
            bitmap |= 0x40
        self._send_cmd(f"SS,{bitmap:02X}")

    # ── Scanning ──────────────────────────────────────────────────────────────

    def scan(
        self,
        duration: float = 5.0,
        interval_ms: float = 375.0,
        window_ms: float = 250.0,
    ) -> list:
        """
        Perform an active BLE scan and return a list of discovered devices.

        Parameters
        ----------
        duration    : float  Total scan time in seconds.
        interval_ms : float  Scan interval in ms (default 375 ms).
        window_ms   : float  Scan window in ms (default 250 ms); must be ≤ interval.

        Returns
        -------
        list of dict, each with keys:
            ``address``   – 12-char hex MAC string
            ``addr_type`` – 0 (public) or 1 (random)
            ``name``      – device name or empty string
            ``rssi``      – RSSI in dBm (int)
            ``connectable`` – bool
            ``raw``       – raw status line bytes
        """
        if not self._in_command_mode:
            raise RNBD451Error("Not in command mode")
        interval_units = max(4, min(0xFFFF, int(interval_ms / 0.625)))
        window_units   = max(4, min(interval_units, int(window_ms / 0.625)))

        self._write_raw(f"F,{interval_units:04X},{window_units:04X}\r\n".encode())

        # Wait for "Scanning" acknowledgement
        matched, _ = self._read_until(_SCANNING, timeout=2.0)
        if matched is None:
            raise RNBD451Error("Scan did not start (no 'Scanning' response)")

        devices = {}
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline:
            line = self._readline(timeout=0.2)
            if not line:
                continue
            if line.startswith(self._delim):
                dev = self._parse_scan_result(line)
                if dev:
                    # Deduplicate by address
                    devices[dev["address"]] = dev
            elif line.startswith(b"AOK") or _CMD_PROMPT in line:
                break

        # Stop scan
        self._write_raw(b"X\r\n")
        self._read_until(_AOK, _CMD_PROMPT, timeout=2.0)

        return list(devices.values())

    def _parse_scan_result(self, line: bytes) -> dict:
        """
        Parse one advertising report line.

        Connectable format:
            %<Address>,<Addr_Type>,<Name>,<UUIDs>,<RSSI>,<TxPHY>,<RxPHY>%
        Non-connectable format:
            %<Address>,<Addr_Type>,<RSSI>,Brcst:<Payload>%
        """
        try:
            inner = line.strip(self._delim)
            parts = inner.split(b",")
            if len(parts) < 3:
                return None

            address   = parts[0].decode().strip()
            addr_type = int(parts[1])

            # Heuristic: if parts[3] looks like a UUID list or name → connectable
            connectable = len(parts) >= 5 and not parts[2].startswith(b"Brcst")

            if connectable:
                name = parts[2].decode().strip()
                # RSSI is the 5th field (index 4)
                rssi = int(parts[4]) if len(parts) > 4 else 0
            else:
                name = ""
                rssi = int(parts[2]) if len(parts) > 2 else 0

            return {
                "address":     address,
                "addr_type":   addr_type,
                "name":        name,
                "rssi":        rssi,
                "connectable": connectable,
                "raw":         line,
            }
        except Exception:
            return None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(
        self,
        address: str,
        addr_type: int = 0,
        timeout: float = _T_CONNECT,
    ) -> None:
        """
        Initiate a connection to a remote BLE device (C command).

        Parameters
        ----------
        address   : str  12-char hex MAC address (e.g. "3481F4A8436A")
        addr_type : int  0 = public, 1 = random
        timeout   : float  Seconds to wait for %CONNECT% status.

        Raises RNBD451Error on failure.
        """
        if not self._in_command_mode:
            raise RNBD451Error("Not in command mode")
        self._write_raw(f"C,{addr_type},{address}\r\n".encode())

        # Expect "Trying" then %CONNECT%
        matched, _ = self._read_until(_TRYING, b"Err", b"ERR", timeout=3.0)
        if matched in (b"Err", b"ERR", None):
            raise RNBD451Error(f"Connection attempt rejected for {address}")

        matched, _ = self._read_until(_S_CONNECT, _S_DISCONNECT, b"ERR", timeout=timeout)
        if matched != _S_CONNECT and _S_CONNECT not in (matched or b""):
            raise RNBD451Error(f"Failed to connect to {address} (timeout or error)")

        self._connected = True
        self._peer_address = address

    def connect_last_bonded(self, timeout: float = _T_CONNECT) -> None:
        """
        Reconnect to the last bonded device (C command, no params).
        Automatically secures the link on reconnect.
        """
        self._write_raw(b"C\r\n")
        matched, _ = self._read_until(_S_CONNECT, b"ERR", b"Err", timeout=timeout)
        if matched not in (_S_CONNECT,) and _S_CONNECT not in (matched or b""):
            raise RNBD451Error("Failed to reconnect to last bonded device")
        self._connected = True

    def disconnect(self) -> None:
        """Disconnect the active BLE link (K,1 command)."""
        self._write_raw(b"K,1\r\n")
        matched, _ = self._read_until(_AOK, _S_DISCONNECT, b"Err", timeout=_T_CMD)
        self._connected = False
        self._peer_address = None

    def cancel_connect(self) -> None:
        """Cancel a pending connection attempt (Z command)."""
        self._write_raw(b"Z\r\n")
        self._read_until(_AOK, b"Err", timeout=_T_CMD)

    # ── Security / bonding ────────────────────────────────────────────────────

    def bond(self) -> None:
        """
        Initiate bonding with the currently connected peer (B command).
        Waits for %SECURED% and %BONDED% status strings.
        """
        if not self._connected:
            raise RNBD451Error("Not connected – cannot bond")
        self._write_raw(b"B\r\n")
        matched, _ = self._read_until(_S_BONDED, b"ERR", b"%ERR_SEC%", timeout=10.0)
        if matched != _S_BONDED:
            raise RNBD451Error("Bonding failed")

    def set_pairing_mode(self, mode: int = 2, secure_only: bool = False) -> None:
        """
        Set I/O capability / authentication method (SA command).

        mode values
        -----------
        0 – No Input No Output with Bonding  (auto-bond; default for M2M)
        1 – Display Yes/No
        2 – No Input No Output  (no auto-bond; module factory default)
        3 – Keyboard Only
        4 – Display Only
        5 – Keyboard Display

        secure_only : bool
            True enables LE Secure Connections Only mode.
        """
        if mode not in range(6):
            raise ValueError("mode must be 0–5")
        param = f"SA,{mode}" + (",1" if secure_only else "")
        self._send_cmd(param)

    def list_bonded_devices(self) -> list:
        """
        Return a list of bonded device dicts (LB command).
        Each dict has keys: ``index``, ``address``, ``addr_type``.
        """
        self._write_raw(b"LB\r\n")
        _, lines = self._read_until(_CMD_PROMPT, _AOK, timeout=_T_CMD, collect_lines=True)
        devices = []
        for line in lines:
            line = line.strip()
            parts = line.split(b",")
            if len(parts) >= 3:
                try:
                    devices.append({
                        "index":     int(parts[0]),
                        "address":   parts[1].decode().strip(),
                        "addr_type": int(parts[2]),
                    })
                except (ValueError, IndexError):
                    pass
        return devices

    # ── Transparent UART data mode ────────────────────────────────────────────

    def write(self, data: bytes) -> None:
        """
        Send raw bytes through the transparent UART data pipe.
        Module must be in DATA mode and connected.
        """
        if self._in_command_mode:
            raise RNBD451Error("Exit command mode before writing transparent UART data")
        self._uart.write(data)

    def read_available(self, size: int = 256) -> bytes:
        """
        Read up to *size* bytes from the transparent UART pipe.
        Returns b"" if nothing is available.
        """
        if self._in_command_mode:
            raise RNBD451Error("Exit command mode before reading transparent UART data")
        if self._uart.in_waiting:
            return self._uart.read(min(size, self._uart.in_waiting)) or b""
        return b""

    def wait_for_connection(self, timeout: float = 30.0) -> bool:
        """
        Block until a %CONNECT% status arrives (peripheral role).
        Returns True if connected, False on timeout.
        Must be in DATA mode.
        """
        if self._connected:
          return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._readline(timeout=0.2)
            if line and _S_CONNECT in line:
                self._handle_async(line)
                return True
        return False

    def wait_for_stream_open(self, timeout: float = 10.0) -> bool:
        """
        Block until %STREAM_OPEN% is received (transparent UART ready).
        Returns True on success.
        """
        self._stream_open = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._readline(timeout=0.2)
            if line:
                self._handle_async(line)
                if self._stream_open:
                    return True
        return False

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        """True if a BLE connection is currently active."""
        return self._connected

    @property
    def peer_address(self):
        """MAC address of the connected peer, or None."""
        return self._peer_address

    @property
    def in_command_mode(self) -> bool:
        """True if the module is currently in Command mode."""
        return self._in_command_mode
