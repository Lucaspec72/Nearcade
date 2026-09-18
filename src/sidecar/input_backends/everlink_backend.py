"""
Nearcade — EverLink Relay backend.

Speaks the EverLink Host<->Relay wire protocol (see EverLink_Protocol.md,
vendored at docs/EverLink_Protocol.md) directly over USB-serial to one or more
EverLink Relay boards (ESP32 running the EverLink Relay firmware). Each Relay
turns Nearcade's controller state into a real USB HID Xbox 360 gamepad the
console/capture target can see — the same role EverLink's original Windows-only
C#/WPF Host app played, reimplemented here so it works on every OS Nearcade
already supports (Windows, Linux, macOS) since pyserial's port enumeration and
serial I/O are all cross-platform, unlike the original Host's WMI-based port
scan and System.IO.Ports usage.

This backend does NOT reimplement EverLink's controller-selection UI, remap
editor, or rumble keep-alive UX — those are superseded by Nearcade's own
viewer/slot system (see InputOrchestrator.js). What's reimplemented is strictly
the wire protocol plus enough device-management plumbing (scan, identify,
assign, disconnect) for Nearcade's host.js panel and server.js WS handlers to
drive it. See docs/everlink-integration.md for the full design.

Protocol summary (authoritative doc: EverLink_Protocol.md):
  - Identification ping: write 0xFE, expect "IAM:EverLink:v<N>:<MAC>:<ChipModel>\\n"
    within a short timeout. No reply (or a garbled one) = not a Relay.
  - Controller packet (14 bytes, little-endian, sent at ~250Hz / every 4ms):
      offset 0      sync byte 0xA5
      offset 1-2    buttons uint16 LE
      offset 3      left trigger  uint8 (0-255)
      offset 4      right trigger uint8 (0-255)
      offset 5-6    left stick X  int16 LE
      offset 7-8    left stick Y  int16 LE
      offset 9-10   right stick X int16 LE
      offset 11-12  right stick Y int16 LE
      offset 13     checksum = XOR of bytes 1-12
  - Rumble (Relay -> Host, v2+ only): a text line "RMBL:<left>:<right>\\n", each
    0-255. Only sent on change; Host is expected to re-arm the physical
    controller's rumble on a steady cadence for as long as the last state is
    non-zero (EverLink's own keep-alive trick) — Nearcade doesn't have a
    "physical controller" to rumble on the Host side (input arrives from a
    remote viewer over WebRTC, not a local SDL gamepad), so incoming RMBL:
    lines are instead forwarded up to Node as a 'rumble' event, which
    server.js already knows how to route back to the owning viewer (see
    server.js's existing inputDriver.events.on('rumble', ...) — the C++
    uinputBridge path feeds that same event today).

JSON-lines protocol with InputOrchestrator.js (mirrors every other backend in
this directory — see windows_vigem.py's module docstring for the shared
shape). Everything this backend emits on stdout is one JSON object per line:
  {"type": "error",         "message": "...", "code": "..."}
  {"type": "log",           "message": "..."}
  {"type": "ready",         "message": "..."}
  {"type": "relay_list",    "relays": [ {mac, port, chipModel, protocolVersion,
                                          isUsbCapable, issues, assignedSlot,
                                          responsive}, ... ]}
  {"type": "relay_added",   "relay": {...}}
  {"type": "relay_removed", "mac": "..."}
  {"type": "rumble",        "viewerId": "...", "strong": 0-65535, "weak": 0-65535,
                             "duration": 200}

Messages this backend accepts on stdin (one JSON object per line):
  {"type": "everlink_scan"}
      Probes every currently-visible serial port for the identification
      reply, adds any newly-found Relay (opens its port, starts its 4ms send
      timer with no controller assigned yet), and replies with a fresh
      "relay_list". Safe to call repeatedly — already-open ports are skipped
      (mirrors Host's ScanForEverLinkRelays(skipPorts) behavior) rather than
      re-probed, since a port already confirmed as a Relay doesn't need to be
      re-identified, and probing a port some other process has isn't safe.
  {"type": "everlink_assign", "mac": "...", "pad_id": "..." | null}
      Assigns (or clears, with null pad_id) which slot's gamepad state this
      Relay streams. pad_id matches the same pad_id InputOrchestrator.js
      already uses everywhere else (viewerId, or viewerId_N for a
      multi-controller viewer) — NOT a slot index, so the assignment survives
      a slot renumber the same way viewer-keyed state elsewhere in this
      codebase does.
  {"type": "everlink_forget", "mac": "..."}
      Closes and forgets a specific Relay (e.g. the user unplugged it and
      wants it out of the list without waiting for a rescan).
  {"type": "everlink_list"}
      Re-emits "relay_list" without rescanning — used when host.js's panel
      just wants a refresh of current state (e.g. on open).
  {"type": "gamepad", ...}
      Same normalized gamepad message every other backend receives (pad_id,
      buttons, lt, rt, lx, ly, rx, ry — see InputOrchestrator.js's
      _validateGamepadMsg). Cached per pad_id; each Relay's own 4ms send loop
      reads the latest cached state for whichever pad_id is currently
      assigned to it, the same "poll freshest state right before sending"
      design SerialLink.OnTick used in the original Host (see
      EverLink-main/Host/SerialLink.cs).
  {"type": "disconnect_viewer", "viewer_id": "..."}
      Clears any Relay assignment(s) currently pointing at pad_ids under this
      viewer (pad_id == viewer_id or pad_id.startswith(viewer_id + "_")), so a
      disconnected viewer doesn't leave a Relay silently frozen on stale
      input. The Relay itself is NOT removed — only the assignment is
      cleared, matching "a Relay exists independently of whether a controller
      is assigned to it" from the original Host's RelayManager.
  {"type": "destroy_all"}
      Closes every open Relay connection and exits the send-loop thread
      cleanly. Sent by InputOrchestrator.destroy() on shutdown.

Dependencies: pyserial (`pip install pyserial`), auto-installed on first import
failure the same way every other Python backend in this folder handles a
missing dependency (see windows_vigem.py's vgamepad import block, mirrored
here).
"""

import sys
import json
import time
import threading


def _emit(payload: dict):
    """Write one JSON line to stdout, flushed immediately — the Node side
    (InputOrchestrator.js) reads stdout line-by-line, so an unflushed write
    can sit invisible in a buffer indefinitely on some platforms."""
    print(json.dumps(payload), flush=True)


def _log(msg: str):
    _emit({"type": "log", "message": msg})


def _error(msg: str, code: str = "EVERLINK_ERROR"):
    _emit({"type": "error", "message": msg, "code": code})


try:
    import serial
    from serial.tools import list_ports
except ImportError:
    _log("pyserial not found — attempting auto-install via pip...")
    import subprocess as _sp
    _result = _sp.run(
        [sys.executable, "-m", "pip", "install", "pyserial", "--quiet", "--no-warn-script-location"],
        capture_output=True, text=True
    )
    if _result.returncode == 0:
        _log("pyserial installed successfully — reloading...")
        try:
            import serial
            from serial.tools import list_ports
        except Exception as _e2:
            _error(f"pyserial installed but still failed to import: {_e2}", "PYSERIAL_MISSING")
            sys.exit(1)
    else:
        _error(
            "pyserial not installed and auto-install failed. Run: pip install pyserial",
            "PYSERIAL_MISSING"
        )
        sys.exit(1)


# ── Wire protocol constants (must match EverLink_Protocol.md exactly) ────────

BAUD_RATE = 921600
PING_BYTE = b"\xFE"
IDENT_PREFIX = "IAM:EverLink:v"
PING_TIMEOUT_S = 0.3
SYNC_BYTE = 0xA5
PACKET_SIZE = 14
SEND_INTERVAL_S = 0.004  # 250Hz, matching Host/SerialLink.cs's SendIntervalMs
HIGHEST_KNOWN_PROTOCOL_VERSION = 2

# Relay-responsiveness / rumble keep-alive timing — mirrors
# Host/SerialLink.cs's RelayResponsiveTimeout / RumbleKeepAliveInterval.
RELAY_RESPONSIVE_TIMEOUT_S = 3.0


def _is_usb_capable(chip_model: str) -> bool:
    """Mirrors DeviceInfo.IsUsbCapable in EverLink-main/Host/SerialLink.cs —
    only S2/S3/P4-family ESP32 chips have the native USB OTG peripheral
    needed to appear as a USB HID gamepad; plain ESP32/C3 do not, no matter
    what firmware they run."""
    up = (chip_model or "").upper()
    return "S2" in up or "S3" in up or "P4" in up


def _compatibility_issues(protocol_version: int, chip_model: str) -> list:
    """Mirrors DeviceInfo.CompatibilityIssues — a human-readable list of known
    feature gaps for this Relay, worded as informational notes (a v1 Relay or
    a non-USB-capable board are still usable for whatever they DO support),
    not hard errors."""
    issues = []
    effective = min(protocol_version, HIGHEST_KNOWN_PROTOCOL_VERSION)

    if protocol_version > HIGHEST_KNOWN_PROTOCOL_VERSION:
        issues.append(
            f"This Relay reports protocol v{protocol_version}, newer than the "
            f"v{HIGHEST_KNOWN_PROTOCOL_VERSION} this backend knows about. Treating it as "
            f"v{HIGHEST_KNOWN_PROTOCOL_VERSION} — it should still work, but newer features "
            "(if any) won't be available."
        )

    if effective < 2:
        issues.append(f"EverLink firmware v{protocol_version} doesn't support rumble.")

    if not _is_usb_capable(chip_model):
        issues.append(f"{chip_model} does not have USB OTG, impossible to use wired bridge.")

    return issues


def _checksum(buf: bytearray) -> int:
    c = 0
    for b in buf[1:13]:
        c ^= b
    return c


def encode_packet(buttons: int, lt: int, rt: int, lx: int, ly: int, rx: int, ry: int) -> bytes:
    """Builds one 14-byte EverLink controller packet — field-for-field
    identical to PacketProtocol.Encode in EverLink-main/Host/SerialLink.cs."""
    buf = bytearray(PACKET_SIZE)
    buf[0] = SYNC_BYTE
    buf[1] = buttons & 0xFF
    buf[2] = (buttons >> 8) & 0xFF
    buf[3] = lt & 0xFF
    buf[4] = rt & 0xFF

    def _write_i16(offset, value):
        v = value & 0xFFFF
        buf[offset] = v & 0xFF
        buf[offset + 1] = (v >> 8) & 0xFF

    _write_i16(5, lx)
    _write_i16(7, ly)
    _write_i16(9, rx)
    _write_i16(11, ry)
    buf[13] = _checksum(buf)
    return bytes(buf)


# JS viewer button bitmask (viewer.js's btnMask, same bits InputOrchestrator.js
# receives as msg.buttons) -> EverLink wire bitmask (EverLink_Protocol.md
# "Button bit layout"). The two are NOT the same layout bit-for-bit — only
# the shoulder buttons happen to share a position — so this needs an explicit
# per-bit remap, the same way InputOrchestrator.js's own _jsBtnsToCpp remaps
# the same JS bitmask into the C++ uinputBridge's W3C_BTN layout. Verified
# against viewer.js's btnMask assignment (search viewer.js for "btnMask |="):
#
#   JS bit    W3C button      EverLink bit
#   0x0001    A (btn 0)       0x1000  A
#   0x0002    B (btn 1)       0x2000  B
#   0x0004    X (btn 2)       0x4000  X
#   0x0008    Y (btn 3)       0x8000  Y
#   0x0100    LB (btn 4)      0x0100  LeftShoulder  (same position)
#   0x0200    RB (btn 5)      0x0200  RightShoulder (same position)
#   0x2000    Back (btn 8)    0x0020  Back
#   0x1000    Start (btn 9)   0x0010  Start
#   0x0400    L3 (btn 10)     0x0040  LeftThumb
#   0x0800    R3 (btn 11)     0x0080  RightThumb
#   0x0010    DUp (btn 12)    0x0001  DPadUp
#   0x0020    DDown (btn 13)  0x0002  DPadDown
#   0x0040    DLeft (btn 14)  0x0004  DPadLeft
#   0x0080    DRight (btn 15) 0x0008  DPadRight
#   0x4000    Guide (btn 16)  0x0400  Guide (in practice already stripped by
#                                      InputOrchestrator.js's _clampButtons
#                                      before this backend ever sees it — see
#                                      that function's comment: "Strip 0x4000
#                                      ... so viewers cannot open system
#                                      menus" — kept here anyway so this
#                                      mapping is correct if that policy ever
#                                      changes upstream)
_JS_TO_EVERLINK_BIT_MAP = (
    (0x0001, 0x1000),  # A
    (0x0002, 0x2000),  # B
    (0x0004, 0x4000),  # X
    (0x0008, 0x8000),  # Y
    (0x0100, 0x0100),  # LB / LeftShoulder
    (0x0200, 0x0200),  # RB / RightShoulder
    (0x2000, 0x0020),  # Back / Select
    (0x1000, 0x0010),  # Start
    (0x0400, 0x0040),  # L3 / LeftThumb
    (0x0800, 0x0080),  # R3 / RightThumb
    (0x0010, 0x0001),  # DPad Up
    (0x0020, 0x0002),  # DPad Down
    (0x0040, 0x0004),  # DPad Left
    (0x0080, 0x0008),  # DPad Right
    (0x4000, 0x0400),  # Guide
)


def _js_buttons_to_everlink(js_buttons: int) -> int:
    out = 0
    for js_bit, ev_bit in _JS_TO_EVERLINK_BIT_MAP:
        if js_buttons & js_bit:
            out |= ev_bit
    return out


# ── Relay bookkeeping ──────────────────────────────────────────────────────

class Relay:
    """One open serial connection to one confirmed EverLink Relay board, plus
    its own 250Hz send thread. Mirrors RelayConnection + SerialLink from
    EverLink-main/Host/RelayManager.cs and SerialLink.cs — a Relay exists (and
    streams) as soon as it's found, whether or not a pad_id is assigned to it
    yet."""

    def __init__(self, mac, port_name, chip_model, protocol_version):
        self.mac = mac
        self.port_name = port_name
        self.chip_model = chip_model
        self.protocol_version = protocol_version
        self.pad_id = None  # None = no controller assigned, same as EverLink's RelayConnection.Controller == null
        self.ser = None
        self.thread = None
        self.reader_thread = None
        self._stop = threading.Event()
        self.last_received_monotonic = None  # mirrors LastReceivedFromRelay
        self.packets_sent = 0
        self._rumble_left = 0
        self._rumble_right = 0
        self._last_rumble_keepalive = 0.0
        self._lock = threading.Lock()

    @property
    def is_usb_capable(self):
        return _is_usb_capable(self.chip_model)

    @property
    def issues(self):
        return _compatibility_issues(self.protocol_version, self.chip_model)

    @property
    def supports_rumble(self):
        return min(self.protocol_version, HIGHEST_KNOWN_PROTOCOL_VERSION) >= 2

    @property
    def is_responsive(self):
        if self.last_received_monotonic is None:
            return False
        return (time.monotonic() - self.last_received_monotonic) < RELAY_RESPONSIVE_TIMEOUT_S

    def to_dict(self):
        return {
            "mac": self.mac,
            "port": self.port_name,
            "chipModel": self.chip_model,
            "protocolVersion": self.protocol_version,
            "isUsbCapable": self.is_usb_capable,
            "supportsRumble": self.supports_rumble,
            "issues": self.issues,
            "assignedPadId": self.pad_id,
            "responsive": self.is_responsive,
            "packetsSent": self.packets_sent,
        }

    def open(self):
        self.ser = serial.Serial(self.port_name, BAUD_RATE, timeout=0, write_timeout=0.05)
        self._stop.clear()
        self.thread = threading.Thread(target=self._send_loop, daemon=True)
        self.thread.start()
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def close(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def set_pad_id(self, pad_id):
        with self._lock:
            self.pad_id = pad_id

    def _send_loop(self):
        """250Hz packet sender — reads the freshest cached state for whatever
        pad_id is currently assigned right before each send, mirroring
        SerialLink.OnTick's "poll happens here rather than in a separate loop"
        design (see EverLink-main/Host/SerialLink.cs)."""
        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                pad_id = self.pad_id
            if pad_id is not None and self.ser and self.ser.is_open:
                state = _latest_gamepad_state.get(pad_id)
                if state is not None:
                    buttons = _js_buttons_to_everlink(state.get("buttons", 0))
                    lt = state.get("lt", 0)
                    rt = state.get("rt", 0)
                    lx = state.get("lx", 0)
                    ly = state.get("ly", 0)
                    rx = state.get("rx", 0)
                    ry = state.get("ry", 0)
                    packet = encode_packet(buttons, lt, rt, lx, ly, rx, ry)
                    try:
                        self.ser.write(packet)
                        self.packets_sent += 1
                    except Exception as e:
                        _error(f"Write failed on {self.port_name} ({self.mac}): {e}", "RELAY_WRITE_ERROR")

                self._rearm_rumble_if_due()

            next_tick += SEND_INTERVAL_S
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Fell behind (system load) — resync rather than free-running catch-up sends.
                next_tick = time.monotonic()

    def _rearm_rumble_if_due(self):
        """Nearcade has no local physical controller to keep buzzing (unlike
        the original Host — see this file's module docstring), so there is no
        keep-alive to re-arm here. Kept as a no-op hook (rather than removed
        outright) so the RMBL: receive path and this comment stay next to each
        other for anyone extending this later — see _read_loop's rumble
        handling for where incoming RMBL: lines actually go."""
        pass

    def _read_loop(self):
        """Reads whatever Relay sends back (debug lines, RMBL: lines) and
        parses the one line type this backend cares about. Any received line
        at all counts as a liveness signal (LastReceivedFromRelay), matching
        SerialLink.ProcessCompleteLine's behavior in the original Host."""
        buf = b""
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.05)
                    continue
                n = self.ser.in_waiting
                if n:
                    chunk = self.ser.read(n)
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip(b"\r")
                        if line:
                            self._handle_line(line)
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.05)

    def _handle_line(self, line: bytes):
        self.last_received_monotonic = time.monotonic()
        try:
            text = line.decode("ascii", errors="ignore")
        except Exception:
            return

        if text.startswith("RMBL:"):
            self._handle_rumble_line(text[len("RMBL:"):])

    def _handle_rumble_line(self, payload: str):
        """Parses "<left>:<right>" (each 0-255) and forwards it up to Node as
        a 'rumble' event keyed by whichever pad_id is currently assigned —
        server.js already knows how to route a 'rumble' event back to the
        owning viewer (see this file's module docstring)."""
        parts = payload.split(":", 1)
        if len(parts) != 2:
            return
        try:
            left = max(0, min(255, int(parts[0])))
            right = max(0, min(255, int(parts[1])))
        except ValueError:
            return

        self._rumble_left = left
        self._rumble_right = right

        with self._lock:
            pad_id = self.pad_id
        if pad_id is None:
            return

        # Scale 0-255 up to SDL/HTML5-Gamepad-style 16-bit motor strength, the
        # same 257x scale EverLink's own Host used going the other direction
        # (see SdlControllerReader.Rumble in EverLink-main/Host/SdlController.cs).
        _emit({
            "type": "rumble",
            "viewerId": pad_id.split("_")[0],
            "strong": min(65535, left * 257),
            "weak": min(65535, right * 257),
            "duration": 200,
        })


# mac -> Relay
_relays = {}
_relays_lock = threading.Lock()

# pad_id -> latest {buttons, lt, rt, lx, ly, rx, ry} — a plain cache, read by
# each Relay's own send loop. lt/rt arrive from InputOrchestrator as 0..1
# floats (see _validateGamepadMsg's _clampTrigger) and axes as -32767..32767
# ints; converted to EverLink's 0-255 trigger scale here, once, on write,
# rather than in every Relay's hot send loop.
_latest_gamepad_state = {}


def _try_ping_device(port_name):
    """Opens the given port, sends the identification ping, and returns
    (protocol_version, mac, chip_model) if a genuine Relay replies within the
    timeout, or None otherwise. Mirrors TryPingDevice in
    EverLink-main/Host/SerialLink.cs exactly, including tolerating a v1
    Relay's reply shape (IdentPrefix already stops right before the version
    digit so both v1 and v2+ replies parse the same way)."""
    try:
        probe = serial.Serial(port_name, BAUD_RATE, timeout=PING_TIMEOUT_S, write_timeout=PING_TIMEOUT_S)
    except Exception:
        return None

    try:
        try:
            probe.reset_input_buffer()
        except Exception:
            pass

        probe.write(PING_BYTE)

        deadline = time.monotonic() + PING_TIMEOUT_S
        line_bytes = b""
        while time.monotonic() < deadline:
            chunk = probe.read(1)
            if not chunk:
                continue
            if chunk == b"\n":
                break
            line_bytes += chunk
        else:
            return None

        line = line_bytes.decode("ascii", errors="ignore").strip("\r\n")
        if not line.startswith(IDENT_PREFIX):
            return None

        rest = line[len(IDENT_PREFIX):]
        parts = rest.split(":", 2)
        if len(parts) < 2:
            return None
        try:
            version = int(parts[0])
        except ValueError:
            return None

        mac = parts[1]
        chip_model = parts[2] if len(parts) > 2 else "Unknown"
        return (version, mac, chip_model)
    except Exception:
        return None
    finally:
        try:
            probe.close()
        except Exception:
            pass


def _scan_available_port_names():
    """Cross-platform port enumeration via pyserial's list_ports — this is
    the piece that made the original Host Windows-only (it used WMI's
    Win32_PnPEntity, see ScanAvailablePortsWithNames in
    EverLink-main/Host/SerialLink.cs); pyserial's list_ports works the same
    way on Windows (COMx), Linux (/dev/ttyUSB*, /dev/ttyACM*), and macOS
    (/dev/cu.*), which is the whole reason this backend exists as a Python
    sidecar rather than a Node addon."""
    try:
        return [p.device for p in list_ports.comports()]
    except Exception as e:
        _error(f"Port enumeration failed: {e}", "PORT_SCAN_ERROR")
        return []


def _emit_relay_list():
    with _relays_lock:
        relays = [r.to_dict() for r in _relays.values()]
    _emit({"type": "relay_list", "relays": relays})


def _do_scan():
    """Probes every currently-visible port not already claimed by an open
    Relay connection, adds any newly-identified Relay, and emits a fresh
    relay_list. Mirrors ScanForEverLinkRelays's "skip already-open ports"
    behavior (EverLink-main/Host/SerialLink.cs) — an already-confirmed Relay
    doesn't need re-probing, and probing a port some other process has open
    isn't safe to attempt anyway."""
    with _relays_lock:
        already_open_ports = {r.port_name for r in _relays.values()}
        already_known_macs = set(_relays.keys())

    candidate_ports = [p for p in _scan_available_port_names() if p not in already_open_ports]

    newly_found = []
    for port_name in candidate_ports:
        identity = _try_ping_device(port_name)
        if identity is None:
            continue
        version, mac, chip_model = identity
        if mac in already_known_macs:
            # Same board reappeared on a different port than we have on
            # record (e.g. moved to a different USB socket) — MAC-keyed
            # identity means we don't want a duplicate entry; just note it,
            # a full re-add-on-move isn't handled automatically since the
            # old port's connection may still be live elsewhere.
            continue
        newly_found.append((mac, port_name, chip_model, version))

    for mac, port_name, chip_model, version in newly_found:
        relay = Relay(mac, port_name, chip_model, version)
        try:
            relay.open()
        except Exception as e:
            _error(f"Failed to open {port_name} for Relay {mac}: {e}", "RELAY_OPEN_ERROR")
            continue
        with _relays_lock:
            _relays[mac] = relay
        _log(f"EverLink Relay found: {mac} on {port_name} ({chip_model}, protocol v{version})")
        _emit({"type": "relay_added", "relay": relay.to_dict()})

    _emit_relay_list()


def _do_assign(mac, pad_id):
    with _relays_lock:
        relay = _relays.get(mac)
    if relay is None:
        _error(f"No known Relay with MAC {mac}", "RELAY_NOT_FOUND")
        return
    relay.set_pad_id(pad_id if pad_id else None)
    _log(f"Relay {mac} assigned to {pad_id if pad_id else '(none)'}")
    _emit_relay_list()


def _do_forget(mac):
    with _relays_lock:
        relay = _relays.pop(mac, None)
    if relay is None:
        return
    relay.close()
    _log(f"EverLink Relay forgotten: {mac}")
    _emit({"type": "relay_removed", "mac": mac})
    _emit_relay_list()


def _do_disconnect_viewer(viewer_id):
    """Clears any Relay assignment(s) pointing at a pad_id under this viewer
    (pad_id == viewer_id, or "<viewer_id>_N" for a multi-controller viewer),
    without removing the Relay itself — see this file's module docstring."""
    if not viewer_id:
        return
    with _relays_lock:
        affected = [
            r for r in _relays.values()
            if r.pad_id == viewer_id or (r.pad_id or "").startswith(viewer_id + "_")
        ]
    for r in affected:
        r.set_pad_id(None)
    _latest_gamepad_state.pop(viewer_id, None)
    if affected:
        _emit_relay_list()


def _handle_gamepad(msg):
    pad_id = str(msg.get("pad_id") or msg.get("viewerId") or "")
    if not pad_id:
        return

    lt_float = msg.get("lt", 0) or 0
    rt_float = msg.get("rt", 0) or 0
    _latest_gamepad_state[pad_id] = {
        "buttons": int(msg.get("buttons", 0) or 0),
        "lt": max(0, min(255, round(float(lt_float) * 255))),
        "rt": max(0, min(255, round(float(rt_float) * 255))),
        "lx": int(msg.get("lx", 0) or 0),
        "ly": int(msg.get("ly", 0) or 0),
        "rx": int(msg.get("rx", 0) or 0),
        "ry": int(msg.get("ry", 0) or 0),
    }


def _destroy_all():
    with _relays_lock:
        relays = list(_relays.values())
        _relays.clear()
    for r in relays:
        r.close()
    _latest_gamepad_state.clear()
    _log("EverLink backend: all Relays closed.")


def _process(msg: dict):
    msg_type = msg.get("type", "")

    if msg_type == "gamepad":
        _handle_gamepad(msg)
        return
    if msg_type == "everlink_scan":
        _do_scan()
        return
    if msg_type == "everlink_list":
        _emit_relay_list()
        return
    if msg_type == "everlink_assign":
        _do_assign(str(msg.get("mac", "")), msg.get("pad_id"))
        return
    if msg_type == "everlink_forget":
        _do_forget(str(msg.get("mac", "")))
        return
    if msg_type == "disconnect_viewer":
        _do_disconnect_viewer(str(msg.get("viewer_id") or msg.get("viewerId") or ""))
        return
    if msg_type == "destroy_all":
        _destroy_all()
        return
    # Unrecognized message types are silently ignored — this backend only
    # cares about the subset listed in the module docstring; everything else
    # (kbm, window-focus, etc.) belongs to the other backends and is routed
    # there by InputOrchestrator.js, not duplicated here.


def main():
    _emit({"type": "ready", "message": "EverLink backend ready."})
    stdin_raw = open(sys.stdin.fileno(), "rb", buffering=0)
    for raw_line in stdin_raw:
        line = raw_line.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        try:
            _process(msg)
        except Exception as e:
            _error(f"Unexpected error processing message: {e}", "PROCESS_ERROR")

    _destroy_all()


if __name__ == "__main__":
    main()
