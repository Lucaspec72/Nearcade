# EverLink Relay Integration — Design Notes

**Status:** Draft · applies to Nearcade 3.0.x
**Scope:** Host-side only. This is an additional controller *output* path, not
part of the Viewer-facing protocol in `docs/protocol.md`.

---

## 1. What this is

[EverLink](https://github.com/Lucaspec72/EverLink) is a small open-source
project (MIT licensed) with two halves:

- **Relay** — ESP32 firmware that turns a board into a real USB HID Xbox 360
  gamepad, driven by 14-byte packets arriving over USB-serial.
- **Host** — a Windows-only WPF/C# app that reads a local physical controller
  (via SDL) and a local controller-remap UI, then streams its state to one or
  more paired Relay boards.

---

## 2. Why this needed to be reimplemented rather than reused

The original Host app is Windows-only for exactly one reason: port
enumeration (`ScanAvailablePortsWithNames` in `Host/SerialLink.cs`) uses WMI's
`Win32_PnPEntity` to get human-friendly port names. Everything else — the
identification ping, the packet protocol, the rumble line format — is plain
serial I/O with a cross-platform equivalent on every OS Nearcade already
supports.

Rather than bundle the C#/WPF Host binary as a sidecar (Windows-only, and
architecturally mismatched — it expects to read a *local* SDL controller,
not a normalized viewer input stream), the protocol was reimplemented from
`EverLink_Protocol.md` as a small OS-agnostic Python sidecar
(`src/sidecar/input_backends/everlink_backend.py`), using `pyserial` for both
port enumeration and serial I/O — both of which work identically on Windows,
Linux, and macOS. This follows the same "one Python sidecar per device-type,
JSON-lines over stdin" convention already used by
`input_backends/experimental/ExperimentalOrchestrator.js` and its own child
sidecars.

**Attribution:** the wire protocol itself (packet layout, identification
handshake, rumble line format) is EverLink's design, documented in its own
`EverLink_Protocol.md` (MIT license). This integration is a clean-room
reimplementation of that protocol against Nearcade's existing input pipeline.

---

## 3. Architecture

```
 Viewer (browser/desktop)                 Nearcade Host process
 ┌──────────────────┐   WebRTC/WS   ┌────────────────────────────────────────┐
 │ gamepad input     │ ─────────────► server.js                              │
 └──────────────────┘               │   │                                    │
                                     │   ▼                                    │
                                     │ InputOrchestrator.js                   │
                                     │   ├─► uinput / ViGEm / HIDMaestro       │
                                     │   │   (existing local virtual pad)      │
                                     │   └─► everlink_backend.py (new)         │
                                     │         │  JSON-lines over stdin/stdout │
                                     │         ▼                               │
                                     │       pyserial ── USB ──► EverLink Relay│
                                     │                            (ESP32) ──► USB HID
                                     │                            gamepad seen by
                                     │                            console/capture
                                     └────────────────────────────────────────┘
```

`everlink_backend.py` is spawned lazily — only on first `everlink-*` action
from the host UI — so hosts with no Relay hardware pay zero cost for this
feature. It runs independently of, and alongside, whichever OS backend is
already emulating input locally; a Relay is an *additional* output, not a
replacement.

---

## 4. Wire protocol (Host process ↔ Relay board)

Unchanged from EverLink's own spec — summarized here for reference; see
`everlink_backend.py`'s module docstring for the authoritative in-repo copy.

- **Identification:** write `0xFE`; a genuine Relay replies within ~300ms
  with `IAM:EverLink:v<N>:<MAC>:<ChipModel>\n`. No reply, or a malformed one,
  means the port isn't a Relay.
- **Controller packet** (14 bytes, little-endian, sent at 250Hz / every 4ms):

  | Offset | Field                | Type      |
  |--------|----------------------|-----------|
  | 0      | Sync byte            | `0xA5`    |
  | 1–2    | Buttons              | `uint16`  |
  | 3      | Left trigger         | `uint8`   |
  | 4      | Right trigger        | `uint8`   |
  | 5–6    | Left stick X         | `int16`   |
  | 7–8    | Left stick Y         | `int16`   |
  | 9–10   | Right stick X        | `int16`   |
  | 11–12  | Right stick Y        | `int16`   |
  | 13     | Checksum (XOR 1–12)  | `uint8`   |

- **Rumble** (Relay → Host, protocol v2+ only): a text line
  `RMBL:<left>:<right>\n`, each 0–255, sent on change.

### Button bit remap

Nearcade's own viewer→JS gamepad bitmask (`viewer.js`) and EverLink's wire
bitmask are **not** the same layout — only the shoulder buttons happen to
share a bit position. `everlink_backend.py`'s `_js_buttons_to_everlink()`
does an explicit per-bit translation table; see that function's comment
block for the full mapping (verified against `viewer.js`'s `btnMask`
assignment).

---

## 5. JSON-lines protocol (Node ↔ Python sidecar)

Mirrors the shape every other backend in `input_backends/` already uses.

**Sidecar → Node** (one JSON object per stdout line):

| `type`           | Payload                                                    |
|------------------|-------------------------------------------------------------|
| `ready`          | `{message}` — sidecar started successfully                 |
| `log`            | `{message}` — informational                                |
| `error`          | `{message, code}`                                           |
| `relay_list`     | `{relays: [...]}` — full current set, always a full replace |
| `relay_added`    | `{relay: {...}}`                                             |
| `relay_removed`  | `{mac}`                                                       |
| `rumble`         | `{viewerId, strong, weak, duration}` — routed through the same `inputDriver.events.on('rumble', ...)` listener server.js already has for the C++ bridge path |

Each relay object: `{mac, port, chipModel, protocolVersion, isUsbCapable,
supportsRumble, issues: [string], assignedPadId, responsive, packetsSent}`.

**Node → sidecar:**

| `type`              | Payload                          | Effect                                                |
|---------------------|-----------------------------------|--------------------------------------------------------|
| `everlink_scan`     | —                                  | Probe unclaimed serial ports, add any new Relays        |
| `everlink_list`     | —                                  | Re-emit `relay_list` without scanning                    |
| `everlink_assign`   | `{mac, pad_id}`                    | Assign (or clear, `pad_id: null`) a Relay's controller     |
| `everlink_forget`   | `{mac}`                            | Close and forget a Relay                                    |
| `gamepad`           | (standard normalized gamepad msg) | Cached per `pad_id`; each Relay's 250Hz send loop reads whichever `pad_id` it's currently assigned |
| `disconnect_viewer` | `{viewer_id}`                      | Clears assignment(s) pointing at that viewer's pad_id(s), without removing the Relay itself |
| `destroy_all`       | —                                  | Closes every open Relay and exits cleanly                    |

`pad_id` is the same key used everywhere else in `InputOrchestrator.js`
(`viewerId`, or `viewerId_N` for a multi-controller viewer) — **not** a slot
index — so an assignment survives a slot renumber the same way other
viewer-keyed state in this codebase does.

---

## 6. Host-side WS contract (browser ↔ `server.js`)

Host-only messages (sent on `/ws/host`; no Viewer-side equivalent exists,
since only the machine physically wired to Relay hardware can use these):

| Client → Server        | Payload             |
|-------------------------|----------------------|
| `everlink-scan`         | —                    |
| `everlink-list`         | —                    |
| `everlink-assign`       | `{mac, padId}`        |
| `everlink-forget`       | `{mac}`                |

| Server → Client (host)     | Payload                          |
|------------------------------|------------------------------------|
| `everlink-relay-list`        | `{relays: [...]}`                  |
| `everlink-relay-added`       | `{relay: {...}}`                    |
| `everlink-relay-removed`     | `{mac}`                              |
| `everlink-error`             | `{message, code}`                    |

`server.js` does no translation on these beyond forwarding — the relay
objects it pushes down are exactly what the sidecar produced.

---

## 7. Host UI

Settings → Input tab → **EverLink Relays** button opens a dedicated modal
(`#everlinkModal` in `src/pages/host-modals.html`, logic in
`src/scripts/host.js`):

- **Scan for Relays** — probes unclaimed serial ports; soft 2.5s UI timeout
  (there's no single discrete "scan complete" server reply, since
  `relay_list` can also arrive from unrelated pushes).
- One card per known Relay: chip model, MAC, port, protocol version, a
  status dot (🟢 responsive / 🟡 stale / ⚠️ not USB-capable), any
  compatibility warnings (e.g. a v1 board has no rumble; a plain ESP32/C3 has
  no USB OTG and can't act as a wired bridge at all), and a controller
  dropdown populated from the existing viewer roster (`window._rosterData`)
  — reusing the exact same `pad_id`/name pairs the rest of the host UI
  already renders, so relay assignment and viewer identity never drift out
  of sync.
- Forget (🗑) closes and drops a specific Relay from the list.

The roster refresh (`roster` WS message) also re-renders the EverLink panel's
dropdowns, so a viewer joining/leaving keeps the assignment options current
without a manual rescan.
