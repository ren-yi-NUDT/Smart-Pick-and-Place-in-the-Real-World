# TCP protocol

All active robot services use one transport framing rule:

```text
[4-byte unsigned big-endian payload length][payload bytes]
```

JSON services put UTF-8 JSON in the payload. The AnyGrasp service uses the
same frame for a binary payload consisting of a JSON header, a newline, raw
depth bytes, and raw RGB bytes. The framing layer is implemented in
`core/tcp_protocol.py`; callers should use it instead of calling `sendall`
and `recv` directly.

Active endpoints:

| Service | Port(s) | Payload |
| --- | --- | --- |
| Arm bridge | 8010 / 8011 | JSON frame |
| Twin inference | 8020 / 8021 | JSON frame |
| PyBullet SimServer | 8031 | JSON frame |
| Real gripper bridge | 8001 / 8002 | JSON frame |
| AnyGrasp | 8030 | Binary frame with JSON header |

The Twin and SimServer detect the first message and continue accepting raw
JSON requests from older clients; their responses remain length-prefixed, as
older clients already expected. The gripper server detects the first message
and mirrors the old raw/raw contract for legacy clients. The arm bridge keeps
its historical framed request format and selects a framed response when
`_tcp_protocol: "json-frame-v1"` is present. This makes rolling upgrades
safe, but processes already running the old server code must be restarted
before new framed clients can use them.

The legacy Inspire-hand endpoint is intentionally not part of the active
Robotiq gripper path; it remains isolated as a legacy integration.
