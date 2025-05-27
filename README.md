# OpenDartboard 🎯

**OpenDartboard** is a hobby-friendly, fully FOSS toolkit for building an automatic *steel-tip* dart–scoring station:

* **Relay** — a super-lean script that turns a Raspberry Pi Zero 2 W into a 3-camera MJPEG broadcaster (no heavy CV on the Pi).
* **Detection / UI** — a cross-platform Python + OpenCV service (laptop / phone)  that ingests those streams, triangulates dart tips, and outputs scores in standard notation (`T20`, `S5`, `D12`, …).

The goal: **drop-in freedom** for home tinkerers who want “Autodarts-style” scoring without closed hardware, subscriptions, or vendor lock-in.

---

## 1  Project Status & Short-term Goals

| Phase | Target | ETA |
|-------|--------|-----|
| **MVP-Relay** | Pi Zero 2 W script → three 720 p/10 fps MJPEG streams | ✅ works |
| **MVP-Scoring** | Pull streams, click-to-calibrate, output JSON & on-screen score | _In progress_ |
| **Polish** | Headless auto-calibration wizard + React UI overlay | T.B.D |
| **Stretch** | Stats DB, cloud sync, fancy LED ring integration | T.B.D |

---

## 2  Tech Stack

| Layer | Tech | Notes |
|-------|------|-------|
| **Relay** | Bash setup script · [`uStreamer`](https://github.com/pikvm/ustreamer) | Pass-through MJPEG, <10 % CPU on Pi Zero 2 W |
| **Computer Vision** | `Python 3.11` · `OpenCV 4` · NumPy | Fork of [Zoofly85/automatic-darts-](https://github.com/Zoofly85/automatic-darts-) with URL inputs |
| **Frontend (optional)** | React / Vite · TypeScript · WebSockets | Lightweight overlay + remote control |
| **Tooling** | `poetry` / `pipx` · `pre-commit` · GitHub Actions | CI checks, auto-formatting |

---

## 3  Hardware Reference

| Item | Minimum spec | Example |
|------|--------------|---------|
| SBC | Raspberry Pi Zero 2 W (+ self-powered USB 2 hub) | Waveshare USB HUB HAT (B) |
| Cameras (×3) | USB 2.0 webcams outputting **MJPEG** @ 720 p 10 fps | HBVCAM OV2710 100° |
| Lighting | 360° LED ring | DIY SmartLite 12 V LED 6000 K |
| Optional | 5 V / 3 A PSU | Any USB-C PD brick + adapter |

---

## 4  Quick Start

```shell
# 0  Flash Raspberry Pi OS Lite (64-bit) & enable SSH/Wi-Fi in Imager.
ssh pi@dart-relay.local

# 1  On the Pi: install relay
curl -sSL https://raw.githubusercontent.com/OpenDartboard/installer/main/install_dart_streamer.sh | bash
# Test you streams now at 
## http://dart-relay.local:8081/stream 
## http://dart-relay.local:8082/stream
## http://dart-relay.local:8083/stream

# 2 Navigate to opendartboard.com and connect.
## Follow on-screen calibration clicks, then throw darts!

# 2.1 (optional) for a completly local offline setup:
git clone https://github.com/OpenDartboard/opendartboard.git
cd opendartboard/ui
poetry install
python main.py --urls \
  http://dart-relay.local:8081/stream \
  http://dart-relay.local:8082/stream \
  http://dart-relay.local:8083/stream
```

## 5 Roadmap

- Better tip detection (replace edge-based guess with lightweight YOLOv8-n).
- Sleek React Native remote UI (show score history, checkout suggestions).
- Cloud sync + friend leaderboards (optional, privacy-first switch).
- many more...


## 6 Contributing
Pull requests are welcome!
Please file an issue first if you plan major changes — we’ll discuss approach & design.

##  7 License
MIT License
