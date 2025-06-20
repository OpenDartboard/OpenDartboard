# OpenDartboard 🎯

**OpenDartboard** is a hobby-friendly, fully FOSS toolkit for building an automatic _steel-tip_ dart–scoring station:

- **Headless Scorer** — Uses a Raspberry Pi Zero 2 W with 3 cameras to detect darts and output scores in standard notation (T20, S5, D12, …) via a easy to consume WebSocket API.
- **Computer Vision** — Lightweight OpenCV, YOLOv8-nano model with ncnn acceleration to run efficiently on a Pi Zero 2 W..

The goal: **drop-in freedom** for home tinkerers who want “Autodarts-style” scoring without closed hardware, subscriptions, or vendor lock-in.

---

## 1 Project Status & Short-term Goals

| Phase           | Target                                             | ETA      |
| --------------- | -------------------------------------------------- | -------- |
| **Packaging**   | Debian package (.deb) for easy installation        | ✅ works |
| **Development** | Docker-based dev environment for consistent builds | ✅ works |
| **MVP**         | Zero 2 W with YOLOv8-nano scoring                  | WIP      |
| **Polish**      | Auto-calibration and improved accuracy             | T.B.D    |
| **Stretch**     | Stats DB, cloud sync                               | T.B.D    |

---

## 2 Tech Stack

| Layer               | Tech                              | Notes                                     |
| ------------------- | --------------------------------- | ----------------------------------------- |
| **Computer Vision** | `YOLOv8-nano` · `ncnn` · `OpenCV` | Optimized INT8 model for arm32            |
| **Runtime**         | C++                               | High-performance dart detection           |
| **Infrastructure**  | `systemd` services · `udev` rules | Reliable auto-start and camera management |
| **Development**     | `Docker` · `Debian Bullseye`      | Reproducible build environment            |
| **Distribution**    | `.deb` package                    | CI checks, One-command installation       |

---

## 3 Hardware Reference

| Item         | Minimum spec                                      | Example                       |
| ------------ | ------------------------------------------------- | ----------------------------- |
| SBC          | Raspberry Pi Zero 2 W (+ self-powered USB 2 hub)  | Waveshare USB HUB HAT (B)     |
| Cameras (×3) | USB 2.0 webcams outputting MJPEG @ 640x480 15 fps | HBVCAM OV2710 100°            |
| Lighting     | 360° LED ring                                     | DIY SmartLite 12 V LED 6000 K |
| Optional     | 5 V / 3 A PSU                                     | Any USB-C PD brick + adapter  |

---

## 4 Quick Start

```shell
# 0 Flash Raspberry Pi OS Lite (64-bit) & enable SSH/Wi-Fi in Rasbperi Pi Imager.
ssh pi@raspberrypi.local

# 1 On the Pi: install from package
curl -sSL https://github.com/OpenDartboard/OpenDartboard/releases/latest/download/opendartboard_0.1.0-1_armhf.deb -o opendartboard.deb
sudo dpkg -i opendartboard.deb

# 2 Watch the scores
sudo journalctl -fu opendartboard.service
# Will print: D20 / S5 / MISS / END etc.
```

## 5. Development

```sh
# Clone repository
git clone https://github.com/OpenDartboard/OpenDartboard.git
cd OpenDartboard

# Build with Docker (reproducible environment)
docker-compose build
docker-compose up -d

# Build .deb package
./make.sh
```

## 6. Roadmap (MVP 1.0)

- Camera auto-calibration
- Tip detection
- WebSocket api
- ASCII HDMI output
- Configurations/options

## 7. Contributing

Pull requests are welcome!
Please file an issue first if you plan major changes — we’ll discuss approach & design.

## 8. License

MIT License
