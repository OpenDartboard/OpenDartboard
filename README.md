# OpenDartboard 🎯

**OpenDartboard** is a hobby-friendly, fully FOSS toolkit for building an automatic _steel-tip_ dart–scoring station:

- **Headless Scorer** — Uses a Raspberry Pi Zero 2 W with 3 cameras to detect darts and output scores in standard notation (T20, S5, D12, …) via a easy to consume WebSocket API.
- **Computer Vision** — Lightweight OpenCV to run efficiently on a Pi Zero 2 W.

The goal: **drop-in freedom** for home tinkerers who want “Automatic darts scoring" without closed hardware, subscriptions, or vendor lock-in.

---

## 1 Project Status & Short-term Goals

| Phase           | Target                                             | ETA      |
| --------------- | -------------------------------------------------- | -------- |
| **Packaging**   | Debian package (.deb) for easy installation        | ✅ works |
| **Development** | Docker-based dev environment for consistent builds | ✅ works |
| **MVP**         | Auto-calibration via OpenCV                        | ✅ works |
| **MMR**         | Scoring vai OpenCV (or) YOLOv8-nano /w NCNN        | WIP      |
| **Polish**      | Improved accuracy & Auto-calibration and CI/CD     | T.B.D    |
| **Stretch**     | WebSocket API / HDMI ASCII output                  | T.B.D    |

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

| Item         | Minimum spec                                       | Example                       |
| ------------ | -------------------------------------------------- | ----------------------------- |
| SBC          | Raspberry Pi Zero 2 W (+ self-powered USB 2 hub)   | Waveshare USB HUB HAT (B)     |
| Cameras (×3) | USB 2.0 webcams outputting MJPEG @ 1280x720 30 fps | HBVCAM OV2710 100°            |
| Lighting     | 360° LED ring                                      | DIY SmartLite 12 V LED 6000 K |
| Power        | 5 V / 3 A PSU                                      | Any USB-C PD brick + adapter  |

---

## 4 Quick Start

```shell
# 0 Flash Raspberry Pi OS Lite (64-bit) via Rasbperi Pi Imager
# 0.1 - set hostname to opendartboard
# 0.2 - enable SSH
# 0.3 - setup Wi-Fi SSID and password
# 0.4 boot and connect to pi via SSH:
ssh pi@opendartboard.local

# 1 Build the package & On the Pi: install from package
git clone https://github.com/OpenDartboard/OpenDartboard.git
./install.sh
./make.sh
sudo apt install ./opendartboard_0.x.0.deb

# 2 Watch the scores
sudo journalctl -fu opendartboard.service
# Will print: D20 / S5 / MISS / END etc.
```

## 5. Development

```sh
# Clone repository
git clone https://github.com/OpenDartboard/OpenDartboard.git
cd OpenDartboard

# run build.deb.sh ONCE to get everything setup
./build.deb.sh

# login into you build
docker run -it --rm -v $PWD:/app opendartboard:latest /bin/bash

# make changes
# rebuild & rerun (can use mock images)
cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local && cmake --build build -j4 && cp build/opendartboard /usr/local/bin/opendartboard && opendartboard --debug --cams mocks/cam_1.mp4,mocks/cam_2.mp4,mocks/cam_3.mp4 --width 1270 --height 720
```

## 6. Roadmap (1.0)

- Camera auto-calibration
- Tip detection & scorring
- WebSocket API & ASCII HDMI output
- Configurations/options

## 7. Contributing

Pull requests are welcome!
Please file an issue first — we’ll discuss approach & design.
