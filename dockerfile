# Use this line for building for ARM architecture (e.g zero 2)
FROM arm64v8/debian:bullseye

ENV DEBIAN_FRONTEND=noninteractive

COPY apt-packages.txt /tmp/
RUN apt-get update -y && xargs apt-get install -y < /tmp/apt-packages.txt

WORKDIR /app
COPY . .
