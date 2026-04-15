# rds-control

CLI tool for controlling RDS encoders (MRDS1322 + PIRA32).

## Build

make

Requires:
sudo apt install build-essential libcurl4-openssl-dev

## Example

./rds_control --encoder mrds1322 --bus 0 --pi C203 --ps RDS --pty 10

## Usage

MRDS1322 (I2C)
./rds_control --encoder mrds1322 --bus 0 --pi C203 --ps RDS --pty 10

PIRA32 (serial)
./rds_control --encoder pira32 --device /dev/ttyUSB0 --pi C203 --ps RDS

## Dynamic Radiotext from URL

./rds_control \
  --encoder pira32 \
  --device /dev/ttyUSB0 \
  --rt-url http://127.0.0.1/rt.txt \
  --interval 5

## Daemon mode

./rds_control \
  --encoder mrds1322 \
  --bus 0 \
  --rt-url http://127.0.0.1/rt.txt \
  --interval 5 \
  --daemon \
  --pidfile /run/rds_control.pid \
  --logfile /var/log/rds_control.log

