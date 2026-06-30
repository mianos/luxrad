# ldr3

ESP32-C3 ambient-light sensor on **ESP-IDF v6.0.1**.

An **APDS9960** ambient-light sensor (I2C0) reports lux over MQTT. Settings persist
in NVS and are adjustable over MQTT and HTTP. A web server exposes health, config and
over-the-air firmware update with rollback verification.

> A radar-equipped variant (LD1125 mmWave presence + lux) lives on the
> [`with-radar`](https://github.com/mianos/luxrad/tree/with-radar) branch.

## Components

Shared infrastructure is pulled from the [`mianesp`](https://github.com/mianos/mianesp)
repo (branch `idf6-network-provisioning`) and the Espressif registry; sensor drivers
keep their own repos; the MQTT and settings wrappers are vendored under `components/`.

| Component | Source |
|---|---|
| `jsonwrapper`, `nvsstoragemanager`, `wifimanager`, `webserver` | mianesp @ `idf6-network-provisioning` |
| `apds9960` | own git repo @ `main` |
| `espressif/cjson`, `espressif/mqtt`, `espressif/network_provisioning` | Espressif registry |
| `mqttwrapper`, `settings` | vendored in `components/` |

## Build

```
./build.sh          # exports IDF v6.0.1 and runs idf.py build
```

`build.sh` is a fixed wrapper (stable command for tooling). For a normal flow:

```
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32c3
idf.py build flash monitor
```

Wi-Fi is provisioned with ESP-Touch v2 on first boot. The dual-OTA partition table
(`partitions.csv`) and `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` give safe OTA: a freshly
flashed image is only marked valid once the device is back on the network, else it rolls
back to the previous slot.

> CI (`.gitlab-ci.yml`) and `idf_component.yml` reference the mianesp repo over SSH
> (`git@github.com:...`); the build host / runner needs a deploy key with read access.

## MQTT topics

Published (`tele/<name>/…`): `init`, `status` (uptime/heap), `lux`, `settingsack`.
Commands (`cmnd/<name>/…`): `settings`, `restart`, `reprovision`.

```
mosquitto_pub -h mqtt2.mianos.com -t cmnd/ldr/settings -m '{"luxPeriodSec":30}'
```

## HTTP

```
GET  /healthz        # version, partition, heap, lux
GET  /config         # current settings
POST /config         # apply a subset of settings (JSON body)
POST /config/reset   # restore defaults; {"wifi":true} also wipes credentials
GET  /firmware       # running image info
POST /firmware       # raw .bin -> inactive OTA slot -> reboot
```

```
curl --data-binary @build/ldr3.bin http://<host>/firmware
```
