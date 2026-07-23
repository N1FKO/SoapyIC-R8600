# SoapyIC-R8600

SoapySDR driver for the IC-R8600 USB I/Q output on macOS.


Greetings, programs!

STOP.  This does not work.

Just kidding.  This project was inspired by, and depends on, Pieter Ibelings' excellent detective work
in his ic-r8600-usb-iq repo.  You must go read and understand that now.  I'll wait...


OK.


This repo contains a SoapySDR module and its Swift
core. It does not include firmware or third-party material.

This project was written and tested against a MacPorts install
of SoapySDR, and it installs into its tree* (usually `/opt/local`).

It supports the radio's full rate (5.120 MS/s, 20 MB/sec) using gqrx, SDRangel, and SDR++.


## Requirements

- macOS 13 or newer
- Swift 5.9 or newer
- CMake 3.16 or newer
- SoapySDR development files discoverable by CMake
- Firmware.  (you did go read Pieter's repo, yes?)
- IC-R8600 firmware must be version 1.3 or later

## Build

Build the Swift core first, then the SoapySDR module:

```
cd SwiftCore
swift build -c release

cd ..
cmake -S . -B build -DSWIFT_BUILD_CONFIG=release
cmake --build build
sudo cmake --install build
```

By default, `CMakeLists.txt` detects the SoapySDR install location found by
`find_package(SoapySDR)` and installs into that same prefix (MacPorts defaults
to `/opt/local`, so the module lands in `/opt/local/lib/SoapySDR/modules0.8/`).

## Verify

```
SoapySDRUtil --find='driver=icr8600'
SoapySDRUtil --probe='driver=icr8600,firmware=/path/to/spt_seq.json'
```

If you don't pass `firmware=` explicitly, the driver looks for the firmware
file at `<install-prefix>/share/SoapyIC-R8600/spt_seq.json` (where
`<install-prefix>` is the `CMAKE_INSTALL_PREFIX` used at build time, e.g.
`/opt/local`).

## License

This repository is released under the MIT License. See `LICENSE`.

\* Yes, I know.
