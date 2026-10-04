# Connecting to the Control Hub for `mise run install`

`mise run install` runs `./gradlew :TeamCode:installDebug`, which installs the
**Robot Controller app** (`TeamCode`, namespace `org.firstinspires.ftc.teamcode`,
the only `com.android.application` module in this project — see
`settings.gradle`) onto whatever device `adb` currently sees. That app runs on
the **Control Hub**, not on the Driver Hub.

## The mistake this doc exists to prevent

**FTC Driver Station is a separate app on a separate device.** It isn't built
by this repo, isn't part of the Gradle project, and doesn't get touched by
`installDebug`. It just displays whatever OpMode list the Control Hub reports
when the Driver Station connects/reconnects to it.

If your Driver Hub is the only thing plugged into `adb` (`adb devices -l`
shows `model:Driver_Hub`), `installDebug` installs the Robot Controller app
onto the *Driver Hub* — the wrong device. The Control Hub never gets the
build, so Driver Station keeps showing stale OpMode names even though the
install "succeeded." Always confirm the target with `adb devices -l` before
trusting an install.

## Why this is hard

The Control Hub only operates as its own Wi-Fi access point (confirmed
against REV's docs: `docs.revrobotics.com/duo-control/managing-the-control-system/ch-wifi`
and the wireless-ADB page). **There is no "Station mode"** where it joins an
existing network — that was floated and ruled out during troubleshooting.
Reaching it wirelessly means your dev machine's network path has to include
a hop onto the Control Hub's own AP (`FTC-####` / `FIRST-####`, default IP
`192.168.43.1:5555`), which conflicts with staying on your normal
internet-connected Wi-Fi.

## Options, roughly cheapest/simplest to most robust

### 1. Direct USB-C tether (recommended default)
Plug a USB-A/USB-C → USB-C cable from your Mac straight into the Control
Hub. This is REV's own recommended method ("We recommend connecting via USB
to reduce the chance of disconnects") and needs zero network configuration —
`adb devices -l` sees it immediately, `mise run install` just works.

- Only usable while the robot is stationary (bench/pit), not while driving.
- If the port is awkward to reach once the Hub is mounted in the chassis,
  panel-mount a short USB-C extension cable (~$8) from the Hub's port to an
  accessible spot on the frame once — no disassembly needed after that.
- No new hardware required if the port is already reachable.

### 2. Second Wi-Fi radio on the dev machine, joined to the Control Hub's AP
Add a USB Wi-Fi adapter as its own macOS network service, join the Control
Hub's SSID on it, and keep the built-in Wi-Fi (your internet network) ranked
above it in Network Service Order:
```
sudo networksetup -ordernetworkservices "Wi-Fi" "<adapter service name>" ...
```
The built-in Wi-Fi keeps the default route (internet stays up); macOS adds a
directly-connected route for the Control Hub's subnet via the second
adapter automatically, so `adb connect 192.168.43.1:5555` reaches it without
touching the primary connection.

- Risk: third-party Wi-Fi chipsets need an Apple Silicon–compatible macOS
  System Extension driver (legacy kexts don't work on this platform).
  Verify the specific adapter model explicitly claims support before buying.

### 3. Client-bridge travel router
A travel router with **AP mode + WISP client** (not just "repeater/extender")
joins the Control Hub's SSID and bridges it to an Ethernet port with no NAT
— your Mac gets a real `192.168.43.x` DHCP lease straight from the Control
Hub. GL.iNet's cheaper models (Mango, Beryl, Slate) are the common choice in
the FTC/FRC community for this.

- **Failure mode we hit:** a router stuck in plain "repeater" mode NATs you
  into its own separate LAN instead of bridging, and without port
  forwarding for TCP 5555 you can't reach the Control Hub through it at
  all. Confirm true bridge/AP+WISP support before buying.
- Also watch macOS **Network Service Order**: if the router's Ethernet
  service outranks Wi-Fi, its DHCP-supplied default gateway wins and takes
  over all internet traffic. Reorder services (see option 2) so Wi-Fi stays
  primary; the router's own subnet stays reachable regardless of rank.

### 4. Onboard Raspberry Pi ADB bridge (most robust, most setup)
Mount a Raspberry Pi Zero 2 W (or similar Wi-Fi-capable SBC) on the robot,
permanently USB-tethered to the Control Hub, powered from the robot
battery. The Pi joins your *normal* internet Wi-Fi as an ordinary client —
your Mac never touches the Control Hub's own AP at all.

Setup:
1. Flash Raspberry Pi OS, join it to your normal Wi-Fi.
2. `sudo apt install android-sdk-platform-tools-common adb`
3. Plug the Pi into the Control Hub over USB; confirm `adb devices` on the
   Pi sees it.
4. Run `adb -a -P 5037 nodaemon server` (the `-a` flag binds the server to
   all interfaces, not just localhost) as a systemd service so it survives
   reboots.

On the Mac, before building (Gradle/ddmlib hardcodes `localhost:5037`, so
tunnel rather than relying on `ADB_SERVER_SOCKET` env-var redirection):
```bash
adb kill-server
ssh -fN -L 5037:localhost:5037 pi@ftc-bridge.local
mise run install
```

- Robot moves freely; the bridge travels with it.
- No dual-NIC juggling, no router shopping, internet never interrupted.
- Cost: ~$25 hardware + one-time Pi setup.

## Recommendation

Default to **Option 1** (direct tether, panel-mounted extension if needed)
for day-to-day iteration at the bench. Reach for **Option 4** only if the
team needs to push builds while the robot is untethered/in motion often
enough to justify the setup cost.
