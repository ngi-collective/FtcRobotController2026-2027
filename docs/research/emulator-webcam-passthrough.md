# Research: Does emulator webcam passthrough satisfy FTC camera discovery?

> Context: [ngi-collective/FtcRobotController2026-2027#13](https://github.com/ngi-collective/FtcRobotController2026-2027/issues/13), part of wayfinder map [#11](https://github.com/ngi-collective/FtcRobotController2026-2027/issues/11) ("FTC Test Framework: architecture spec").

## Bottom line

**No — hard incompatibility.** Android Studio emulator webcam passthrough (`-camera-back webcam<N>` / `hw.camera.back=webcam<N>`) surfaces the host webcam to the guest **only** through the Android camera-HAL layer (legacy `Camera` / `Camera2` `CameraManager`), routed over a paravirtual QEMU pipe (`/dev/qemu_pipe`, service `qemud:camera`). It never creates a real (or even fake-but-enumerable) USB device node in the guest, so it never appears in `android.hardware.usb.UsbManager.getDeviceList()`.

FTC's `WebcamName` discovery (the thing `VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, ...))` ultimately depends on) is built **entirely** on raw `UsbManager` USB-device enumeration plus a bundled native `libusb`/`libuvc` stack that talks directly to the raw USB device — it never calls `android.hardware.camera2.CameraManager` (grep of the full RobotCore SDK source tree for `android.hardware.camera2` returns zero matches). Since the emulator's passthrough webcam has no corresponding `UsbDevice`, `WebcamNameImpl.isAttached()` / `getUsbDeviceNameIfAttached()` will always return `false`/`null` for it, `hardwareMap.get(WebcamName.class, "Webcam 1")` will never resolve to a live device, and `VisionPortal.Builder().setCamera(...)` has nothing to open.

**Conclusion for the vision initiative:** a real, unmodified `VisionPortal`/`VisionProcessor` pipeline running against `WebcamName` cannot consume an emulator-passthrough webcam with zero custom code. A custom frame-relay (or a custom `CameraName`/`Camera` implementation that bypasses `WebcamName`/`UsbManager` entirely and talks to the emulator's Camera2 camera ID directly, feeding frames into a hand-built `VisionProcessor` harness) is required if development against a live camera feed inside the emulator is a goal.

## Evidence

### 1. FTC SDK source: `WebcamName` discovery is pure `UsbManager` enumeration + native libusb/libuvc

Source: `org.firstinspires.ftc:RobotCore:11.2.0` sources jar (resolved via `~/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/RobotCore/11.2.0/*/RobotCore-11.2.0-sources.jar`).

- `org/firstinspires/ftc/robotcore/external/hardware/camera/WebcamName.java` (interface): `getUsbDeviceNameIfAttached()` is documented `@see UsbManager#getDeviceList()` directly in the public API doc comment.

- `org/firstinspires/ftc/robotcore/internal/camera/names/WebcamNameImpl.java`:
  - Imports `android.hardware.usb.UsbDevice` and `android.hardware.usb.UsbManager` (no `android.hardware.camera2.*` import anywhere in the file).
  - `protected static final UsbManager usbManager = (UsbManager) AppUtil.getDefContext().getSystemService(Context.USB_SERVICE);`
  - `static String getUsbDeviceNameIfAttached(...)` (the method backing both `isAttached()` and `getUsbDeviceNameIfAttached()`) does:
    ```java
    for (UsbDevice usbDevice : getUsbManager().getDeviceList().values()) {
        SerialNumber candidate = cameraManagerInternal.getRealOrVendorProductSerialNumber(usbDevice);
        if (candidate != null && candidate.matches(serialNumberPattern)) { ... }
    }
    ```
    i.e. discovery is a linear scan of `UsbManager.getDeviceList()`, matched by a VID/PID-derived `SerialNumber` (`org.firstinspires.ftc.robotcore.internal.usb.VendorProductSerialNumber`). There is no fallback to `CameraManager.getCameraIdList()`.
  - Once a `UsbDevice` is matched, actual frame capture goes through `findUvcDevice()` -> `CameraManagerInternal.findUvcDevice(this)` -> native `org.firstinspires.ftc.robotcore.internal.camera.libuvc.nativeobject.UvcDevice` / `UvcStreamingInterface` objects — FTC's own bundled native libusb+libuvc (USB Video Class) implementation that talks to the raw USB device, not any Android camera HAL.

- `org/firstinspires/ftc/robotcore/internal/camera/CameraManagerImpl.java` (the singleton backing `ClassFactory.getInstance().getCameraManager()`):
  - Constructor registers a `BroadcastReceiver` for `UsbManager.ACTION_USB_DEVICE_ATTACHED` / `ACTION_USB_DEVICE_DETACHED` and constructs a native `UvcContext` (`this.uvcContext = new UvcContext(this, AppUtil.getInstance().getUsbFileSystemRoot())`) — driven by the *usbfs* root, i.e. the raw Linux USB filesystem (`/sys/bus/usb` / `/dev/bus/usb`), again bypassing any Android Camera HAL entirely.
  - `UsbAttachmentMonitor.onReceive()` pulls the `UsbDevice` out of the `UsbManager` intent extra (`UsbManager.EXTRA_DEVICE`) and computes `getRealOrVendorProductSerialNumber(usbDevice)` (VID/PID) before notifying any registered `WebcamNameImpl` — this is the live attach/detach path that `WebcamNameImpl.ArmableDeviceHelper.onAttached/onDetached` listens to.

- A full-text search of the entire `RobotCore-11.2.0-sources.jar` for `android.hardware.camera2` returns **zero matches**. `WebcamName`/`CameraManagerImpl` discovery never touches `android.hardware.camera2.CameraManager`.

### 2. Official Android docs: emulator webcam passthrough is a camera-HAL-level feature, not USB passthrough

- [Start the emulator from the command line](https://developer.android.com/studio/run/emulator-commandline) (`developer.android.com/studio/run/emulator-commandline`), "Device Hardware" table, `-camera-back *mode*` / `-camera-front *mode*` row: `webcam*n*` is listed as one mode value alongside `emulated`, `environment`, `imagefile:*filename*`, `videofile:*filename*`, `image360:*filename*`, and `none` — i.e. it is documented purely as a **camera emulation mode** (an input source for the emulator's virtual camera), structurally identical in the API surface to the pure-software `emulated`/`imagefile`/`videofile` modes. Nothing in this doc claims it creates a USB device.
- [External USB cameras](https://source.android.com/docs/core/camera/external-usb-cameras) (`source.android.com/docs/core/camera/external-usb-cameras`) describes how a **genuine physical** USB UVC webcam is exposed to apps on real hardware: "The Android platform supports the use of plug-and-play USB cameras ... using the standard Android Camera2 API and the camera HAL interface... The USB camera HAL process is part of the external camera provider that listens to USB device availability and enumerates external camera devices accordingly." This requires real kernel support (`CONFIG_USB_VIDEO_CLASS`, `CONFIG_MEDIA_USB_SUPPORT`), the `android.hardware.usb.host` feature, and SELinux rules granting `cameraserver` access to `video_device` nodes — i.e. a real USB bus, a real kernel UVC/V4L driver, and a real external-camera-provider HAL translating that into a Camera2 camera ID. This is the mechanism for genuine USB webcams on real Android hardware (e.g. a REV Control Hub); it is **not** how the AVD emulator implements webcam passthrough (the emulator has no real USB host controller wired to the host's webcam and does not run this external-camera-provider/UVC kernel path against the passthrough webcam).
- Android emulator camera HAL implementation (goldfish/ranchu, `device/generic/goldfish` in AOSP, `camera.goldfish.so`): the guest-side camera HAL (`EmulatedQemuCamera` / `CameraQemuClient`) communicates with the emulator process over `/dev/qemu_pipe` using a `qemud:camera` service — a paravirtualized QEMU pipe device, architecturally unrelated to the USB subsystem. On the host side (Linux), the emulator's camera service drives the physical webcam via V4L2 ioctls and streams frames back down the same pipe. At no point is a `/dev/bus/usb/*` node created in the guest, and no `UsbManager.ACTION_USB_DEVICE_ATTACHED` broadcast fires for the passthrough camera — it only ever surfaces as a Camera (legacy) / Camera2 camera ID (e.g. camera ID `"0"`/`"1"`) via `camera.goldfish.so`. (Modern `ranchu`/Cuttlefish builds use an equivalent virtio-based transport instead of the original goldfish pipe, but the architectural point is unchanged: it is a camera-HAL-level virtual device, never a USB device.)

### 3. Cross-reference

| | FTC `WebcamName` discovery | Emulator webcam passthrough |
|---|---|---|
| Enumeration API | `android.hardware.usb.UsbManager.getDeviceList()` (raw USB VID/PID) | `android.hardware.camera2.CameraManager` / legacy `Camera` (camera HAL ID list) |
| Attach/detach signal | `UsbManager.ACTION_USB_DEVICE_ATTACHED`/`_DETACHED` broadcast | Camera HAL device presence (goldfish `camera.goldfish.so` via `/dev/qemu_pipe`) |
| Frame transport | FTC's own native libusb + libuvc (UVC bulk/isochronous transfer against the raw USB device) | QEMU pipe -> host V4L2 -> camera HAL buffers |
| Underlying kernel object | `/dev/bus/usb/*` node with real VID/PID | none (no USB node at all) |

The two enumeration mechanisms are disjoint by construction: FTC's stack only ever looks at `UsbManager`, and the emulator's passthrough webcam only ever exists as a camera-HAL device. There is no overlap, so `WebcamName.isAttached()` is guaranteed `false` for a passthrough webcam, confirmed directly by reading the `getUsbDeviceNameIfAttached()` implementation above (it iterates `UsbManager.getDeviceList()`, which will simply never contain the passthrough camera).

## Implication for the vision initiative architecture

Because the incompatibility is structural (different Android subsystems entirely, not a missing permission or a fixable descriptor gap), **emulator webcam passthrough cannot be made to satisfy `WebcamName` discovery by configuration alone.** Options for a vision-development-in-emulator workflow, in order of increasing effort:

1. **Accept the limitation** — develop/test `VisionPortal`/`VisionProcessor` logic against recorded video files or synthetic frame sources outside the real `WebcamName`/`VisionPortal` device-open path (e.g. by unit-testing `VisionProcessor.processFrame()` directly with `Bitmap`/`Mat` frames from disk), and reserve the emulator for non-camera OpMode logic.
2. **Custom video relay** — build a small bridge that pushes frames from the host (or browser, if this project's OpMode dev loop runs through a browser-based front end) into the guest over an existing network/socket channel already available to the emulator (e.g. `adb forward`/`abstract socket`), landing them in a fake/custom `CameraName`+`Camera` implementation that feeds a `VisionPortal` (or bypasses `VisionPortal` and calls `VisionProcessor.processFrame()` directly with the relayed `Bitmap`). This is the "unmodified pipeline" compromise: the `VisionProcessor` code under test is unmodified, but camera acquisition is custom.
3. **Camera2-based custom `CameraName`** — write a custom `org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName`/`Camera` implementation that opens the emulator's passthrough webcam via `android.hardware.camera2.CameraManager` directly (since that *is* how the emulator exposes it) and adapts its frames into FTC's `CameraFrame`/`Bitmap` shape for `VisionPortal`. More work than (2) but keeps frame acquisition in-process rather than relayed.

Given the effort/value tradeoff, option 2 (custom relay into a fake `CameraName`) most closely matches what the issue calls "a custom video relay from the browser/host into the emulator" and is the recommended default unless a stronger reason favors option 3.

## Sources

- `org.firstinspires.ftc:RobotCore:11.2.0` sources jar (Maven, resolved locally): `org/firstinspires/ftc/robotcore/external/hardware/camera/WebcamName.java`, `org/firstinspires/ftc/robotcore/internal/camera/names/WebcamNameImpl.java`, `org/firstinspires/ftc/robotcore/internal/camera/CameraManagerImpl.java`.
- [Start the emulator from the command line — Android Studio](https://developer.android.com/studio/run/emulator-commandline) (`-camera-back`/`-camera-front`/`-webcam-list` reference table).
- [External USB cameras — Android Open Source Project](https://source.android.com/docs/core/camera/external-usb-cameras).
- [Camera support — Android Studio](https://developer.android.com/studio/run/emulator-use-camera) (confirms no USB-passthrough capability is documented for the emulator camera feature).
- AOSP `device/generic/goldfish` camera HAL (`camera.goldfish.so`, `EmulatedQemuCamera`/`CameraQemuClient` over `/dev/qemu_pipe`, service `qemud:camera`) — architecture corroborated via secondary technical writeups ("Running down a dream: Android, QEMU and the Camera", parts II–III) and the current AOSP `hardware/google/camera/devices/EmulatedCamera/hwl` tree structure.
