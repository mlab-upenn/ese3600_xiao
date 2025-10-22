# tinyml-mic-test (XIAO ESP32S3 Sense · PlatformIO)

Ready-to-run microphone test for the **Seeed XIAO ESP32S3 Sense** using **VS Code + PlatformIO**.

## Quick start

```bash
git clone <REPO_URL> tinyml-mic-test
cd tinyml-mic-test
# open folder in VS Code, ensure PlatformIO extension is installed
```

Build & upload:
- Status bar **▶ Upload**, or **View → Command Palette → "PlatformIO: Upload"**.

Open the Serial Monitor:
- **Ctrl+Alt+M** (Win/Linux) or **⌘⌥M** (macOS), or click the **plug icon** in the bottom-right.
- Set **115200 baud** and **Both NL & CR (CRLF)**.

Run:
- Press the **BOOT** button once (or type `click`) to start capture (~1s of samples).
- Press again (or type `click`) to stop.

Visualize (optional):
- Use **Arduino IDE → Tools → Serial Plotter** (close the PlatformIO monitor first), or
- Install a **Serial Plotter** extension in VS Code.

## Notes
- Port names: Windows `COM#`, macOS `/dev/cu.usbmodem#` or `/dev/cu.usbserial-…`, Linux `/dev/ttyACM#` or `/dev/ttyUSB#`.
- If upload fails, close any open Serial Monitor/plotter (the port can only be used by one tool).
