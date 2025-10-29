#!/usr/bin/env python3
"""
Thermal UDP Frame Receiver
Receives fragmented 640x480x16-bit frames via UDP and visualises or logs them.

Derived from Thermal_UDP_Streaming_Integration_Plan.md (lines 1880-2183).
"""

from __future__ import annotations

import argparse
import logging
import socket
import struct
import sys
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, Optional

import numpy as np


# ---------------------------------------------------------------------------
# Configuration constants (mirror firmware defaults)
# ---------------------------------------------------------------------------
UDP_PORT = 6100
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_PIXELS = FRAME_WIDTH * FRAME_HEIGHT
FRAME_SIZE_BYTES = FRAME_PIXELS * 2  # 16-bit pixels
PACKET_PAYLOAD_SIZE = 1400
EXPECTED_PACKETS = (FRAME_SIZE_BYTES + PACKET_PAYLOAD_SIZE - 1) // PACKET_PAYLOAD_SIZE

# Frame header struct: frame_num(uint32), packet_num(uint16), total_packets(uint16), timestamp(uint32)
HEADER_FORMAT = "<IHHI"  # little-endian layout from integration plan
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# ---------------------------------------------------------------------------
# Thermal frame receiver implementation
# ---------------------------------------------------------------------------
class ThermalFrameReceiver:
    """Collects UDP fragments, reconstructs thermal frames, and reports statistics."""

    def __init__(self, bind_ip: str, port: int, log: logging.Logger) -> None:
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(1.0)  # allow graceful shutdown / stats updates

        self.frames: Dict[int, Dict[int, bytes]] = defaultdict(dict)
        self.frame_times: Deque[float] = deque(maxlen=200)
        self.stats = {
            "received_packets": 0,
            "completed_frames": 0,
            "dropped_frames": 0,
            "last_frame_num": -1,
        }

        self.log.info(
            "[RX] Listening on %s:%d (expect %d packets / frame)",
            bind_ip,
            port,
            EXPECTED_PACKETS,
        )
        self.log.info(
            "[RX] Frame geometry: %dx%d (%d bytes)",
            FRAME_WIDTH,
            FRAME_HEIGHT,
            FRAME_SIZE_BYTES,
        )

    # ---------------------------------------------------------------------
    def close(self) -> None:
        self.sock.close()

    # ---------------------------------------------------------------------
    def receive_packet(self) -> Optional[dict]:
        """Receive and parse a single UDP packet."""
        try:
            data, addr = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        except OSError as exc:  # socket closed
            self.log.debug("[RX] Socket closed: %s", exc)
            return None

        if len(data) < HEADER_SIZE:
            self.log.warning("[RX] Packet too small (%d bytes) from %s", len(data), addr)
            return None

        frame_num, packet_num, total_packets, timestamp = struct.unpack(
            HEADER_FORMAT, data[:HEADER_SIZE]
        )

        payload = data[HEADER_SIZE:]
        self.stats["received_packets"] += 1

        if total_packets != EXPECTED_PACKETS:
            self.log.debug(
                "[RX] Adjusted total packet count: %d != expected %d",
                total_packets,
                EXPECTED_PACKETS,
            )

        return {
            "frame_num": frame_num,
            "packet_num": packet_num,
            "total_packets": total_packets,
            "timestamp": timestamp,
            "payload": payload,
            "addr": addr,
        }

    # ---------------------------------------------------------------------
    def process_packet(self, packet: Optional[dict]) -> Optional[np.ndarray]:
        """Store a packet and assemble full frames when all fragments arrive."""
        if packet is None:
            return None

        frame_num = packet["frame_num"]
        packet_num = packet["packet_num"]
        total_packets = packet["total_packets"]

        self.frames[frame_num][packet_num] = packet["payload"]

        if len(self.frames[frame_num]) < total_packets:
            return None

        # Reconstruct frame in-order
        frame_data = bytearray()
        for idx in range(total_packets):
            payload = self.frames[frame_num].get(idx)
            if payload is None:
                self.log.error("[RX] Missing packet %d in frame %d", idx, frame_num)
                return None
            frame_data.extend(payload)

        frame_array = np.frombuffer(frame_data, dtype=np.uint16, count=FRAME_PIXELS)
        frame_2d = frame_array.reshape((FRAME_HEIGHT, FRAME_WIDTH))

        # Update statistics
        self.stats["completed_frames"] += 1
        last_frame = self.stats["last_frame_num"]
        if last_frame >= 0:
            dropped = frame_num - last_frame - 1
            if dropped > 0:
                self.stats["dropped_frames"] += dropped
                self.log.warning("[RX] Dropped %d frame(s)", dropped)

        self.stats["last_frame_num"] = frame_num
        self.frame_times.append(time.time())

        # release storage
        del self.frames[frame_num]

        if self.stats["completed_frames"] % 10 == 0:
            fps = self.calculate_fps()
            self.log.info(
                "[RX] Frame %d: %d total, %.1f FPS, %d dropped",
                frame_num,
                self.stats["completed_frames"],
                fps,
                self.stats["dropped_frames"],
            )

        return frame_2d

    # ---------------------------------------------------------------------
    def calculate_fps(self) -> float:
        if len(self.frame_times) < 2:
            return 0.0
        elapsed = self.frame_times[-1] - self.frame_times[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self.frame_times) - 1) / elapsed

    # ---------------------------------------------------------------------
    def run_display(self, cmap: str = "hot") -> None:
        """Live Matplotlib display."""
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
        except ImportError as exc:
            self.log.error("Matplotlib required for display mode: %s", exc)
            raise

        fig, ax = plt.subplots(figsize=(10, 7.5))
        img = ax.imshow(
            np.zeros((FRAME_HEIGHT, FRAME_WIDTH)),
            cmap=cmap,
            vmin=0,
            vmax=65535,
        )
        ax.set_title("Thermal Camera Stream")
        plt.colorbar(img, ax=ax, label="Raw ADC Value")

        def update(_frame_num: int):
            packet = self.receive_packet()
            frame = self.process_packet(packet)
            if frame is not None:
                img.set_data(frame)
                fps = self.calculate_fps()
                ax.set_title(
                    f"Thermal Camera Stream - Frame {self.stats['completed_frames']} - {fps:.1f} FPS"
                )
                return [img]
            return []

        FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
        plt.show()

    # ---------------------------------------------------------------------
    def run_headless(self, save_frames: bool, num_frames: int, output_dir: Path) -> None:
        """Receive frames without GUI, optionally persisting first frames."""
        self.log.info("[RX] Headless test: target=%d frames, save=%s", num_frames, save_frames)
        frames_received = 0
        start_time = time.time()

        while frames_received < num_frames:
            packet = self.receive_packet()
            frame = self.process_packet(packet)

            if frame is None:
                continue

            frames_received += 1

            if save_frames and frames_received <= 10:
                output_dir.mkdir(parents=True, exist_ok=True)
                filename = output_dir / f"thermal_frame_{frames_received:04d}.npy"
                np.save(filename, frame)
                self.log.info("[RX] Saved %s", filename)

        elapsed = max(time.time() - start_time, 1e-6)
        avg_fps = frames_received / elapsed
        dropped = self.stats["dropped_frames"]
        total_attempted = frames_received + dropped
        loss_pct = (dropped / total_attempted * 100.0) if total_attempted else 0.0

        self.log.info("")
        self.log.info("[RX] Test complete:")
        self.log.info("  Frames received : %d", frames_received)
        self.log.info("  Time elapsed    : %.2f s", elapsed)
        self.log.info("  Average FPS     : %.2f", avg_fps)
        self.log.info("  Dropped frames  : %d", dropped)
        self.log.info("  Packet loss     : %.2f %%", loss_pct)


# ---------------------------------------------------------------------------
# Command-line interface
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Thermal UDP Frame Receiver")
    parser.add_argument("--bind", default="0.0.0.0", help="Local IP to bind (default: all interfaces)")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP port to listen on (default: 6100)")
    parser.add_argument("--headless", action="store_true", help="Run without GUI and exit after --frames")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames to capture in headless mode")
    parser.add_argument("--save-frames", action="store_true", help="Store first 10 frames as .npy in output directory")
    parser.add_argument("--output-dir", default=".", help="Directory for saved frames/logs (default: current)")
    parser.add_argument("--logfile", default="thermal_test.log", help="Path to log file (default: thermal_test.log)")
    return parser.parse_args()


def configure_logging(logfile: Path) -> logging.Logger:
    logfile.parent.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(logfile, mode="a", encoding="utf-8"),
        ],
    )
    return logging.getLogger("thermal_udp_receiver")


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    logfile = output_dir / args.logfile
    log = configure_logging(logfile)

    receiver = ThermalFrameReceiver(args.bind, args.port, log)

    try:
        if args.headless:
            receiver.run_headless(
                save_frames=args.save_frames,
                num_frames=max(1, args.frames),
                output_dir=output_dir,
            )
        else:
            log.info("[RX] Starting live display (use --headless for automated tests)")
            receiver.run_display()
    except KeyboardInterrupt:
        log.info("[RX] Interrupted by user")
    finally:
        receiver.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
