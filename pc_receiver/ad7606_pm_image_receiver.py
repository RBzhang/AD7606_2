#!/usr/bin/env python3
"""
AD7606 UDP PM-image receiver for direct DAC capture validation.

This script reuses the AD7606_2 UDP bank protocol, extracts one ADC channel
(default: CH1), detects the Barker-13 preamble in the sampled DAC voltage, and
reconstructs grayscale image frames.

Typical use:
    python pc_receiver/ad7606_pm_image_receiver.py \
        --port 5001 \
        --target-ip 192.168.10.200 \
        --frame-width 320 \
        --frame-height 180 \
        --adc-sample-rate 1000000 \
        --dac-sample-rate 100000 \
        --channel 1 \
        --max-frames 10

UDP protocol inherited from pc_receiver/ad7606_receiver.py:
    bytes 0..3   : bank_id   (uint32 LE)
    bytes 4..7   : frag_seq  (uint32 LE)
    bytes 8..11  : start_idx (uint32 LE, sample index in bank)
    bytes 12..   : int16 LE interleaved samples: ch1,ch2,ch3,ch4,...
"""

from __future__ import annotations

import argparse
import shutil
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from PIL import Image


# AD7606 UDP stream constants. Each logical sample contains 4 x int16 channels.
BANK_SAMPLE_COUNT = 4096
CHANNELS = 4
BYTES_PER_LOGICAL_SAMPLE = CHANNELS * 2
SAMPLES_PER_PACKET = 1460 // BYTES_PER_LOGICAL_SAMPLE  # 182
HEADER_SIZE = 12
PACKET_PAYLOAD_MAX = SAMPLES_PER_PACKET * BYTES_PER_LOGICAL_SAMPLE
FRAGMENTS_PER_BANK = (BANK_SAMPLE_COUNT + SAMPLES_PER_PACKET - 1) // SAMPLES_PER_PACKET

# Sender IP used by the existing AD7606 receiver. Can be overridden by --target-ip.
DEFAULT_TARGET_IP = "192.168.10.200"

BARKER_13 = np.array(
    [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1],
    dtype=np.float64,
)


class PmVoltageFrameDecoder:
    """Streaming decoder for voltage samples captured from the DAC output.

    The transmitter sends one Barker-13 preamble followed by one image frame.
    The ADC samples the analog DAC voltage, so the receiver estimates the low
    and high voltage levels from the detected Barker preamble and then maps the
    following frame samples back to uint8 grayscale values.
    """

    def __init__(
        self,
        frame_width: int,
        frame_height: int,
        samples_per_symbol: int,
        sync_chip_symbols: int = 32,
        corr_thresh: float = 0.85,
        min_sync_span: float = 0.0,
        max_buffer_symbols: int = 8,
    ) -> None:
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError("frame_width and frame_height must be positive")
        if samples_per_symbol <= 0:
            raise ValueError("samples_per_symbol must be positive")
        if sync_chip_symbols <= 0:
            raise ValueError("sync_chip_symbols must be positive")

        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.frame_size = self.frame_width * self.frame_height
        self.samples_per_symbol = int(samples_per_symbol)
        self.sync_chip_symbols = int(sync_chip_symbols)
        self.corr_thresh = float(corr_thresh)
        self.min_sync_span = float(min_sync_span)

        self.chip_samples = self.sync_chip_symbols * self.samples_per_symbol
        self.sync_len = len(BARKER_13) * self.chip_samples
        self.frame_sample_count = self.frame_size * self.samples_per_symbol

        self._sync_chips = np.repeat(BARKER_13, self.chip_samples)
        self._high_mask = self._sync_chips > 0
        self._low_mask = self._sync_chips < 0

        # Correlation template with DC and linear trend removed. This makes sync
        # detection less sensitive to ADC offset and slow baseline drift.
        self._trend_index = (
            np.arange(self.sync_len, dtype=np.float64) - (self.sync_len - 1) / 2.0
        )
        self._trend_energy = float(np.dot(self._trend_index, self._trend_index))
        template = self._sync_chips - np.mean(self._sync_chips)
        template_slope = float(np.dot(template, self._trend_index) / self._trend_energy)
        template = template - template_slope * self._trend_index
        self._template = template
        self._template_energy = float(np.dot(template, template))
        self._template_norm = float(np.sqrt(self._template_energy))

        # Keep enough data for sync search plus several frames, but avoid unbounded
        # growth if no valid sync is found.
        self.max_buffer_samples = max(
            self.sync_len + self.frame_sample_count,
            (self.sync_len + self.frame_sample_count) * max(1, int(max_buffer_symbols)),
        )
        self._buffer = np.empty(0, dtype=np.float64)
        self.frames_decoded = 0

    def push(self, samples: np.ndarray) -> List[Tuple[np.ndarray, dict]]:
        """Append samples and return all newly decoded frames."""
        if samples.size == 0:
            return []

        x = np.asarray(samples, dtype=np.float64)
        self._buffer = np.concatenate((self._buffer, x))

        frames: List[Tuple[np.ndarray, dict]] = []
        while True:
            match = self._find_sync(self._buffer)
            if match is None:
                # Keep the tail needed for a future sync window, plus some margin.
                if len(self._buffer) > self.max_buffer_samples:
                    keep = min(len(self._buffer), self.sync_len - 1)
                    self._buffer = self._buffer[-keep:]
                break

            sync_start, sync_end, stats = match
            frame_start = sync_end + 1
            frame_end = frame_start + self.frame_sample_count
            if len(self._buffer) < frame_end:
                # Wait for more ADC samples. Drop old samples before the sync start.
                if sync_start > 0:
                    self._buffer = self._buffer[sync_start:]
                break

            sync_window = self._buffer[sync_start : sync_end + 1]
            payload_samples = self._buffer[frame_start:frame_end]
            image, level_stats = self._decode_payload(sync_window, payload_samples)

            self.frames_decoded += 1
            info = {
                "frame_index": self.frames_decoded - 1,
                "sync_start": int(sync_start),
                "sync_end": int(sync_end),
                **stats,
                **level_stats,
            }
            frames.append((image, info))

            # Remove the decoded frame and search for the next Barker preamble.
            self._buffer = self._buffer[frame_end:]

        return frames

    def _find_sync(self, samples: np.ndarray) -> Optional[Tuple[int, int, dict]]:
        if len(samples) < self.sync_len:
            return None

        a = np.asarray(samples, dtype=np.float64)
        cumulative = np.concatenate(([0.0], np.cumsum(a)))
        cumulative_sq = np.concatenate(([0.0], np.cumsum(a * a)))
        window_sum = cumulative[self.sync_len :] - cumulative[: -self.sync_len]
        window_sq = cumulative_sq[self.sync_len :] - cumulative_sq[: -self.sync_len]

        trend_projection = np.correlate(a, self._trend_index, mode="valid")
        sync_projection = np.correlate(a, self._template, mode="valid")
        detrended_energy = (
            window_sq
            - window_sum * window_sum / self.sync_len
            - trend_projection * trend_projection / self._trend_energy
        )
        detrended_energy = np.maximum(detrended_energy, 1e-12)

        correlation = sync_projection / (np.sqrt(detrended_energy) * self._template_norm)
        amplitude = sync_projection / self._template_energy
        residual_energy = detrended_energy - sync_projection * sync_projection / self._template_energy
        noise = np.sqrt(np.maximum(residual_energy, 0.0) / self.sync_len)

        # Estimate span from candidate windows. This is optional and disabled by
        # default because ADC count ranges depend on analog gain/range settings.
        matches = np.flatnonzero(
            (correlation > self.corr_thresh)
            & (np.abs(2.0 * amplitude) >= self.min_sync_span)
        )
        if len(matches) == 0:
            return None

        best = int(matches[np.argmax(correlation[matches])])
        end = best + self.sync_len - 1
        stats = {
            "corr": float(correlation[best]),
            "sync_amp_half_span": float(amplitude[best]),
            "sync_noise": float(noise[best]),
        }
        return best, end, stats

    def _decode_payload(
        self,
        sync_window: np.ndarray,
        payload_samples: np.ndarray,
    ) -> Tuple[np.ndarray, dict]:
        high_samples = np.asarray(sync_window[self._high_mask], dtype=np.float64)
        low_samples = np.asarray(sync_window[self._low_mask], dtype=np.float64)

        # Median is robust to occasional ADC spikes or packet-level glitches.
        sync_high = float(np.median(high_samples))
        sync_low = float(np.median(low_samples))
        span = sync_high - sync_low
        if abs(span) < 1e-12:
            raise RuntimeError("Detected Barker sync has near-zero high/low span")

        payload = np.asarray(payload_samples, dtype=np.float64)
        if self.samples_per_symbol > 1:
            payload = payload.reshape(self.frame_size, self.samples_per_symbol)
            # Average samples belonging to one DAC symbol. A normal arithmetic mean
            # is correct here because this is voltage, not wrapped phase.
            pixel_level = payload.mean(axis=1)
        else:
            pixel_level = payload

        normalized = (pixel_level - sync_low) / span
        image_u8 = np.clip(np.rint(normalized * 255.0), 0, 255).astype(np.uint8)
        image = image_u8.reshape(self.frame_height, self.frame_width)

        level_stats = {
            "sync_low": sync_low,
            "sync_high": sync_high,
            "sync_span": float(span),
            "payload_min": float(np.min(pixel_level)),
            "payload_max": float(np.max(pixel_level)),
            "payload_mean": float(np.mean(pixel_level)),
        }
        return image, level_stats


class Ad7606UdpBankReceiver:
    """Reassemble AD7606 UDP fragments into complete 4096-sample banks."""

    def __init__(self, port: int, target_ip: str = "", timeout_s: float = 10.0) -> None:
        self.target_ip = target_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", int(port)))
        self.sock.settimeout(float(timeout_s))

        self.pending_banks: Dict[Tuple[int, int], Dict[int, Tuple[int, bytes]]] = {}
        self.bank_seq_counter = {0: 0, 1: 0}
        self.total_bytes_received = 0

    def close(self) -> None:
        self.sock.close()

    def receive_bank(self) -> Optional[Tuple[int, np.ndarray, Tuple[str, int]]]:
        """Block until one complete bank is available, then return it.

        Returns:
            (bank_id, samples, sender), where samples has shape (4096, 4)
            and dtype int16.
        """
        while True:
            data, addr = self.sock.recvfrom(65536)
            if self.target_ip and addr[0] != self.target_ip:
                continue
            if len(data) < HEADER_SIZE:
                print(f"[WARN] short UDP packet: {len(data)} bytes from {addr}")
                continue

            bank_id = struct.unpack_from("<I", data, 0)[0]
            frag_seq = struct.unpack_from("<I", data, 4)[0]
            start_idx = struct.unpack_from("<I", data, 8)[0]
            payload = data[HEADER_SIZE:]

            if bank_id not in self.bank_seq_counter:
                print(f"[WARN] unexpected bank_id={bank_id}; accepting with sequence 0")
                self.bank_seq_counter[bank_id] = 0

            expected_payload = min(
                (BANK_SAMPLE_COUNT - start_idx) * BYTES_PER_LOGICAL_SAMPLE,
                PACKET_PAYLOAD_MAX,
            )
            if len(payload) != expected_payload:
                print(
                    f"[WARN] payload size mismatch: got {len(payload)}, "
                    f"expected {expected_payload} (bank={bank_id}, frag={frag_seq}, start={start_idx})"
                )

            self.total_bytes_received += len(data)
            bank_key = (bank_id, self.bank_seq_counter[bank_id])
            self.pending_banks.setdefault(bank_key, {})[frag_seq] = (start_idx, payload)

            if len(self.pending_banks[bank_key]) != FRAGMENTS_PER_BANK:
                continue

            fragments = self.pending_banks.pop(bank_key)
            bank_data = bytearray(BANK_SAMPLE_COUNT * BYTES_PER_LOGICAL_SAMPLE)
            for _frag, (s_idx, pld) in fragments.items():
                byte_offset = s_idx * BYTES_PER_LOGICAL_SAMPLE
                bank_data[byte_offset : byte_offset + len(pld)] = pld

            samples = np.frombuffer(bank_data, dtype="<i2").reshape(BANK_SAMPLE_COUNT, CHANNELS).copy()
            self.bank_seq_counter[bank_id] += 1
            return bank_id, samples, addr


def save_frame(image: np.ndarray, output_dir: Path, frame_index: int, save_latest: bool) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"frame_{frame_index:04d}.png"
    Image.fromarray(image, mode="L").save(path)
    if save_latest:
        latest = output_dir / "latest.png"
        shutil.copyfile(path, latest)
    return path


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode PM grayscale images from AD7606 UDP samples captured on CH1."
    )
    parser.add_argument("--port", type=int, default=5001, help="UDP port to listen on")
    parser.add_argument(
        "--target-ip",
        type=str,
        default=DEFAULT_TARGET_IP,
        help="Only accept packets from this sender IP; use empty string to accept all",
    )
    parser.add_argument("--output-dir", type=str, default="ad7606_pm_frames")
    parser.add_argument("--frame-width", type=int, default=320)
    parser.add_argument("--frame-height", type=int, default=180)
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        choices=range(1, CHANNELS + 1),
        metavar="{1,2,3,4}",
        help="AD7606 channel carrying the DAC output; default CH1",
    )
    parser.add_argument(
        "--adc-sample-rate",
        type=float,
        default=1_000_000.0,
        help="AD7606 sample rate in samples/s for one channel",
    )
    parser.add_argument(
        "--dac-sample-rate",
        type=float,
        default=100_000.0,
        help="DAC output symbol/sample rate used by the transmitter",
    )
    parser.add_argument(
        "--samples-per-symbol",
        type=int,
        default=0,
        help="Override adc_sample_rate/dac_sample_rate ratio; 0=auto round",
    )
    parser.add_argument(
        "--sync-chip-symbols",
        type=int,
        default=32,
        help="Transmitter Barker chip length in DAC symbols",
    )
    parser.add_argument("--corr-thresh", type=float, default=0.85)
    parser.add_argument(
        "--min-sync-span",
        type=float,
        default=0.0,
        help="Minimum Barker high-low span in ADC counts after correlation; 0 disables",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds; 0=disabled")
    parser.add_argument("--max-banks", type=int, default=0, help="Stop after N complete banks; 0=disabled")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N decoded frames; 0=unlimited")
    parser.add_argument("--no-latest", action="store_true", help="Do not update latest.png")
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    if args.samples_per_symbol > 0:
        samples_per_symbol = args.samples_per_symbol
    else:
        if args.dac_sample_rate <= 0:
            print("Error: --dac-sample-rate must be positive", file=sys.stderr)
            return 2
        samples_per_symbol = max(1, int(round(args.adc_sample_rate / args.dac_sample_rate)))

    output_dir = Path(args.output_dir)
    decoder = PmVoltageFrameDecoder(
        frame_width=args.frame_width,
        frame_height=args.frame_height,
        samples_per_symbol=samples_per_symbol,
        sync_chip_symbols=args.sync_chip_symbols,
        corr_thresh=args.corr_thresh,
        min_sync_span=args.min_sync_span,
    )

    target = args.target_ip.strip()
    receiver = Ad7606UdpBankReceiver(args.port, target_ip=target)

    print("\nAD7606 PM image receiver")
    print(f"  UDP port           : {args.port}")
    print(f"  Target IP          : {target if target else '(any)'}")
    print(f"  ADC channel         : CH{args.channel}")
    print(f"  Frame size          : {args.frame_width} x {args.frame_height}")
    print(f"  ADC sample rate     : {args.adc_sample_rate:g} S/s")
    print(f"  DAC sample rate     : {args.dac_sample_rate:g} S/s")
    print(f"  Samples per symbol  : {samples_per_symbol}")
    print(f"  Sync length         : {decoder.sync_len} ADC samples")
    print(f"  Payload length      : {decoder.frame_sample_count} ADC samples/frame")
    print(f"  Output directory    : {output_dir}")
    print("\nConnect DAC output to AD7606 CH1 and start the Zynq UDP streamer.\n")

    banks_received = 0
    frames_received = 0
    t_start = time.monotonic()

    try:
        while True:
            if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
                print(f"[STOP] duration reached: {args.duration:.1f}s")
                break
            if args.max_banks > 0 and banks_received >= args.max_banks:
                print(f"[STOP] max banks reached: {args.max_banks}")
                break
            if args.max_frames > 0 and frames_received >= args.max_frames:
                print(f"[STOP] max frames reached: {args.max_frames}")
                break

            try:
                bank_id, bank_samples, addr = receiver.receive_bank()
            except socket.timeout:
                print("[TIMEOUT] no UDP data received for 10 seconds")
                break

            banks_received += 1
            selected = bank_samples[:, args.channel - 1]
            decoded = decoder.push(selected)

            if banks_received == 1 or (banks_received % 16) == 0:
                print(
                    f"[BANK] #{banks_received} bank_id={bank_id} from {addr[0]}:{addr[1]} "
                    f"CH{args.channel} min={int(selected.min())} max={int(selected.max())}"
                )

            for image, info in decoded:
                path = save_frame(image, output_dir, frames_received, save_latest=not args.no_latest)
                frames_received += 1
                print(
                    f"[FRAME] {info['frame_index']} saved {path} | "
                    f"corr={info['corr']:.3f}, "
                    f"low={info['sync_low']:.1f}, high={info['sync_high']:.1f}, "
                    f"span={info['sync_span']:.1f}, "
                    f"payload=[{info['payload_min']:.1f}, {info['payload_max']:.1f}]"
                )

                if args.max_frames > 0 and frames_received >= args.max_frames:
                    break

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        receiver.close()

    print("\nDone.")
    print(f"  Banks received : {banks_received}")
    print(f"  Frames decoded : {frames_received}")
    print(f"  UDP bytes      : {receiver.total_bytes_received}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
