#!/usr/bin/env python3
"""
AD7606 Ethernet UDP Receiver for Windows 11

Listens for UDP packets from Zynq-7020 AD7606 streaming system.
Reassembles multi-fragment bank data and saves per-channel binary files.

Output directory layout:
    <output_dir>/
        ch1.bin   -- int16 LE, channel 1 samples
        ch2.bin   -- int16 LE, channel 2 samples
        ch3.bin   -- int16 LE, channel 3 samples
        ch4.bin   -- int16 LE, channel 4 samples

Usage:
    python ad7606_receiver.py [--port 5001] [--output-dir ad7606_data] [--max-banks 10]
    python ad7606_receiver.py --duration 5 --output-dir capture_01

Protocol (per packet, little-endian):
    bytes 0..3   : bank_id  (uint32) 0=bank0, 1=bank1
    bytes 4..7   : frag_seq (uint32) fragment sequence, 0-based
    bytes 8..11  : start_idx(uint32) sample index offset in bank
    bytes 12..   : raw 16-bit ADC samples interleaved (ch1,ch2,ch3,ch4,...)

Each sample = 8 bytes = 4 x int16 (ch1, ch2, ch3, ch4)
Bank = 4096 samples = 32768 bytes
"""

import socket
import struct
import argparse
import sys
import time
from pathlib import Path

BANK_SAMPLE_COUNT = 4096
SAMPLES_PER_PACKET = 1460 // 8  # 182
HEADER_SIZE = 12
PACKET_PAYLOAD_MAX = SAMPLES_PER_PACKET * 8  # 1456 bytes

FRAGMENTS_PER_BANK = (BANK_SAMPLE_COUNT + SAMPLES_PER_PACKET - 1) // SAMPLES_PER_PACKET
CHANNELS = 4

# Set to "" to accept all senders, or set to sender IP to filter (e.g. "192.168.1.10")
TARGET_IP = "192.168.10.200"


def receive_data(port: int, output_dir: str, max_duration: float, max_banks: int,
                 target_ip: str = ""):
    """Receive UDP data and write per-channel binary files."""
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    ch_files = {}
    for ch in range(CHANNELS):
        fname = out_path / f"ch{ch + 1}.bin"
        ch_files[ch] = open(fname, "wb")
        print(f"Output CH{ch + 1}: {fname}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(10.0)

    print(f"\nListening on UDP port {port}...")
    if target_ip:
        print(f"Filtering by source IP: {target_ip}")
    if max_duration > 0:
        print(f"Duration: {max_duration:.1f}s")
    if max_banks > 0:
        print(f"Max banks: {max_banks}")
    print("Waiting for data from Zynq...\n")

    pending_banks = {}
    banks_received = 0
    bank_seq_counter = {0: 0, 1: 0}
    total_bytes_received = 0

    try:
        t_start = time.monotonic()
        while True:
            if max_duration > 0:
                elapsed = time.monotonic() - t_start
                if elapsed >= max_duration:
                    print(f"\n[STOP] Duration {max_duration}s reached.")
                    break

            if max_banks > 0 and banks_received >= max_banks:
                print(f"\n[STOP] {max_banks} banks received.")
                break

            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                print("[TIMEOUT] No data received for 10 seconds. Stopping.")
                break

            if target_ip and addr[0] != target_ip:
                continue

            if len(data) < HEADER_SIZE:
                print(f"[WARN] Received short packet ({len(data)} bytes) from {addr}")
                continue

            bank_id = struct.unpack_from("<I", data, 0)[0]
            frag_seq = struct.unpack_from("<I", data, 4)[0]
            start_idx = struct.unpack_from("<I", data, 8)[0]
            payload = data[HEADER_SIZE:]

            expected_payload = min(
                (BANK_SAMPLE_COUNT - start_idx) * 8,
                PACKET_PAYLOAD_MAX,
            )
            if len(payload) != expected_payload:
                print(
                    f"[WARN] Payload size mismatch: got {len(payload)}, "
                    f"expected {expected_payload} (bank={bank_id}, frag={frag_seq}, start={start_idx})"
                )

            total_bytes_received += len(data)

            bank_key = (bank_id, bank_seq_counter[bank_id])
            if bank_key not in pending_banks:
                pending_banks[bank_key] = {}
            pending_banks[bank_key][frag_seq] = (start_idx, payload)

            # Check if this bank is complete
            if len(pending_banks[bank_key]) == FRAGMENTS_PER_BANK:
                print(
                    f"[BANK{bank_id}] Complete: {FRAGMENTS_PER_BANK} fragments "
                    f"({BANK_SAMPLE_COUNT * 8} bytes) from {addr[0]}:{addr[1]}"
                )

                fragments = pending_banks.pop(bank_key)
                bank_data = bytearray(BANK_SAMPLE_COUNT * 8)
                for frag_seq_val, (s_idx, pld) in fragments.items():
                    byte_offset = s_idx * 8
                    bank_data[byte_offset : byte_offset + len(pld)] = pld

                # Deinterleave and write per-channel
                for i in range(BANK_SAMPLE_COUNT):
                    base = i * 8
                    ch_files[0].write(bank_data[base : base + 2])    # ch1
                    ch_files[1].write(bank_data[base + 2 : base + 4])  # ch2
                    ch_files[2].write(bank_data[base + 4 : base + 6])  # ch3
                    ch_files[3].write(bank_data[base + 6 : base + 8])  # ch4

                # Print first and last sample for verification
                ch1_0 = struct.unpack_from("<h", bank_data, 0)[0]
                ch2_0 = struct.unpack_from("<h", bank_data, 2)[0]
                ch3_0 = struct.unpack_from("<h", bank_data, 4)[0]
                ch4_0 = struct.unpack_from("<h", bank_data, 6)[0]

                last_off = (BANK_SAMPLE_COUNT - 1) * 8
                ch1_n = struct.unpack_from("<h", bank_data, last_off)[0]
                ch2_n = struct.unpack_from("<h", bank_data, last_off + 2)[0]
                ch3_n = struct.unpack_from("<h", bank_data, last_off + 4)[0]
                ch4_n = struct.unpack_from("<h", bank_data, last_off + 6)[0]

                print(f"  sample#0:     {ch1_0:6d} {ch2_0:6d} {ch3_0:6d} {ch4_0:6d}")
                print(f"  sample#{BANK_SAMPLE_COUNT-1}: {ch1_n:6d} {ch2_n:6d} {ch3_n:6d} {ch4_n:6d}")

                bank_seq_counter[bank_id] += 1
                banks_received += 1
                print(f"  Progress: {banks_received}/{max_banks} banks received\n")
            else:
                if frag_seq == 0:
                    print(
                        f"[BANK{bank_id}] Fragment 0 received, "
                        f"waiting for {FRAGMENTS_PER_BANK - 1} more..."
                    )

            # Clean up stale banks
            stale_keys = [
                k
                for k in pending_banks
                if k[1] < bank_seq_counter[k[0]] - 1
            ]
            for k in stale_keys:
                print(f"[WARN] Dropping stale bank {k}")
                del pending_banks[k]

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")

    finally:
        sock.close()
        for fh in ch_files.values():
            fh.close()

    print(f"\nDone. {banks_received} banks received.")
    print(f"Total bytes received: {total_bytes_received}")
    for ch in range(CHANNELS):
        fpath = out_path / f"ch{ch + 1}.bin"
        if fpath.exists():
            size_mb = fpath.stat().st_size / (1024 * 1024)
            print(f"CH{ch + 1}: {fpath}  ({size_mb:.2f} MB)")


def main():
    parser = argparse.ArgumentParser(description="AD7606 Ethernet UDP Receiver")
    parser.add_argument("--port", type=int, default=5001, help="UDP port to listen on")
    parser.add_argument("--output-dir", type=str, default="ad7606_data",
                        help="Output directory for per-channel binary files")
    parser.add_argument("--duration", type=float, default=0,
                        help="Collection duration in seconds (0=until --max-banks)")
    parser.add_argument("--max-banks", type=int, default=0,
                        help="Number of banks to receive (0=unlimited, use --duration instead)")
    args = parser.parse_args()

    if args.duration <= 0 and args.max_banks <= 0:
        print("Error: Specify --duration or --max-banks")
        sys.exit(1)

    receive_data(args.port, args.output_dir, args.duration, args.max_banks,
                 TARGET_IP)


if __name__ == "__main__":
    main()
