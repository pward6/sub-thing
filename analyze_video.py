#!/usr/bin/env python3
"""
analyze_video.py - Run gate_detector.find_gate over every frame of a video
and report how many frames the gate was found in.

    python3 analyze_video.py path/to.mkv
    python3 analyze_video.py path/to.mkv --every 5   # sample every 5th frame
"""

import argparse
import sys

import cv2

import gate_detector as gd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--every", type=int, default=1, help="process every Nth frame")
    ap.add_argument("--save", help="write an annotated video to this path")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"could not open {args.video!r}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    processed = 0
    found = 0
    confs = []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            gate = gd.find_gate(frame)
            processed += 1
            if gate:
                found += 1
                confs.append(gate["conf"])
            if writer:
                writer.write(gd.annotate(frame, gate))
        idx += 1

    cap.release()
    if writer:
        writer.release()

    print(f"video frames: {total}  ({total/fps:.1f}s @ {fps:.0f}fps)")
    print(f"processed: {processed}  gate found: {found} "
         f"({found/processed:.1%})" if processed else "processed: 0")
    if confs:
        print(f"confidence: min {min(confs):.2f}  mean {sum(confs)/len(confs):.2f}  "
             f"max {max(confs):.2f}")


if __name__ == "__main__":
    main()
