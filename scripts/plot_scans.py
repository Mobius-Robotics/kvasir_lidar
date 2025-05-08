# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "matplotlib",
#     "numpy",
#     "pyserial",
# ]
# ///
import queue
import signal
import struct
import threading
from typing import Iterator, NamedTuple

import matplotlib.pyplot as plt
import numpy as np
import serial

# Configuration
SERIAL_PORT = "COM4"
BAUD_RATE = 115_200
SERIAL_TIMEOUT = 0.1  # seconds
SERIAL_BUFFER_SIZE = 2048  # bytes

# LIDAR data format
# <SOF> <angle_deg (uint16)> <distance_mm (uint16)>
SOF = b"\xa5\x5a"
POINT_STRUCT_FORMAT = "<HH"
PACKET_SIZE = len(SOF) + struct.calcsize(POINT_STRUCT_FORMAT)


class Point(NamedTuple):
    angle_deg: int
    distance_mm: int


class ScanReader:
    def __init__(self):
        self.current_scan = []
        self.prev_angle = None
        self.raw_buffer = bytearray()

    def _process_next_point(self) -> Point | None:
        sof_index = self.raw_buffer.find(SOF)
        if sof_index == -1:
            return
        remaining_buffer = self.raw_buffer[sof_index:]
        if len(remaining_buffer) < PACKET_SIZE:
            return
        point_bytes = remaining_buffer[len(SOF) : PACKET_SIZE]
        self.raw_buffer = remaining_buffer[PACKET_SIZE:]

        return Point(*struct.unpack(POINT_STRUCT_FORMAT, point_bytes))

    def read(self, data: bytes) -> Iterator[list[Point]]:
        self.raw_buffer.extend(data)
        while point := self._process_next_point():
            if self.prev_angle is not None and point.angle_deg < self.prev_angle:
                finished_scan = self.current_scan
                self.current_scan = []
                yield finished_scan
            self.current_scan.append(point)
            self.prev_angle = point.angle_deg


def serial_reader_thread_func(data_queue: queue.Queue, shutdown_event: threading.Event):
    """
    Reads continuous point data from the serial port, groups into scans based on angle rollover, and
    puts full scans onto the queue.
    """

    port = None
    reader = ScanReader()

    try:
        port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        print(f"Serial reader: Opened {SERIAL_PORT} at {BAUD_RATE} baud.")

        while not shutdown_event.is_set():
            try:
                for finished_scan in reader.read(port.read(SERIAL_BUFFER_SIZE)):
                    try:
                        data_queue.put_nowait(finished_scan)
                    except queue.Full:
                        # Drop oldest and insert
                        try:
                            data_queue.get_nowait()
                        except queue.Empty:
                            # Race condition: our queue got emptied under us
                            pass
                        data_queue.put_nowait(finished_scan)
            except Exception as e:
                print(f"Serial reader: Unexpected error: {e}")
                shutdown_event.set()

    except serial.SerialException as e:
        print(f"Serial reader: Could not open port {SERIAL_PORT}: {e}")
        shutdown_event.set()
    finally:
        # After shutdown, flush any remaining scan
        if reader.current_scan:
            try:
                data_queue.put_nowait(reader.current_scan)
            except Exception:
                pass
        if port and port.is_open:
            port.close()
            print("Serial reader: Port closed.")
        print("Serial reader: Thread finished.")


def plot_realtime_data(data_queue, shutdown_event):
    plt.ion()
    fig, ax = plt.subplots(subplot_kw={"projection": "polar"})  # type: ignore
    scatter = ax.scatter([], [], s=5)  # type: ignore
    ax.set_theta_zero_location("N")  # type: ignore
    ax.set_theta_direction(-1)  # type: ignore
    ax.set_rlabel_position(0)  # type: ignore
    ax.set_title("LiDAR Scan Data", va="bottom")  # type: ignore
    ax.set_rmax(500)  # type: ignore
    ax.grid(True)  # type: ignore

    print("Plotter: Plotting started. Press Ctrl+C to exit.")

    try:
        while not shutdown_event.is_set():
            try:
                scan_points = data_queue.get(timeout=0.1)
                angles = np.array([p.angle_deg for p in scan_points])
                distances = np.array([p.distance_mm for p in scan_points])
                angles_rad = np.deg2rad(angles)

                offsets = np.vstack((angles_rad, distances)).T
                if len(offsets) != 0:
                    scatter.set_offsets(offsets)

                fig.canvas.draw_idle()  # type: ignore
                plt.pause(0.01)
            except queue.Empty:
                plt.pause(0.01)
            except Exception as e:
                print(f"Plotter: Error: {e}")
                shutdown_event.set()
    finally:
        plt.ioff()
        plt.close(fig)
        print("Plotter: Finished.")


# --- Main Program ---
def main():
    data_queue = queue.Queue(maxsize=2)  # Store max 2 full scans to prevent plotter lag
    shutdown_event = threading.Event()

    def signal_handler(*_):
        print("\nCtrl+C received. Shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    print(f"Using serial port: {SERIAL_PORT} at {BAUD_RATE} baud.")
    reader = threading.Thread(target=serial_reader_thread_func, args=(data_queue, shutdown_event))
    reader.start()
    plot_realtime_data(data_queue, shutdown_event)
    print("Main: Waiting for reader thread to finish...")
    reader.join(timeout=2)
    print("Main: Program terminated.")


if __name__ == "__main__":
    main()
