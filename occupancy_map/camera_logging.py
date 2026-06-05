import threading, time, csv, argparse
from pathlib import Path
import cv2
import depthai as dai

# Parsing arguments to get the sequence name
parser = argparse.ArgumentParser(description="Takes the name of your sequence as a string")
parser.add_argument("sequence")
args = parser.parse_args()
SEQUENCE = args.sequence

FPS = 20


# Directory setup
BASE_DIR = Path(SEQUENCE)

LEFT_DIR = BASE_DIR / "left"
RIGHT_DIR = BASE_DIR / "right"
RGB_DIR = BASE_DIR / "rgb"
IMU_PATH  = BASE_DIR / "imu.csv"
LEFT_PATH = BASE_DIR / "left_times.csv"
RIGHT_PATH = BASE_DIR / "right_times.csv"
RGB_PATH = BASE_DIR / "rgb_times.csv"
CALIB_PATH = BASE_DIR / "calib.json"

for d in (LEFT_DIR, RIGHT_DIR, RGB_DIR):
    d.mkdir(parents=True, exist_ok=True)

IMU_PATH.parent.mkdir(parents=True, exist_ok=True)


# Dump the calibration data
try:
    with dai.Device() as dev:
        dev.readFactoryCalibration().eepromToJsonFile(str(CALIB_PATH))
except Exception as ex:
    print(f'No calibration: {ex}')


# Helpers
def timestamp_format(timestamp):
    return timestamp.days * 86400000000 + timestamp.seconds * 1000000 + timestamp.microseconds


# Thread functions
def read_frame(q, save_dir, csv_path, stop_event):
    csv_path = Path(csv_path)
    with csv_path.open("w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ms_python", "timestamp_us_depthai", "filename"])
        while not stop_event.is_set():
            frame = q.get()
            depthai_time_us = timestamp_format(frame.getTimestampDevice())
            py_time_ms = int(time.time() * 1000)
            img = frame.getCvFrame()
            fname = save_dir / f"{py_time_ms:012d}.png"
            is_frame_saved = cv2.imwrite(str(fname), img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            if not is_frame_saved:
                continue
            w.writerow([f"{py_time_ms}", f"{depthai_time_us}", fname.name])

def read_imu(q, csv_path, stop_event):
    csv_path = Path(csv_path)
    with csv_path.open("w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ms_python","timestamp_us_depthai","ax","ay","az","gx","gy","gz"])
        while not stop_event.is_set():
            data = q.get()
            for pkt in data.packets:
                accel = pkt.acceleroMeter
                gyro = pkt.gyroscope
                if accel is None or gyro is None: # if the packet doesnt contain both accel and gyro then skip it
                    continue
                depthai_time_us = timestamp_format(gyro.getTimestampDevice())
                py_time_ms = int(time.time() * 1000)
                w.writerow([f"{py_time_ms}", f"{depthai_time_us}", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z])


# Pipeline
pipeline = dai.Pipeline()

# IMU
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

# LEFT mono
left = pipeline.create(dai.node.Camera)
left.build(boardSocket=dai.CameraBoardSocket.CAM_B)
left_image = left.requestFullResolutionOutput(type=dai.ImgFrame.Type.GRAY8, fps=FPS)

# RIGHT mono
right = pipeline.create(dai.node.Camera)
right.build(boardSocket=dai.CameraBoardSocket.CAM_C)
right_image = right.requestFullResolutionOutput(type=dai.ImgFrame.Type.GRAY8, fps=FPS)

# RGB
rgb = pipeline.create(dai.node.Camera)
rgb.build(boardSocket=dai.CameraBoardSocket.CAM_A)
rgb_image = rgb.requestOutput(size=(1920, 1080), type=dai.ImgFrame.Type.BGR888p, fps=FPS)

stop_event = threading.Event()

q_left = left_image.createOutputQueue(maxSize=32)
q_right = right_image.createOutputQueue(maxSize=32)
q_rgb = rgb_image.createOutputQueue(maxSize=32)
q_imu = imu.out.createOutputQueue(maxSize=500)

pipeline.start()

t1 = threading.Thread(target=read_frame, args=(q_left, LEFT_DIR, LEFT_PATH, stop_event))
t2 = threading.Thread(target=read_frame, args=(q_right, RIGHT_DIR, RIGHT_PATH, stop_event))
t3 = threading.Thread(target=read_frame, args=(q_rgb, RGB_DIR, RGB_PATH, stop_event))
t4 = threading.Thread(target=read_imu, args=(q_imu, IMU_PATH, stop_event))

t1.start()
t2.start()
t3.start()
t4.start()

try:
    while not stop_event.is_set():
        time.sleep(0.2)
finally:
    stop_event.set()
    pipeline.stop()
    t1.join()
    t2.join()
    t3.join()
    t4.join()
