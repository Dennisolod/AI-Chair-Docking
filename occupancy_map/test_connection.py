import depthai as dai
try:
    print("Direct hardware ping...")
    # Using the specific UsbSpeed enum required by your version
    device = dai.Device(dai.UsbSpeed.HIGH) 
    print(f"Success! Connected to {device.getMxId()}")
    print(f"USB Speed: {device.getUsbSpeed()}")
    device.close()
except Exception as e:
    print(f"Hardware still unreachable: {e}")