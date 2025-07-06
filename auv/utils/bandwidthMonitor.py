import psutil
import time
import os

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

def get_interface_name():
    interfaces = psutil.net_io_counters(pernic=True)
    print("Available interfaces:")
    for i, name in enumerate(interfaces.keys()):
        print(f"{i}: {name}")
    index = int(input("Enter the interface number to monitor: "))
    return list(interfaces.keys())[index]

def monitor_bandwidth(interface, interval=1):
    print(f"\nMonitoring interface: {interface}")
    print("Press Ctrl+C to stop.\n")
    
    prev = psutil.net_io_counters(pernic=True)[interface]
    prev_time = time.time()

    try:
        while True:
            time.sleep(interval)
            curr = psutil.net_io_counters(pernic=True)[interface]
            curr_time = time.time()
            duration = curr_time - prev_time

            bytes_sent = curr.bytes_sent - prev.bytes_sent
            bytes_recv = curr.bytes_recv - prev.bytes_recv

            upload_speed = bytes_sent / duration / 1024  # KB/s
            download_speed = bytes_recv / duration / 1024  # KB/s

            clear_terminal()
            print(f"Monitoring: {interface}")
            print(f"Upload Speed  : {upload_speed:.2f} KB/s")
            print(f"Download Speed: {download_speed:.2f} KB/s")

            prev = curr
            prev_time = curr_time

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

if __name__ == "__main__":
    iface = get_interface_name()
    monitor_bandwidth(iface)
