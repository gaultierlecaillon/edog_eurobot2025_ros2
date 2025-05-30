import requests
import time
import threading

esp32_ips = ['http://pami1.local', 'http://pami2.local']  # ESP32 IPs to trigger
timestamp = int(time.time()) + 1  # Set to run 10 seconds from now

def send_request(esp32_ip, timestamp):
    try:
        res = requests.post(f'{esp32_ip}/timestamp', data=str(timestamp))
        print(f'{esp32_ip}: {res.text}')
    except Exception as e:
        print(f'{esp32_ip}: Error - {e}')

# Create threads to send requests simultaneously
threads = []
for esp32_ip in esp32_ips:
    thread = threading.Thread(target=send_request, args=(esp32_ip, timestamp))
    threads.append(thread)
    thread.start()

# Wait for all threads to complete
for thread in threads:
    thread.join()
