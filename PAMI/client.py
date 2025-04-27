import requests
import time

esp32_ip = 'http://192.168.1.102'  # Replace with your ESP32 IP
timestamp = int(time.time()) + 1  # Set to run 10 seconds from now

res = requests.post(f'{esp32_ip}/timestamp', data=str(timestamp))
print(res.text)
