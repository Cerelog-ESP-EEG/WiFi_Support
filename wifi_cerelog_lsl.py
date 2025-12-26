import socket
import struct
import time
import sys
from pylsl import StreamInfo, StreamOutlet, local_clock

# --- Configuration ---
DEVICE_PORT = 4545      
DISCOVERY_PORT = 4445   

# --- Packet Constants ---
SAMPLING_RATE_HZ = 250.0
SAMPLE_PERIOD = 1.0 / SAMPLING_RATE_HZ
HARDWARE_VREF = 4.50
HARDWARE_GAIN = 24
GUI_CORRECTION_FACTOR = 24.0 * 0.6133
DATA_PACKET_START_MARKER = 0xABCD
DATA_PACKET_END_MARKER = 0xDCBA
DATA_PACKET_TOTAL_SIZE = 37
HANDSHAKE_START_MARKER_1 = 0xAA
HANDSHAKE_END_MARKER_1 = 0xCC
PACKET_IDX_LENGTH = 2
PACKET_IDX_CHECKSUM = 34
ADS1299_NUM_CHANNELS = 8
ADS1299_NUM_STATUS_BYTES = 3
ADS1299_BYTES_PER_CHANNEL = 3

class TcpSerial:
    def __init__(self, target_ip, port):
        self.sock = None
        self.buffer = bytearray()
        self.connect_with_retry(target_ip, port)

    def connect_with_retry(self, target_ip, port, retries=10):
        print(f"Attempting TCP connection to {target_ip}:{port}...")
        for i in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0) 
                self.sock.connect((target_ip, port))
                self.sock.setblocking(False)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f">>> SUCCESS: CONNECTED (Attempt {i+1})! <<<")
                return
            except Exception as e:
                print(f"  -> Attempt {i+1} Failed: {e}. Retrying in 1.5s...")
                if self.sock:
                    self.sock.close()
                time.sleep(1.5) # Increased delay to let ESP32 stabilize
        
        raise Exception(f"Connection Timed Out after {retries} attempts.")

    def write(self, data):
        try:
            self.sock.setblocking(True)
            self.sock.sendall(data)
            self.sock.setblocking(False)
        except Exception:
            pass

    def read(self, size):
        try:
            data = self.sock.recv(4096) 
            if data: self.buffer.extend(data)
        except Exception:
            pass
        if len(self.buffer) > 0:
            ret = self.buffer[:size]
            self.buffer = self.buffer[size:]
            return bytes(ret)
        return b''

    @property
    def in_waiting(self):
        try:
            data = self.sock.recv(4096)
            if data: self.buffer.extend(data)
        except Exception:
            pass
        return len(self.buffer)

    def close(self):
        if self.sock: self.sock.close()

def find_device_ip():
    print(f"Scanning for Cerelog Device on UDP port {DISCOVERY_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(1.0)
    try: sock.bind(('', 0))
    except: pass

    for i in range(5): 
        print(f"  -> Broadcast Ping {i+1}...")
        try:
            sock.sendto(b"CERELOG_FIND_ME", ('<broadcast>', DISCOVERY_PORT))
            start = time.time()
            while time.time() - start < 1.0:
                try:
                    data, addr = sock.recvfrom(1024)
                    if b"CERELOG_HERE" in data:
                        print(f"  -> FOUND DEVICE AT: {addr[0]}")
                        sock.close()
                        return addr[0]
                except socket.timeout: break
        except Exception as e: print(f"  Socket Error: {e}")
    sock.close()
    return None

def convert_to_microvolts(raw_val):
    scale_factor = (2 * HARDWARE_VREF / HARDWARE_GAIN) / (2**24)
    return raw_val * scale_factor * 1000000 * GUI_CORRECTION_FACTOR

def main():
    target_ip = find_device_ip()
    if not target_ip:
        target_ip = input("Enter IP Manually: ")

    # WAIT for ESP32 to recover from UDP Reply current spike
    print("Waiting for device to stabilize...")
    time.sleep(2.0)

    try:
        ser = TcpSerial(target_ip, DEVICE_PORT)
    except Exception as e:
        print(f"\nCRITICAL CONNECTION ERROR: {e}")
        return

    print("Creating LSL Stream Outlet...")
    info = StreamInfo('Cerelog_EEG', 'EEG', ADS1299_NUM_CHANNELS, SAMPLING_RATE_HZ, 'float32', 'cerelog_uid_1234')
    outlet = StreamOutlet(info)
    print("LSL Outlet Created. Sending Handshake...")

    current_unix_time = int(time.time())
    checksum_payload = struct.pack('>BI', 0x02, current_unix_time) + bytes([0x01, 0x04])
    checksum = sum(checksum_payload) & 0xFF
    handshake_packet = struct.pack('>BB', HANDSHAKE_START_MARKER_1, 0xBB) + checksum_payload + struct.pack('>B', checksum) + struct.pack('>BB', HANDSHAKE_END_MARKER_1, 0xDD)
    
    ser.write(handshake_packet)
    time.sleep(0.5)

    R = 0.995
    prev_x = [0.0] * ADS1299_NUM_CHANNELS
    prev_y = [0.0] * ADS1299_NUM_CHANNELS
    first_sample = True
    buffer = bytearray()
    start_marker = DATA_PACKET_START_MARKER.to_bytes(2, 'big')
    end_marker = DATA_PACKET_END_MARKER.to_bytes(2, 'big')
    next_schedule = local_clock()

    print("\n>>> STREAMING VIA WIFI >>>")

    try:
        while True:
            waiting = ser.in_waiting
            if waiting > 0:
                buffer.extend(ser.read(waiting))
            else:
                if len(buffer) < DATA_PACKET_TOTAL_SIZE:
                    time.sleep(0.001) 
                    continue

            while True:
                start_idx = buffer.find(start_marker)
                if start_idx == -1: break
                if len(buffer) < start_idx + DATA_PACKET_TOTAL_SIZE: break

                potential_packet = buffer[start_idx : start_idx + DATA_PACKET_TOTAL_SIZE]
                if potential_packet.endswith(end_marker):
                    payload = potential_packet[PACKET_IDX_LENGTH:PACKET_IDX_CHECKSUM]
                    if (sum(payload) & 0xFF) == potential_packet[PACKET_IDX_CHECKSUM]:
                        while local_clock() < next_schedule: pass 

                        ads_data = potential_packet[7:34] 
                        lsl_sample = []
                        for ch in range(ADS1299_NUM_CHANNELS):
                            idx = ADS1299_NUM_STATUS_BYTES + ch * ADS1299_BYTES_PER_CHANNEL
                            raw_val = int.from_bytes(ads_data[idx : idx + ADS1299_BYTES_PER_CHANNEL], byteorder='big', signed=True)
                            current_x = convert_to_microvolts(raw_val)
                            
                            if first_sample: current_y = 0.0
                            else: current_y = current_x - prev_x[ch] + (R * prev_y[ch])
                            
                            prev_x[ch] = current_x
                            prev_y[ch] = current_y
                            lsl_sample.append(current_y)
                        
                        first_sample = False
                        outlet.push_sample(lsl_sample, next_schedule)
                        next_schedule += SAMPLE_PERIOD
                        buffer = buffer[start_idx + DATA_PACKET_TOTAL_SIZE:]
                        continue
                buffer = buffer[start_idx + 1:]

    except KeyboardInterrupt:
        print("\nStopping Stream...")
    finally:
        if ser: ser.close()

if __name__ == "__main__":
    main()