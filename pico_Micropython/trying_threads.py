from machine import Pin, Timer
import time
import bluetooth
import struct
from ble_advertising import advertising_payload
from micropython import const
import _thread  # Import the threading module

# Global variables
global people_count
people_count = 0

lock = _thread.allocate_lock()  # Create a lock object


# Function to read distance (remains unchanged)
def read_distance(trig_pin, echo_pin):
    trig_pin.low()
    time.sleep_us(2)
    trig_pin.high()
    time.sleep_us(10)  # Send a 10-microsecond pulse.
    trig_pin.low()

    signal_off = signal_on = time.ticks_us() # Initialize the variables

    while echo_pin.value() == 0:
        signal_off = time.ticks_us()
    while echo_pin.value() == 1:
        signal_on = time.ticks_us()

    time_passed = signal_on - signal_off
    distance = (time_passed * 0.0343) / 2
    return distance


_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)


class BLESimplePeripheral:
    def __init__(self, ble, name="one-uart"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection. Alsways connects when there is an available node.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)

    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def on_write(self, callback):
        self._write_callback = callback


def movement_detection():
    global people_count
    # Sensor setup
    trig_pin_1 = Pin(15, Pin.OUT)
    echo_pin_1 = Pin(14, Pin.IN)
    trig_pin_2 = Pin(0, Pin.OUT)
    echo_pin_2 = Pin(1, Pin.IN)

    threshold_distance = 100.0  # Threshold in cm
    activation_time_threshold = 500  # time threshold in milliseconds to consider two activations related
    last_activation_time_1 = 0
    last_activation_time_2 = 0
    movement_detected = False
    last_direction = None
    people_count = 0  

    while True:
        current_time = time.ticks_ms()
        distance_1 = read_distance(trig_pin_1, echo_pin_1)
        distance_2 = read_distance(trig_pin_2, echo_pin_2)

        # resett condition based on timeout or explicit user logic
        if current_time - max(last_activation_time_1, last_activation_time_2) > activation_time_threshold:
            movement_detected = False
            last_direction = None

        # sensor 1 activation logic
        if distance_1 < threshold_distance:
            if last_direction != "left" and not movement_detected:
                if current_time - last_activation_time_2 < activation_time_threshold:
                    movement_detected = True
                    last_direction = "right"

                    lock.acquire()  # Acquire the lock
                    try:
                        people_count += 1  # Increment on entering
                        print(f"people_count: {people_count}")
                    finally:
                        lock.release()

            last_activation_time_1 = current_time

        # Sensor 2 activation logic
        if distance_2 < threshold_distance:
            if last_direction != "right" and not movement_detected:
                if current_time - last_activation_time_1 < activation_time_threshold:
                    movement_detected = True
                    last_direction = "left"

                    lock.acquire()  # Acquire the lock
                    try:
                        people_count -= 1  # Decrement on leaving
                        print(f"people_count: {people_count}")
                    finally:
                        lock.release()

            last_activation_time_2 = current_time

        # Reset movement_detected if no recent activations fit the expected sequence
        if movement_detected and (current_time - last_activation_time_1 > activation_time_threshold or current_time - last_activation_time_2 > activation_time_threshold):
            movement_detected = False
            last_direction = None  

        time.sleep_ms(100)  # we must avoid busy waiting




def main_function():
    global people_count
    time.sleep(2)  # Wait for 2 seconds for system stabilization

    print("Pico 1 starting")
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)

    # thread for movement detection
    _thread.start_new_thread(movement_detection, ())

    has_sent = False
    while True:
        # BLE connection and data sending logic, using global people_count
        if p.is_connected() and not has_sent:
            time.sleep_ms(1000)  # delaying before sending...

            lock.acquire()  # Acquire the lock
            try:
                p.send(struct.pack('i', people_count))  
                print(f"Sending people_count: {people_count}")
                people_count = 0  # Reset people_count after sending
                has_sent = True
            finally:
                lock.release()
        elif not p.is_connected():
            has_sent = False

        time.sleep_ms(100)
    



if __name__ == "__main__":
    main_function()


