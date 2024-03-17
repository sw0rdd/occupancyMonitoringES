from machine import Pin, Timer
import time
import bluetooth
import struct
from micropython import const


# Constants for advertisement packet types
_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x3)
_ADV_TYPE_UUID32_COMPLETE = const(0x5)
_ADV_TYPE_UUID128_COMPLETE = const(0x7)


# Constants for BLE event types
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)


# Flags for GATT (Generic Attribute Profile) characteristics
_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)


# UUIDs for UART service and characteristics
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_READ | _FLAG_NOTIFY)
_UART_RX = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX))



# Generate an advertising payload for BLE advertising
def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None, appearance=0):
    payload = bytearray()


    def _append(adv_type, value):
        nonlocal payload
        payload += struct.pack("BB", len(value) + 1, adv_type) + value


    # Add flags to the payload
    _append(
        _ADV_TYPE_FLAGS,
        struct.pack("B", (0x01 if limited_disc else 0x02) + (0x18 if br_edr else 0x04)),
    )


    # Optionally add a name and service UUIDs to the payload
    if name:
        _append(_ADV_TYPE_NAME, name)
    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                _append(_ADV_TYPE_UUID16_COMPLETE, b)
            elif len(b) == 4:
                _append(_ADV_TYPE_UUID32_COMPLETE, b)
            elif len(b) == 16:
                _append(_ADV_TYPE_UUID128_COMPLETE, b)


    return payload




class BLESimplePeripheral:
    def __init__(self, ble, name="one-uart"):
        """
        Initialize the BLE peripheral.
        
        Parameters:
        - ble: The BLE interface
        - name: The name of the BLE device
        """
        self._ble = ble
        self._ble.active(True)  # Activate BLE
        self._ble.irq(self._irq)  # Set up an interrupt request handler for BLE events
        # Register UART service, creating characteristic handles for transmission (TX) and reception (RX)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()  # Keep track of active connections
        self._write_callback = None  # Callback function for write events
        # Prepare the advertising payload with the device name and UART service UUID
        self._payload = advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()  # Start advertising


    def _irq(self, event, data):
        """
        Handle BLE events.
        
        Parameters:
        - event: The event type
        - data: Data associated with the event
        """
        # Handling connection events
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)  # Add the connection to the tracking set
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)  # Remove the connection from the tracking set
            self._advertise()  # Restart advertising for new connections
        # Handling data write events to the RX characteristic
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)  # Call the write callback with the received value


    def send(self, data):
        """
        Send data to all connected clients.
        
        Parameters:
        - data: The data to send
        """
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)  # Notify all connected devices


    def is_connected(self):
        """
        Check if there are any active connections.
        
        Returns:
        True if there is at least one active connection, False otherwise.
        """
        return len(self._connections) > 0


    def _advertise(self, interval_us=500000):
        """
        Start advertising the BLE service.
        
        Parameters:
        - interval_us: Advertising interval in microseconds
        """
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)  # Begin advertising with the specified payload


    def on_write(self, callback):
        """
        Set a callback function for write events.
        
        Parameters:
        - callback: The function to call when a write event occurs
        """
        self._write_callback = callback  # Set the callback function for handling write events



# Function to read distance using ultrasonic sensor
def read_distance(trig_pin, echo_pin):
    # Ensure the trigger pin is low, and then emit a short pulse
    trig_pin.low()
    time.sleep_us(2)
    trig_pin.high()
    time.sleep_us(10)
    trig_pin.low()


    # Measure the duration for which the echo pin is high
    signal_off = signal_on = time.ticks_us()
    while echo_pin.value() == 0:
        signal_off = time.ticks_us()
    while echo_pin.value() == 1:
        signal_on = time.ticks_us()


    # Calculate distance based on the time of flight
    time_passed = signal_on - signal_off
    distance = (time_passed * 0.0343) / 2
    return distance



# The main function and its logic are defined here.
# This includes initialization, sensor setup, main loop for detecting movement and managing BLE connections and data transmission.
def main_function():
    time.sleep(2)  # Wait for 2 seconds for system stabilization
    print("Pico 1 starting")
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)
    
    led = Pin(18, Pin.OUT)
    
    # Sensor setup
    trig_pin_1 = Pin(15, Pin.OUT)
    echo_pin_1 = Pin(14, Pin.IN)
    trig_pin_2 = Pin(16, Pin.OUT)
    echo_pin_2 = Pin(17, Pin.IN)


    time.sleep(0.5) # Wait for 0.5 seconds for sensor stabilization


    threshold_distance = 10.0
    activation_time_threshold = 500  # Time threshold in milliseconds to consider two activations related
    last_activation_time_1 = 0
    last_activation_time_2 = 0
    movement_detected = False
    last_direction = None
    people_count = 0
    has_sent = False


    while True:
        current_time = time.ticks_ms()
        distance_1 = read_distance(trig_pin_1, echo_pin_1)
        distance_2 = read_distance(trig_pin_2, echo_pin_2)


        # Reset condition based on timeout or explicit user logic
        if current_time - max(last_activation_time_1, last_activation_time_2) > activation_time_threshold:
            movement_detected = False
            last_direction = None


        # Sensor 1 activation logic
        if distance_1 < threshold_distance:
            # print("Sensor 1 activated")
            if last_direction != "left" and not movement_detected:
                if current_time - last_activation_time_2 < activation_time_threshold:
                    movement_detected = True
                    last_direction = "right"
                    people_count += 1  # Increment on entering
                    # if people_count > 0:
                        # led.value(1)
                    print(f"people_count: {people_count}")
            last_activation_time_1 = current_time


        # Sensor 2 activation logic
        if distance_2 < threshold_distance:
            # print("Sensor 2 activated")
            if last_direction != "right" and not movement_detected:
                if current_time - last_activation_time_1 < activation_time_threshold:
                    movement_detected = True
                    last_direction = "left"
                    people_count -= 1  # Decrement on leaving
                    # if people_count < 0:
                        # led.value(0)
                    print(f"people_count: {people_count}")
            last_activation_time_2 = current_time


        # Reset movement_detected if no recent activations fit the expected sequence
        if movement_detected and (current_time - last_activation_time_1 > activation_time_threshold or current_time - last_activation_time_2 > activation_time_threshold):
            movement_detected = False
            last_direction = None


        # BLE connection and data sending logic
        if p.is_connected() and not has_sent:
            time.sleep_ms(1000) # give some time, then send only once
            p.send(struct.pack('i', people_count))  # Ensure to use 'i' for signed int
            print(f"Sending people_count: {people_count}")
            has_sent = True
            people_count = 0
        elif not p.is_connected():
            has_sent = False
            
        led.value(1) # led indicating that the main loop is running and not stuck somewhere
        time.sleep_ms(100)
        led.value(0)


if __name__ == "__main__":
    main_function()
