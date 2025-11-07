import simplepyble
import keyboard
import time

found_device = False

# device UUID of interest
YOUR_ADDRESS = "c0:98:e5:49:aa:bb" # Replace address with your device address

# service-characteristic pairs of interest
##SERV_CHAR_PAIRS = {"c05899c4-457c-4c75-93ab-e55018bb3073": "c05899c5-457c-4c75-93ab-e55018bb3073",
##                   "c05899c4-457c-4c75-93ab-e55018bb3073": "c05899c6-457c-4c75-93ab-e55018bb3073",
##                   "c05899c4-457c-4c75-93ab-e55018bb3073": "c05899c7-457c-4c75-93ab-e55018bb3073",
##                   "c05899c4-457c-4c75-93ab-e55018bb3073": "c05899c8-457c-4c75-93ab-e55018bb3073"}
# services of interest
SERVICE_UUID = {"led":"32e61089-2b22-4db5-a914-43ce41986c70",
                "drive":"c05899c4-457c-4c75-93ab-e55018bb3073"}
# characteristics of interest
CHAR_UUIDS = {"up": "c05899c5-457c-4c75-93ab-e55018bb3073",
        "down": "c05899c6-457c-4c75-93ab-e55018bb3073",
        "right": "c05899c7-457c-4c75-93ab-e55018bb3073",
        "left": "c05899c8-457c-4c75-93ab-e55018bb3073",
        "led":"32e68911-2b22-4db5-a914-43ce41986c70"}
YOUR_NAME = "KOBUKI"



LOW = b'\x00'
HIGH = b'\x01'

robot_fwd = b'\x01'

peripherals= []
service_uuid = []
characteristic_uuid = []
class RobotController():
    def __del__(self):
        print('Destructor called, Object deleted.')
    def print_conents(self, service_uuid):
        for characteristics in CHAR_UUIDS.get_keys():
            contents = self.peripheral.read(self.service_uuid, self.characteristic_uuid)
    def __init__(self, address):

        adapters = simplepyble.Adapter.get_adapters()

        found_device = False
        connect_device = False

        self.pressed = {"up": False, "down": False, "right": False, "left": False}
        self.chars = {"up": None, "down": None, "right": None, "left": None, "led": None}
        self.drive = dict()
        self.led   = dict()
        self.drive_service = []


        if len(adapters) == 0:
            print("No adapters found")

        # we can pick the top most in the list by default. If it does not work for you, print the list of adapters and choose the appropriate one.
        adapter = adapters[0]

        print(f"Selected adapter: {adapter.identifier()} [{adapter.address()}]")

        adapter.set_callback_on_scan_start(lambda: print("Scan started."))
        adapter.set_callback_on_scan_stop(lambda: print("Scan complete.\n"))
        adapter.set_callback_on_scan_found(lambda peripheral: print(f"Found {peripheral.identifier()} [{peripheral.address()}]"))

        # Scan for 3 seconds
        adapter.scan_for(3000)
        peripherals.append(adapter.scan_get_results())

        # Query the user to pick a peripheral
        print("Selecting the Peripheral from the Provided Address")
        for i, peripheral in enumerate(peripherals[0]):
            print(f"{i}: {peripheral.identifier()} [{peripheral.address()}]")
            if peripheral.identifier()==YOUR_NAME:
                found_device = True
                connect_device = True
                self.peripheral=peripherals[0][i]
                print(f"Connecting to: {self.peripheral.identifier()} [{self.peripheral.address()}]")
                self.peripheral.connect()
                print("Successfully connected, listing services...")
                services = self.peripheral.services()
                service_characteristic_pair = []
                ######## Printing the Services and Characteristics from YOUR_NAME ##############
                for service in services:
                    for characteristic in service.characteristics():
                        service_characteristic_pair.append((service.uuid(), characteristic.uuid()))
                                                            
                for i, (service_uuid, characteristic) in enumerate(service_characteristic_pair):
                    
                    print(f"{i}: {service_uuid} {characteristic}")
                    if service_uuid == SERVICE_UUID.get('drive'):
                        self.drive_service = service_uuid
                    elif service_uuid == SERVICE_UUID.get('led'):
                        self.led_service = service_uuid
                    if characteristic==CHAR_UUIDS.get("up"):
                        self.up_characteristic = characteristic
                    elif characteristic==CHAR_UUIDS.get("down"):
                        self.down_characteristic = characteristic
                    elif characteristic==CHAR_UUIDS.get("left"):
                        self.left_characteristic = characteristic
                    elif characteristic==CHAR_UUIDS.get("right"):
                        self.right_characteristic = characteristic
                while(True):
                        try:
                            time.sleep(0.3)
                            if (connect_device==False):
                                print("Safely exiting now")
                                self.peripheral.disconnect()
                                return
                            else:
                                if keyboard.is_pressed("UP"):
                                    self.peripheral.write_request(self.drive_service, self.up_characteristic, HIGH)
                                    print("Moving Forward")
                                    self.peripheral.write_request(self.drive_service, self.down_characteristic, LOW)
                                elif keyboard.is_pressed("DOWN"):
                                    self.peripheral.write_request(self.drive_service, self.down_characteristic, HIGH)
                                    print("Moving Backward")
                                    self.peripheral.write_request(self.drive_service, self.up_characteristic, LOW)
                                elif keyboard.is_pressed("LEFT"):
                                    self.peripheral.write_request(self.drive_service, self.left_characteristic, HIGH)
                                    print("Moving Left")
                                    self.peripheral.write_request(self.drive_service, self.right_characteristic, LOW)
                                elif keyboard.is_pressed("RIGHT"):
                                    self.peripheral.write_request(self.drive_service, self.right_characteristic, HIGH)
                                    print("Moving Right")
                                    self.peripheral.write_request(self.drive_service, self.left_characteristic, LOW)
                                elif keyboard.is_pressed("w"):
                                    self.peripheral.write_request(self.led_service, self.right_characteristic, HIGH)
                                    print("LED On")
                                elif keyboard.is_pressed("s"):                                    
                                    self.peripheral.write_request(self.led_service, self.right_characteristic, LOW)
                                    print("LED Off")
                        except KeyboardInterrupt:
                            print("Interrupted by Ctrl-C")
                            self.peripheral.disconnect()
                            print("Safely disconnecting now")
                            return


robot = RobotController(YOUR_ADDRESS)
del robot
