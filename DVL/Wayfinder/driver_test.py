from dvl.dvl import Dvl
from dvl.system import OutputData
import math
import numpy as np

def update_data(output_data: OutputData, obj):
    """Prints data time to screen
    """
    del obj
    if output_data is not None:
        time = output_data.get_date_time()
        txt = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print("Got data {0}".format(txt))
        velocities = np.array([output_data.vel_x, output_data.vel_y, output_data.vel_z])
        print("velocities :" , velocities)
        beams = np.array([output_data.range_beam1, output_data.range_beam2, output_data.range_beam3, output_data.range_beam4])
        print("beams" , beams)
        print("coordinates: ", output_data.COORDINATES)
        print("coordinate system:", output_data.coordinate_system)
        print("fw major version", output_data.fw_major_version)
        print("fw minor version:", output_data.fw_minor_version)
        print("patch version:", output_data.fw_patch_version)
        print("build version:", output_data.fw_build_version)
        print("mean range:", output_data.mean_range)
        print("data status:", output_data.status)
        print("input voltage:", output_data.voltage)
        print("transmit voltage:", output_data.transmit_voltage)
        print("current:", output_data.current)
        print("serial number:", output_data.serial_number)

if __name__ == "__main__":
    PORT = 'COM5'

    # Connect to serial port
    with Dvl(PORT, 115200) as DVL:

        if DVL.is_connected():

            # Get user system setup
            if DVL.get_setup():
                # Print setup
                print (DVL.system_setup)

            # Stop pinging
            if not DVL.enter_command_mode():
                print("Failed to stop pinging")
            # Enter command mode
            # Reset to factory defaults (requires Wayfinder to be in 'command mode')
            if not DVL.reset_to_defaults():
                print("Failed to reset to factory defaults")

            # Register callback function
            DVL.register_ondata_callback(update_data)

            # Start pinging
            if not DVL.exit_command_mode():
                print("Failed to start pinging")

            # Blocking call to wait for key pressed to end program
            KEY = input("Press Enter to stop\n")

        else:
            print("Failed to open {0} - make sure it is not used by any other program".format(PORT))

        # Unregister
        DVL.unregister_all_callbacks()
