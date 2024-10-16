import tkinter as tk
from tkinter import scrolledtext
from dvl.dvl import Dvl
from dvl.system import OutputData
import numpy as np


class DvlInterface:
    def __init__(self, port='COM5', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.dvl = None

    def connect(self):
        self.dvl = Dvl(self.port, self.baud_rate)
        return self.dvl.is_connected()

    def start_pinging(self):
        if self.dvl:
            if self.dvl.get_setup():
                if not self.dvl.enter_command_mode():
                    return False
                return self.dvl.exit_command_mode()
        return False

    def stop_pinging(self):
        if self.dvl:
            self.dvl.unregister_all_callbacks()
            self.dvl.close()

    def register_callback(self, callback):
        if self.dvl:
            self.dvl.register_ondata_callback(callback)

    def reset_to_defaults(self):
        if self.dvl:
            return self.dvl.reset_to_defaults()
        return False


class DvlApp:
    def __init__(self, master):
        self.master = master
        master.title("Interfaz DVL")

        master.resizable(False, False)

        self.output_area = scrolledtext.ScrolledText(master, width=80, height=20)
        self.output_area.pack(pady=10)

        self.start_button = tk.Button(master, text="Iniciar DVL", command=self.start_dvl)
        self.start_button.pack(side=tk.LEFT, padx=5)

        self.stop_button = tk.Button(master, text="Detener DVL", command=self.stop_dvl)
        self.stop_button.pack(side=tk.RIGHT, padx=5)

        self.dvl_interface = DvlInterface()

    def update_data(self, output_data: OutputData, obj):
        if output_data is not None:
            time = output_data.get_date_time()
            txt = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.output_area.insert(tk.END, f"Datos recibidos: {txt}\n")
            velocities = np.array([output_data.vel_x, output_data.vel_y, output_data.vel_z])
            self.output_area.insert(tk.END, f"Velocidades: {velocities}\n")
            beams = np.array(
                [output_data.range_beam1, output_data.range_beam2, output_data.range_beam3, output_data.range_beam4])
            self.output_area.insert(tk.END, f"Beams: {beams}\n")
            self.output_area.insert(tk.END, f"Coordenadas: {output_data.COORDINATES}\n")
            self.output_area.insert(tk.END, f"Estado de datos: {output_data.status}\n")

            self.output_area.see(tk.END)

    def start_dvl(self):
        if self.dvl_interface.connect():
            if self.dvl_interface.start_pinging():
                self.dvl_interface.register_callback(self.update_data)
                self.output_area.insert(tk.END, "DVL iniciado.\n")
            else:
                self.output_area.insert(tk.END, "Fallo al iniciar el ping.\n")
        else:
            self.output_area.insert(tk.END, f"Fallo al conectar al DVL en {self.dvl_interface.port}.\n")

    def stop_dvl(self):
        self.dvl_interface.stop_pinging()
        self.output_area.insert(tk.END, "DVL detenido.\n")


if __name__ == "__main__":
    root = tk.Tk()
    app = DvlApp(root)
    root.mainloop()