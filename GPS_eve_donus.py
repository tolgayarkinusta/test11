from MainSystem import USVController
import math

class GPSevedonus:
    def __init__(self, mavlink_connection):
        self.controller = mavlink_connection
        self.first_position = None
        self.current_position = None

    def get_current_position(self):
        #anlık GPS konumu alınır
        msg = self.controller.recv_match(type='GLOBAL_POSITION_INT', timeout=1.0)
        if msg:
            enlem = msg.lat / 1e7
            boylam = msg.lon / 1e7
            return enlem, boylam
        return None, None

    def save_initial_position(self):
        #başlangıç noktasını kaydet
        self.first_position = self.get_current_position()
        if self.first_position:
            print(f"Başlangıç noktası kaydedildi: Enlem: {self.first_position[0]}, Boylam: {self.first_position[1]}")
        else:
            print("GPS verileri alınamıyor.")

    def update_current_position(self):
        #anlık GPS konumu güncelleme
        self.current_position = self.get_current_position()
        if self.current_position:
            print(f"Güncel konum: Enlem: {self.current_position[0]}, Boylam: {self.current_position[1]}")
        else:
            print("GPS verileri alınamıyor.")

    def calculate_heading(self):
        #ilk ve son GPS konumları arasındaki heading açısını hesapla
        if not self.first_position or not self.current_position:
            print("Başlangıç veya Bitiş konumu eksik, heading hesaplanamıyor.")
            return None

        enlem1, boylam1 = self.first_position
        enlem2, boylam2 = self.current_position

        #enlem ve boylamları radyan cinsine çevirme
        enlem1_radyan = math.radians(enlem1)
        enlem2_radyan = math.radians(enlem2)
        delta_boylam = math.radians(boylam2 - boylam1)

        #heading hesaplama
        x = math.sin(delta_boylam) * math.cos(enlem2_radyan)
        y = math.cos(enlem1_radyan) * math.sin(enlem2_radyan) - math.sin(enlem1_radyan) * math.cos(enlem2_radyan) * math.cos(delta_boylam)
        initial_kerteriz = math.atan2(x, y)

        #açıyı dereceye çevirme
        initial_kerteriz = math.degrees(initial_kerteriz)
        pusula_kerteriz = (initial_kerteriz + 360) % 360
        return pusula_kerteriz

    def give_desired_heading(self):
        kerteriz = self.calculate_heading()
        print(f"Evin kerterizi: {kerteriz}")




