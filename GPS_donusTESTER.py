print("Importing libraries...")

from MainSystem import USVController
from pymavlink import mavutil

print("Libraries imported!")

print("Connecting to the vehicle through IP address...")
controller = USVController('udpin:0.0.0.0:14550')
print("Vehicle connected!")

print("Arming vehicle...")
controller.arm_vehicle()
print("Vehicle armed!")

print("Setting mode...")
controller.set_mode("MANUAL")
print("Mode set!")

#GPSevedonus classı kullanılarak bi instance variable oluşturma(mavlink bağlantısı parametre olarak verilir)
navigator = USVController(controller)

#başlangıç konumu al ve sakla
navigator.save_initial_position()

#aracın hareket ettiğini varsayıyoruz, bi süre sonra...
input("Devam etmek için ENTER'a basın...")

#anlık pozisyonu güncelle
navigator.update_current_position()

#ilk konumdan son konuma kerteriz açısını hesapla
heading= navigator.calculate_heading()
if heading is not None:
    print(f"Başlangıç noktasına dönebilmek için gerekli olan kerteriz: {heading} derece")