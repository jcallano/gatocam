import time
try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1309
except ImportError:
    print("Por favor instala las librerias luma.oled y luma.core en tu entorno virtual:")
    print("source /home/jcallano/ros2_ws/.venv/bin/activate")
    print("pip install luma.oled luma.core")
    import sys
    sys.exit(1)

def test_oled_on_bus(bus_num, addr):
    try:
        # Intenta inicializar el bus I2C
        serial = i2c(port=bus_num, address=addr)
        device = ssd1309(serial)
        print(f"¡CONECTADO! Bus: {bus_num}, Dirección: {hex(addr)}")
        
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline="white", fill="black")
            draw.text((10, 20), "OLED SSD1309 OK", fill="white")
            draw.text((10, 35), f"Bus: {bus_num} Addr: {hex(addr)}", fill="white")
            draw.text((10, 50), "Probando...", fill="white")
            
        time.sleep(5)
        return True
    except Exception as e:
        # Silenciamos errores de "Device not found" para no ensuciar la salida
        if "No such device or address" not in str(e) and "not found" not in str(e):
            print(f"Error en bus {bus_num} addr {hex(addr)}: {e}")
        return False

def main():
    print("Iniciando escaneo profundo de pantalla OLED SSD1309...")
    
    # Buses detectados en tu sistema + el posible bus 5
    buses_a_probar = [5, 2, 6, 7, 0] 
    # Direcciones comunes para OLED (0x3C es la mas comun, 0x3D ocurre en algunas de 1.54)
    direcciones = [0x3C, 0x3D]
    
    exito = False
    for bus in buses_a_probar:
        for addr in direcciones:
            print(f"Probando Bus {bus} en dirección {hex(addr)}...", end="\r")
            if test_oled_on_bus(bus, addr):
                exito = True
                break
        if exito: break
            
    if not exito:
        print("\n\nNo se detectó ninguna pantalla en los buses 0, 2, 6, 7.")
        print("-" * 40)
        print("Sugerencias de hardware:")
        print("1. Revisa las conexiones VCC (3.3V) y GND.")
        print("2. Revisa que SDA y SCL no estén invertidos.")
        print("3. ¿En qué pines físicos (números de pin) has conectado la pantalla?")
        print("   I2C-7 suele estar en los pines 3 (SDA) y 5 (SCL).")
        print("   I2C-2 suele estar en los pines 13 (SDA) y 15 (SCL).")
        print("-" * 40)

if __name__ == "__main__":
    main()
