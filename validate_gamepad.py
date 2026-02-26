import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import time
import termios
import tty

# Mapeo a validar (Basado en la documentación)
MAPPING = [
    {"name": "Boton A (Cruz)", "type": "button", "index": 0, "val": 1},
    {"name": "Boton B (Circulo)", "type": "button", "index": 1, "val": 1},
    {"name": "Boton X (Cuadrado)", "type": "button", "index": 3, "val": 1},
    {"name": "Boton Y (Triangulo)", "type": "button", "index": 4, "val": 1},
    {"name": "Boton L1", "type": "button", "index": 6, "val": 1},
    {"name": "Boton R1", "type": "button", "index": 7, "val": 1},
    {"name": "Boton Select", "type": "button", "index": 10, "val": 1},
    {"name": "Boton Start", "type": "button", "index": 11, "val": 1},
    {"name": "Joystick Izquierdo ->", "type": "axis", "index": 0, "val": -1.0},
    {"name": "Joystick Izquierdo <-", "type": "axis", "index": 0, "val": 1.0},
    {"name": "Joystick Izquierdo Arriba", "type": "axis", "index": 1, "val": 1.0},
    {"name": "Joystick Izquierdo Abajo", "type": "axis", "index": 1, "val": -1.0},
    {"name": "Joystick Derecho ->", "type": "axis", "index": 2, "val": -1.0},
    {"name": "Joystick Derecho <-", "type": "axis", "index": 2, "val": 1.0},
    {"name": "Joystick Derecho Arriba", "type": "axis", "index": 3, "val": 1.0},
    {"name": "Joystick Derecho Abajo", "type": "axis", "index": 3, "val": -1.0},
    {"name": "Gatillo R2 (Presionado)", "type": "axis", "index": 4, "val": -1.0},
    {"name": "Gatillo L2 (Presionado)", "type": "axis", "index": 5, "val": -1.0},
    {"name": "D-Pad ->", "type": "axis", "index": 6, "val": -1.0},
    {"name": "D-Pad <-", "type": "axis", "index": 6, "val": 1.0},
    {"name": "D-Pad Arriba", "type": "axis", "index": 7, "val": 1.0},
    {"name": "D-Pad Abajo", "type": "axis", "index": 7, "val": -1.0},
]

class InteractiveValidator(Node):
    def __init__(self):
        super().__init__('joy_interactive_validator')
        self.subscription = self.create_subscription(Joy, '/ros_robot_controller/joy', self.joy_callback, 10)
        self.current_step = 0
        self.wait_for_release = False
        self.initial_axes = None
        
        print("\n" + "="*50)
        print("🎮 INICIANDO VALIDACION INTERACTIVA DE GAMEPAD 🎮")
        print("="*50)
        print("Por favor, asegurate de no tocar ningun control.")
        print("Calibrando posicion inicial...")

    def print_current_instruction(self):
        if self.current_step < len(MAPPING):
            control = MAPPING[self.current_step]
            print(f"\n[{self.current_step + 1}/{len(MAPPING)}] -> Presiona y manten: \033[93m{control['name']}\033[0m")
        else:
            print("\n" + "="*50)
            print("\033[92m✅ VALIDACION COMPLETADA. El mapeo es 100% correcto.\033[0m")
            print("="*50)
            sys.exit(0)

    def joy_callback(self, msg):
        # 1. Calibracion inicial silenciosa
        if self.initial_axes is None:
            self.initial_axes = list(msg.axes)
            time.sleep(1)
            print("Calibracion exitosa. ¡Empecemos!")
            self.print_current_instruction()
            return

        if self.current_step >= len(MAPPING):
            return

        target = MAPPING[self.current_step]

        # 2. Esperar a que el usuario suelte todo (anti-rebote)
        is_pressed = False
        
        # Check buttons
        for btn in msg.buttons:
            if btn == 1: is_pressed = True
        
        # Check axes deviation from initial
        for i, ax in enumerate(msg.axes):
            if abs(ax - self.initial_axes[i]) > 0.5: # 50% threshold for axes
                is_pressed = True
                
        if self.wait_for_release:
            if not is_pressed:
                self.wait_for_release = False
                self.current_step += 1
                self.print_current_instruction()
            return

        # 3. Validar el input actual
        success = False
        if target["type"] == "button":
            if msg.buttons[target["index"]] == target["val"]:
                success = True
        elif target["type"] == "axis":
            current_val = msg.axes[target["index"]]
            expected = target["val"]
            
            # Direccion correcta y fuerza > 50%
            if (expected > 0 and current_val > 0.5) or (expected < 0 and current_val < -0.5):
                success = True

        if success:
            print(f"\033[92m  [OK] {target['name']} detectado correctamente!\033[0m")
            print("  -> (Por favor, suelta el control)")
            self.wait_for_release = True
            
def main(args=None):
    rclpy.init(args=args)
    validator = InteractiveValidator()
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("\nValidacion cancelada por el usuario.")
    
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
