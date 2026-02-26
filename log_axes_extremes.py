import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys

class AxesLimitLogger(Node):
    def __init__(self):
        super().__init__('axes_limit_logger')
        self.subscription = self.create_subscription(Joy, '/ros_robot_controller/joy', self.joy_callback, 10)
        self.axes_min = None
        self.axes_max = None
        print("\n\033[92m[ LOGGER EN LINEA ] - Mueve todos los joysticks, D-Pad y Gatillos a sus extemos MAXIMOS y MINIMOS...\033[0m")
        print("Para terminar y ver el resumen, presiona Ctrl+C.\n")

    def joy_callback(self, msg):
        if self.axes_min is None:
            self.axes_min = list(msg.axes)
            self.axes_max = list(msg.axes)
            return

        for i, val in enumerate(msg.axes):
            if val < self.axes_min[i]:
                self.axes_min[i] = val
                print(f"EJE {i} -> NUEVO MINIMO: {val}")
            
            if val > self.axes_max[i]:
                self.axes_max[i] = val
                print(f"EJE {i} -> NUEVO MAXIMO: {val}")

    def print_summary(self):
        print("\n\n" + "="*50)
        print("          RESUMEN DE LIMITES DE EJES")
        print("="*50)
        if self.axes_min is None:
            print("No se recibieron datos.")
            return
            
        for i in range(len(self.axes_min)):
            print(f"Eje {i}: Min = \033[93m{self.axes_min[i]:.2f}\033[0m, Max = \033[93m{self.axes_max[i]:.2f}\033[0m")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    logger = AxesLimitLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.print_summary()
    
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
