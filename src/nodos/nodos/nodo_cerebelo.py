import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String # Tipos de mensajes

# Clase para el cerebelo
class nodoCerebelo(Node):
    def __init__(self):
        # Inicializar nodo con nombre
        super().__init__('nodo_cerebelo')
        # Crear suscriptor. Escucha al topic
        # En el topic, el nodo nociceptor publica los valores
        self.subscription = self.create_subscription(
            Float32,
            'nociceptor_data',
            self.respuesta_nociceptor,
            10)
        # Crear publicador que envia comandos de movimiento al topic
        # Nombre del topic: movimientos_robot
        self.publisher_ = self.create_publisher(String, 'movimientos_robot', 10)
        self.get_logger().info("Nodo Cerebelo iniciado")

    # Funcion que se ejecuta cada que recibe un mensaje en el topic: nociceptor_data
    def respuesta_nociceptor(self, msg):
        # Extraer el valor de nociceptor que se envio
        nociceptor_value = msg.data
        self.get_logger().info(f"Valor recibido del nociceptor: {nociceptor_value}")
        # Llamar a la funcion para saber como se debe mover 
        self.control_robot(nociceptor_value)

    # Funcion para saber que hacer segun el valor que le llego  
    def control_robot(self, nociceptor_value):
        if nociceptor_value > 0.7:
            adjusted_command = f"Cambiar posicion. Valor detectado: {nociceptor_value}"
            self.comando_robot(adjusted_command)
        else:
            self.comando_robot("Todo bien. Sigue asi")

    # Funcion que recibe el comando y lo publica en el topic: movimientos_robot 
    def comando_robot(self, command):
        # Crear mensaje
        msg = String()
        # Se asigna el comando al campo data
        msg.data = command
        # Publicar el mendaje con el comando
        self.publisher_.publish(msg)
        self.get_logger().info(f"Comando enviado: {command}")

def main(args=None):
    # Conexion con el entorno de ROS
    rclpy.init(args=args)
    # Crear la instancia de nodo cerebelo 
    cerebelo_node = nodoCerebelo()
    try:
        rclpy.spin(cerebelo_node)
    except KeyboardInterrupt:
        pass
    finally:
        cerebelo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()