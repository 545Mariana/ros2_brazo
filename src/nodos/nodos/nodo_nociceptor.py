import rclpy #Crear nodos
from rclpy.node import Node
from std_msgs.msg import Float32
from random import uniform

# Esta clase representa el nodo
class nodoNociceptor(Node):
    def __init__(self):
        # El nombre identifica el nodo dentro del sistema
        super().__init__('nodo_nociceptor')
        # Crear publicador asociado al topic 
        # Nombre del topic: nociceptor_data
        self.publisher_ = self.create_publisher(Float32, 'nociceptor_data', 10)
        # Creamos un timer que genera un valor cada segundo
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("Nodo Nociceptor iniciado")

    # Funcion para generar el valor aleatoria y publicarlo en 
    # el topic: nociceptor_data
    def timer_callback(self):
        # Genera el valor entre 0 - 1
        nociceptor_value = uniform(0, 1)
        if nociceptor_value > 0.7:
            self.get_logger().warn(f"Valor detectado: {nociceptor_value}. Dolor detectado")
        else:
            self.get_logger().warn(f"Valor detectado: {nociceptor_value}. Todo bien")
        # Publicar el valor en el topic
        # Crear el mensaje para almacenar el valor del nociceptor
        msg = Float32()
        # Asigna el valor al campo data
        msg.data = nociceptor_value
        # Publica el mensaje en el topic nociceptor_data
        self.publisher_.publish(msg)


def main(args=None):
    # Conexion con el entorno de ROS
    rclpy.init(args=args)
    # Instancia del nodoNociceptor
    nociceptor_node = nodoNociceptor()
    try:
        rclpy.spin(nociceptor_node)
    except KeyboardInterrupt:
        pass
    finally:
        nociceptor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()