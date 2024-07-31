// Seccion de importe de librerias
// Librerias de ros
#include "rclcpp/rclcpp.hpp"
// Librerias de mensajes
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Definicion de using para la seleccion de parametros de los callbacks
using std::placeholders::_1;

// Creacion de objeto nodo (clase)
class ClickPointToPose : public rclcpp::Node
{
    public:
    // Seccion publica de ejecucion de clase de nodo
    // Primero se realiza la inicializacion de la clase, o bien el constructor
        ClickPointToPose(const std::string & name)
        : Node(name)
        {
            // En este constructor se crea un subscriptor y un publicador que seran los que interactuen con el punto
            sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point",
                10,
                std::bind(&ClickPointToPose::callback_clicked,
                this,
                _1
            ));
            // El siguiente corresponde al publicador con el cual se establece una nueva posicion para la ejecucion del pose to goal
            pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal_update",
                10
            );
        }

    // En la seccion de private tendremos acceso a las funciones y variables 
    private:
        // Funcion para la realimentacion del click de posicion nueva
        void callback_clicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) const
        {   
            //  RCLCPP_INFO(get_logger(), "Hola, recibimos un valor");
            // Inicializacion de pose objetivo
            geometry_msgs::msg::PoseStamped pose;
            // Se extrae el valor de pose ingresada y se asigna a los valores de position de la variable de mensaje
            pose.header = msg->header;
            pose.pose.position = msg->point;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;

            pub_->publish(pose);
        }

    private:
        // Finalmente se inicializan las variables de punteros para el subscriptor y el publicador
        // El subscriptor se hace un el mensaje de interfaz PointStamped
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
        // El publicador se hace con el pose Stamped
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

// Se define tambien la funcion main de ejecucion de main del nodo
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto clicked_point_to_pose_node = std::make_shared<ClickPointToPose>("clicked_point_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(clicked_point_to_pose_node);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}