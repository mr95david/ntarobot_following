/*  El siguiente programa sigue las funciones de lectura de un punto dinamico
    este punto se va actualizando cada cierto tiempo y va almacenando las posiciones
    objetivo, concorde estas posiciones vayan cambiando se realizara la actualzacion
    del punto deseado */
// Seccion de importe de librerias de ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Seguido a esto se incluyen las librerias de mensajes e interfaces necesarios
// Mensaje Point_stamped, lee un punto dado en el mapa
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Seccion de librerias utilitarias
#include <memory>
#include <iostream>

// Seccion de declracion de usings:: Esto corresponde a los placeholders necesarios para los callbacks, 
// En caso un callback reciba mas de 2 parametros de entrada sera necesario usaro un _2
using std::placeholders::_1;
using std::placeholders::_2;
// Se incluye el using que permite comandos de tiempo como 1s, 10s para ejecucion rapida
using namespace std::chrono_literals;

// Nodo de ROS2
class HumanFollowNode :public rclcpp::Node
{
    // Seccion de ejecucion publica del nodo
    public:
        // Se aagrega el uso de actions definidos por variables especificas
        // Action para la navegacion hasta un punto especifico
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        // Action para el manejo de estado de ejecucion del desplazamiento hasta el objetivo
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        // Constructor de nodo
        HumanFollowNode(const std::string & name)
        : Node(name)
        {
            // Seccion de creacion de subscriptores
            sub_point_S = create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point",
                10,
                std::bind(&HumanFollowNode::pointS_Callback,
                this,
                _1
            ));

            // Seccion de creadores de publicadores
            pub_pose_leida = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/ntarobot/human_point",
                10
            );
        }
    // Seccion de instancias privadas de la clase
    private:
        // INICIO: Seccion de creacion de funciones utilitarias y callbacks necesarias
        void pointS_Callback(
            const geometry_msgs::msg::PointStamped::SharedPtr msg
        );
        // Funcion de validacion de estado actual de accion de navegacion
        void feedback_callback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback
        );
        // Funcion de estado final de accion de navegacion
        void result_callback(
            const GoalHandleNavigateToPose::WrappedResult & result
        );
        // Funcion para llamado de serviro y envio de posicion objetivo
        void call_server(double x, double y);
        // FINAL

        // Inicializacion/declracion de variables
        // INICIO: Seccion de inicializacion de subscriptores
        // Subscriptor de valor de punto en el espacio
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_S;
        // FINAL

        // INICIO: Seccion de creacion de publicadores
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_leida;
        // FINAL

        // INICIO: Seccion de creacion de variables de instancia de la calse
        // Primero se crea la variable que corresponde al cliente del servidor de navegacion
        rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
        bool validation_Function = false;
        // FINAL
};

// La siguiente funcion recibe la posicion dada por el punto solicitado en rviz
// y lo publica como un nodo
void HumanFollowNode::pointS_Callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Creacion de variable de pose 
    geometry_msgs::msg::PoseStamped pose;

    // Asignacion de valores de mensaje obtenidos por la lectura de pose
    pose.header = msg->header;
    // Datos de posicion
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;
    // Valor de orientacion estimada
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    // Publicacion de pose leida
    pub_pose_leida->publish(pose);

    // ejecccion se solicitud de servicio
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 1");
    if (!validation_Function){
        call_server(msg->point.x, msg->point.y);
    };
}

// La siguiente funcion realiza el llamado al servidor de manera que publica la 
// navegacion hasta el punto especificado.
void HumanFollowNode::call_server(double x, double y) {
    this->validation_Function = true;
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 2");
    // Se usa la variable que corresponde al cliente creada anteriormente
    navigate_to_pose_client_ptr_ = 
        rclcpp_action::create_client<NavigateToPose>(
            shared_from_this(), // Se comparte para el mismo objeto
            "navigate_to_pose" // Se usa la funcion navigate_to_pose
        );
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 3");
    // Se valida si el servidor se encuentra disponible
    if (!this->navigate_to_pose_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        // Esta validacion se activara si el servidor dura mas de 10 sg en ejecutarse
        RCLCPP_ERROR(get_logger(), "Action server  no disponible después de esperar");
        return;
    }
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 4");
    // Ahora se inicializa la variable o objeto correspondiente al punto de navegacion objetivo
    auto goal_msg = NavigateToPose::Goal();

    // Se definen las configuraciones con las que se publicara el objetivo
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    // Seguido a esto es necesario realizar la variable que hara el envio de las opciones
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 5");
    // a la variable de envio de objetivo, se agrega la funcion de callback feedback, 
    // Esta recive el estado actual en el que se esta ejecutando la accion
    send_goal_options.feedback_callback = std::bind(
        &HumanFollowNode::feedback_callback, // Esta corresponde a la funcion de callback de feedback que valida el estado de laccion de navegacion 
        this, // Funcion propia del objeto, en este caso el nodo 
        _1,// Primera placa de valor recibido desde el callback
        _2 // Segunda plaza recibida de la funcion de callback
    );
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 6");
    // De la misma manera se crea un feedback para el proceso de ejecucion de la funcion de navegacion establecido
    // Este corresponde a un resultfeedback, dado como un sistema que valida el estado final en el que se encuentra la ejecucion de la accion
    send_goal_options.result_callback = std::bind(
        &HumanFollowNode::result_callback, this, _1
    );
    // RCLCPP_ERROR(get_logger(), "Ejecutando servicio - puerta 7");
    // Se crea una variable asincrona que realiza el manejo del futuro objetivo
    // De manera formal esta corresponde a la ejecucion del action de navegacion
    auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(
            goal_msg, // Esta es la posicion objetivo
            send_goal_options // Esta es la respuesta de feedbeck durante la ejecucion de la accion
        );
};

void HumanFollowNode::feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    // La funcion responde con el valor restante hasta la posicion deseada
    RCLCPP_INFO(get_logger(), 
        "Distancia restante = %f",
        feedback->distance_remaining
    );
}

void HumanFollowNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    // Esta funcion responde con la el estado final de laccion solicitada
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Success!!!");
            this->validation_Function = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "El objetivo fue abortado");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "El objetivo fue cancelado");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "Código de resultado desconocido");
            return;
    }
}

// Main necesario para la ejecucion general del nodo
int main(int argc, char * argv[])
{
    // Inicializacion de ros2
    rclcpp::init(argc, argv);
    // Declaracion de variable nodo con la clase creada
    auto human_follow_node = std::make_shared<HumanFollowNode>(
            "human_follow_node"
        );
    // Spin de nodo creado
    rclcpp::spin(human_follow_node);

    // Apagado de nodo
    rclcpp::shutdown();
    // Return final
    return 0;
}
