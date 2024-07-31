// Seccion de librerias
// Librerias relacionadas a ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
// Seguirian las librerias de uso utilitario
#include <memory>
// Uso de libreria de action para navigate_to_pose
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Declracion de los using
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Seccion de creacion de clase de nodo
class FollowNode : public rclcpp::Node
{
    // Seccion de declracion de valores publicos de clase
    public:
        // Uso de action para navegacion hasta pose
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        // Action que valida la plenitud de la tarea
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        // Inicialiacion de nodo general
        FollowNode() : Node("prueba_follow")
        {
            // En este caso no estamos agregando nada de variables de instancia al nodo por el momento
        }

        // Se agrega una funcion vacion que funciona como el llamado al servicio de ejecucion de navtogoal
        /* Como parametros de entrada este recibe:
        pos x : double
        pos y: double
        "str" bt: Representa el tipo de behavieur tree usado*/
        void call_server(double x, double y, const std::string & bt)
        {
            // El siguiente corresponde al valor que se usara del action
            navigate_to_pose_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            shared_from_this(), "navigate_to_pose");
            // Valida la espera de 10 segundos hasta el que action este disponible
            if (!this->navigate_to_pose_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
                return;
            }
            // Define el objetivo a donde el robot debe avanzar como un goal
            auto goal_msg = NavigateToPose::Goal();
            // A este se le atribuye un tipo de ejecucion de behaivour tree
            goal_msg.behavior_tree = bt;
            // Se realizan las descripciones generales del action
            goal_msg.pose.header.stamp = now();
            goal_msg.pose.header.frame_id = "map";

            goal_msg.pose.pose.position.x = x;
            goal_msg.pose.pose.position.y = y;
            goal_msg.pose.pose.orientation.w = 1.0;

            // Se declara el envio del objetivo omo una variable
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            // Se envia el feedback del obketivo creado previamente,
            // Este feedback consiste en la recepcion del client por medio de 2 entradas
            send_goal_options.feedback_callback =
            std::bind(&FollowNode::feedback_callback, this, _1, _2);
            // de ça misma manera se obtiene el callback del resultado del action llamado
            send_goal_options.result_callback =
            std::bind(&FollowNode::result_callback, this, _1);
            // De esta manera se crea la variable que obtiene la funcion de objetivo futuro
            auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(
            goal_msg, send_goal_options);
            // Validamos si ya se llego al punto final, se usa un if para establecer si el objetivo se logro o no
            if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "send_goal failed");
                return;
            }
            // Ahora la validacion igual para rejectar el objetivo pedido dadas las condiciones de movimiento del robot
            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(
                    get_logger(), "ExecutorClient: Execution was rejected by the action server");
                return;
            }
        }
        // 
    private:
        // Seccion de declaracion de variables y nombre de funciones privadas de la clase
        rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
        // Incluye el feedback del proceso del action llamado
        void feedback_callback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            // Esta funcion tiene como parametros de entrada el puntero relacionado al action 
            // La variable donde se albergara el feedback obtenido
            // Esta variable corresponde a la validacion visual del feedback
            RCLCPP_INFO(get_logger(), "Distance remaininf = %f",
            feedback->distance_remaining);
        }

        // Esta funcion entonces comprende el callback correspondiente al valor resultante de la navegacion solicitada
        void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
        {
            // Esta funcion almacena los posibles errores resultantes de la solicitud realizada al action
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Success!!!");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    return;
            }
        }
};

// Continua la funcion main para la ejecucion del nodo
int main(int argc, char **argv)
{
    // Inicializacion de libreria de ros
    rclcpp::init(argc, argv);
    // Declaracion de nodos
    auto node = std::make_shared<FollowNode>();
    // Se se ingresan menos de 3 argumentos
    if (argc < 3) {
        std::cerr << "use " + std::string(argv[0]) + " x y bt" << std::endl;
        return 1;
    }
    //
    std::string bt;
    if  (argc == 4) {
        bt = std::string(argv[3]);
    }

    node->call_server(
        std::stod(argv[1]), std::stod(argv[2]), bt);

    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
