// Libraries section
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// Msg library section
#include <std_msgs/msg/string.hpp>
// Untils library section
#include <memory>
#include <thread>

// Estandard declarations
using std::placeholders::_1;
using namespace std::chrono_literals;

// class of cpp library
class SimpleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode{
    public: 
        explicit SimpleLifeCycleNode(const std::string & node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
            {
                
            }
        
        // Esta funcion considera establecer la configuracion del ciclo de vida de un nodo especifico
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &){
            // Creacion de subscriptor
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleLifeCycleNode::msgCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Nodo Lifecycle configurado correctamente"); // Visualizacion de mensaje usando getlogger
            // Validacion de retorno correcto del mensaje indicado
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // Esta funcion considera apagar un nodo en cuanto este ya no se este utilizando
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &){
            // reset de subscriptor
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Apagando nodo de prueba de lifecycle"); // Visualizacion de mensaje usando getlogger
            // Validacion de retorno correcto del mensaje indicado
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // Ahora para realizar un cleanup
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &){
            // Creacion de subscriptor
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Clean up de nodo de prueba de lifecycle"); // Visualizacion de mensaje usando getlogger
            // Validacion de retorno correcto del mensaje indicado
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // Ahora para realizar un active
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state){
            // Creacion de state subscriptor
            LifecycleNode::on_activate(state);
            RCLCPP_INFO(get_logger(), "Se activa el nodo de lifecycle"); // Visualizacion de mensaje usando getlogger
            // Apagado de la funcion por un tiempo especifico
            std::this_thread::sleep_for(2s);
            // Validacion de retorno correcto del mensaje indicado
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // Ahora para realizar un deactive
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state){
            // Creacion de subscriptor
            LifecycleNode::on_deactivate(state);
            RCLCPP_INFO(get_logger(), "se deactiva la prueba de lifecycle"); // Visualizacion de mensaje usando getlogger
            // Validacion de retorno correcto del mensaje indicado
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // Definir la funcion de msgCallback
        void msgCallback(const std_msgs::msg::String & msg){
            auto state = get_current_state();
            if (state.label() == "active"){
                RCLCPP_INFO_STREAM(get_logger(), "Lifecyclo node ejecutado" << msg.data.c_str());
            }
        }

    private:
        // Subscriptor creado para verificar el estado de comunicacion del nodo
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

//Seccion main de la funcion
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleLifeCycleNode> simple_lifecycleNode = std::make_shared<SimpleLifeCycleNode>("simple_lifecycle_node");
    ste.add_node(simple_lifecycleNode->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}