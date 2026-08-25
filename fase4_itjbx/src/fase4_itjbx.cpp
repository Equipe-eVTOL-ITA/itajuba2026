#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msgs/msg/lane_direction.hpp"
#include "custom_msgs/msg/base_circle.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/return_home_state.hpp"
#include "stdstates/landing_state.hpp"

// Estados desta missao: segue a linha azul ate avistar a mangueira vermelha
// entre os dois postes, e se alinha com ela.
#include "fase4_itjbx/states/search_line_state.hpp"
#include "fase4_itjbx/states/follow_line_state.hpp"
#include "fase4_itjbx/states/approach_red_line_state.hpp"
#include "fase4_itjbx/states/align_red_line_state.hpp"
#include "fase4_itjbx/states/open_gripper_state.hpp"

/**
 * @brief Maquina de estados da missao fase4_itjbx: segue uma linha azul no chao
 * (com uma base circular no ponto de partida) ate avistar a mangueira vermelha
 * entre os dois postes e se alinhar com ela.
 */
class Fase4ItjbxFSM : public fsm::FSM {
public:
    Fase4ItjbxFSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // O drone fica na blackboard: todo estado o acessa por aqui.
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // Parametros do ROS 2 (vindos do YAML) viram entradas da blackboard.
        for (const auto &[key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // Dados de visao -- valor inicial ate chegar a primeira mensagem de /lane_detection
        this->blackboard_set<bool>("line_detected", false);
        this->blackboard_set<float>("line_theta", 0.0f);
        this->blackboard_set<int>("line_x_centroid", 0);
        this->blackboard_set<int>("line_y_centroid", 0);
        this->blackboard_set<int>("line_area", 0);
        this->blackboard_set<bool>("line_is_circle", false);
        this->blackboard_set<bool>("line_circle_exit_valid", false);
        this->blackboard_set<float>("line_circle_exit_theta", 0.0f);
        // Ultimo theta com a linha realmente detectada (line_theta zera quando lost=true,
        // entao isso e o que sobra pra SEARCH_LINE saber pra que lado retomar a busca).
        this->blackboard_set<float>("last_seen_line_theta", 0.0f);

        // Mangueira vermelha (/red_line_detection, red_line_detector) -- valor inicial
        // ate chegar a primeira mensagem. Precisa estar aqui mesmo antes de FOLLOW_LINE
        // checar isso, porque o no' de visao roda desde o inicio da missao.
        this->blackboard_set<bool>("red_line_detected", false);
        this->blackboard_set<float>("red_line_theta", 0.0f);
        this->blackboard_set<int>("red_line_x_centroid", 0);
        this->blackboard_set<int>("red_line_y_centroid", 0);

        // Posicao de casa pro ReturnHomeState: origem do frame local NED, onde
        // o drone armou/decolou.
        this->blackboard_set<Eigen::Vector3d>("home_position", Eigen::Vector3d(0.0, 0.0, 0.0));

        // ========================= ESTADOS =========================
        this->add_state("ARMING",            std::make_unique<ArmingState>());
        this->add_state("TAKEOFF",           std::make_unique<TakeoffState>());
        this->add_state("SEARCH_LINE",       std::make_unique<SearchLineState>());
        this->add_state("FOLLOW_LINE",       std::make_unique<FollowLineState>());
        this->add_state("APPROACH_RED_LINE", std::make_unique<ApproachRedLineState>());
        this->add_state("ALIGN_RED_LINE",    std::make_unique<AlignRedLineState>());
        this->add_state("OPEN_GRIPPER",      std::make_unique<OpenGripperState>());
        this->add_state("RETURN_TO_BASE",    std::make_unique<ReturnHomeState>());
        this->add_state("LANDING",           std::make_unique<LandingState>());

        // ======================= TRANSICOES ========================
        // Cada linha e: {outcome retornado pelo estado, proximo estado}.
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "SEARCH_LINE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_LINE", {
            {"LINE FOUND", "FOLLOW_LINE"},
            {"TIMEOUT",    "LANDING"},
            {"ERROR",      "ERROR"}
        });

        this->add_transitions("FOLLOW_LINE", {
            {"LINE LOST",      "SEARCH_LINE"},
            {"RED LINE FOUND", "APPROACH_RED_LINE"},
            {"TIMEOUT",        "LANDING"},
            {"ERROR",          "ERROR"}
        });

        this->add_transitions("APPROACH_RED_LINE", {
            {"ABOVE LINE", "ALIGN_RED_LINE"},
            // Volta pra base em vez de pousar em cima dos postes -- mesmo raciocinio
            // do TIMEOUT de ALIGN_RED_LINE abaixo.
            {"TIMEOUT",    "RETURN_TO_BASE"},
            {"ERROR",      "ERROR"}
        });

        this->add_transitions("ALIGN_RED_LINE", {
            {"ALIGNED", "OPEN_GRIPPER"},
            {"TIMEOUT", "RETURN_TO_BASE"},
            {"ERROR",   "ERROR"}
        });

        this->add_transitions("OPEN_GRIPPER", {
            {"GRIPPER OPENED", "RETURN_TO_BASE"},
            {"ERROR",           "ERROR"}
        });

        this->add_transitions("RETURN_TO_BASE", {
            {"AT HOME", "LANDING"},
            {"ERROR",   "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

/**
 * @brief No ROS 2 que executa a FSM da missao fase4_itjbx.
 *
 * Declara os parametros (sobrescritos pelo YAML no launch), monta a FSM e a
 * executa a 20 Hz. Assina /lane_detection (lane_detector) e /base_circle
 * (circle_detector) e escreve os dois na blackboard -- os estados so leem de la.
 */
class Fase4ItjbxNode : public rclcpp::Node {
public:
    explicit Fase4ItjbxNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("fase4_itjbx_node"), drone_(drone) {

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e trocar
        // de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Decolagem (lidos pelo TakeoffState)
            {"takeoff_height",          -2.0},   // metros, NED: negativo e para cima
            {"max_vertical_velocity",    1.2},
            {"position_tolerance",       0.15},

            // Pouso (lidos pelo LandingState)
            {"landing_velocity_max",     0.5},
            {"landing_velocity_min",     0.15},
            {"max_base_height",          0.5},
            {"landing_timeout",          5.0},

            // Busca / seguimento de linha
            {"max_horizontal_velocity",   0.3},
            {"search_timeout",           15.0},
            {"follow_timeout",           60.0},
            {"search_yaw_rate",           0.3},
            {"search_sweep_angle",        0.6},
            {"max_yaw_rate",              0.8},
            {"max_angle_for_translation", 0.6},
            {"pid_lateral_kp",          0.0025},
            {"pid_lateral_ki",             0.0},
            {"pid_lateral_kd",           0.001},
            {"pid_angular_kp",             0.9},
            {"pid_angular_ki",             0.0},
            {"pid_angular_kd",            0.15},

            // Alinhamento com o vetor centro-do-circulo->linha antes de liberar
            // FOLLOW_LINE (ver SearchLineState)
            {"align_kp",                    0.9},
            {"align_ki",                    0.0},
            {"align_kd",                    0.1},
            {"align_tolerance_rad",      0.1745},
            {"max_align_yaw_rate",          0.8},

            // Circle detector (topico /base_circle, ver comentario no callback abaixo)
            {"base_circle_max_age_s",      1.0},

            // Aproximacao ate ficar em cima da mangueira vermelha, antes de girar
            // (ver ApproachRedLineState)
            {"approach_red_timeout",             20.0},
            {"approach_red_tolerance_px",         15.0},
            {"max_approach_red_forward_velocity", 0.3},
            {"approach_red_forward_kp",        0.0025},
            {"approach_red_forward_ki",           0.0},
            {"approach_red_forward_kd",         0.001},

            // Alinhamento (giro) com a mangueira vermelha, depois de ja estar
            // em cima dela (ver AlignRedLineState). Giro: SEM PID, medicao
            // unica (mediana de align_red_theta_samples leituras) + yaw_rate
            // capado por align_red_max_yaw_velocity. Translacao (vx/vy):
            // corrigida por visao o giro inteiro, com PID + EMA -- ver
            // align_red_translation_*.
            {"align_red_timeout",          20.0},
            {"align_red_tolerance_rad",  0.0873},   // ~5 graus -- se a mediana ja vier dentro, nem gira
            {"align_red_theta_samples",    20.0},   // leituras cruas pra' mediana
            {"align_red_yaw_reach_tolerance_rad", 0.035},  // ~2 graus -- criterio de "chegou", medido no proprio drone
            {"align_red_rotate_settle_s",   0.8},   // s parado dentro dessa tolerancia antes de aceitar
            {"align_red_max_yaw_velocity",  0.4},   // rad/s -- teto do "little yaw" por tick
            {"align_red_translation_kp",         0.0025},
            {"align_red_translation_ki",            0.0},
            {"align_red_translation_kd",           0.001},
            {"align_red_max_translation_velocity",  0.3},   // m/s
            {"align_red_translation_ema_alpha",     0.3},   // suaviza x/y_centroid antes do PID -- evita a oscilacao que o forward_pid_ do ApproachRedLineState tinha sem isso

            // Abre a garra depois de alinhado (ver OpenGripperState).
            {"gripper_script_path", std::string("~/evtol/dev/scripts/abrirgarra.py")},
            {"gripper_settle_s",    1.0},   // s parado depois do script antes de liberar a transicao
        };

        auto params = declareAndGetParameters(default_params);
        base_circle_max_age_s_ = std::get<double>(params.at("base_circle_max_age_s"));

        fsm_ = std::make_unique<Fase4ItjbxFSM>(drone_, params);

        rclcpp::QoS qos(10);
        qos.best_effort();
        lane_sub_ = this->create_subscription<custom_msgs::msg::LaneDirection>(
            "/lane_detection", qos,
            [this](const custom_msgs::msg::LaneDirection::SharedPtr msg) {
                fsm_->blackboard_set<bool>("line_detected", !msg->lost);
                fsm_->blackboard_set<float>("line_theta", msg->theta);
                fsm_->blackboard_set<int>("line_x_centroid", msg->x_centroid);
                fsm_->blackboard_set<int>("line_y_centroid", msg->y_centroid);
                fsm_->blackboard_set<int>("line_area", msg->area);
                if (!msg->lost) {
                    fsm_->blackboard_set<float>("last_seen_line_theta", msg->theta);
                }
            }
        );

        // Mangueira vermelha (red_line_detector) -- roda desde o inicio da
        // missao, entao a deteccao ja esta disponivel antes de FOLLOW_LINE
        // checar red_line_detected.
        red_line_sub_ = this->create_subscription<custom_msgs::msg::LaneDirection>(
            "/red_line_detection", qos,
            [this](const custom_msgs::msg::LaneDirection::SharedPtr msg) {
                fsm_->blackboard_set<bool>("red_line_detected", !msg->lost);
                fsm_->blackboard_set<float>("red_line_theta", msg->theta);
                fsm_->blackboard_set<int>("red_line_x_centroid", msg->x_centroid);
                fsm_->blackboard_set<int>("red_line_y_centroid", msg->y_centroid);
            }
        );

        // circle_detector se auto-encerra depois de um tempo (ver
        // circle_detector_node.py) -- guarda quando a ultima mensagem chegou
        // pra zerar line_is_circle/exit_valid se o topico ficar velho demais.
        base_circle_sub_ = this->create_subscription<custom_msgs::msg::BaseCircle>(
            "/base_circle", qos,
            [this](const custom_msgs::msg::BaseCircle::SharedPtr msg) {
                has_base_circle_msg_ = true;
                last_base_circle_time_ = this->now();
                fsm_->blackboard_set<bool>("line_is_circle", msg->found);
                fsm_->blackboard_set<bool>("line_circle_exit_valid", msg->exit_valid);
                fsm_->blackboard_set<float>("line_circle_exit_theta", msg->exit_theta);
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&Fase4ItjbxNode::executeFSM, this));

        // Trajetoria para o RViz2 (convertida de NED para ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "FSM da missao fase4_itjbx iniciada");
    }

private:
    void executeFSM() {
        // Se a ultima mensagem de /base_circle ja esta velha demais (no
        // morreu, ou nunca chegou nenhuma), forca "sem base a vista" em vez
        // de confiar num valor congelado.
        bool base_circle_fresh = has_base_circle_msg_ &&
            (this->now() - last_base_circle_time_).seconds() <= base_circle_max_age_s_;
        if (!base_circle_fresh) {
            fsm_->blackboard_set<bool>("line_is_circle", false);
            fsm_->blackboard_set<bool>("line_circle_exit_valid", false);
        }

        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        // NED -> ENU para visualizar no RViz2.
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp       = this->now();
        ps.header.frame_id    = "map";
        ps.pose.position.x    =  static_cast<float>(pos.y());   // East  = NED y
        ps.pose.position.y    =  static_cast<float>(pos.x());   // North = NED x
        ps.pose.position.z    = -static_cast<float>(pos.z());   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        // Log de estado e posicao a cada 2 s (40 ticks a 20 Hz).
        if (log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] pos=(%.2f, %.2f, %.2f) yaw=%.2f rad",
                        fsm_->get_current_state().c_str(),
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        static_cast<float>(pos.z()),
                        static_cast<float>(orient[2]));
        }

        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "FSM terminou com: %s",
                        fsm_->get_fsm_outcome().c_str());
            rclcpp::shutdown();
        }
    }

    /// Declara cada parametro com seu padrao e le o valor efetivo.
    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>> &defaults) {

        std::map<std::string, std::variant<double, std::string>> result;
        for (const auto &[name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }
        return result;
    }

    std::shared_ptr<Drone> drone_;
    std::unique_ptr<Fase4ItjbxFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::LaneDirection>::SharedPtr lane_sub_;
    rclcpp::Subscription<custom_msgs::msg::LaneDirection>::SharedPtr red_line_sub_;
    rclcpp::Subscription<custom_msgs::msg::BaseCircle>::SharedPtr base_circle_sub_;
    bool has_base_circle_msg_ = false;
    rclcpp::Time last_base_circle_time_;
    double base_circle_max_age_s_ = 1.0;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
    int log_counter_ = 0;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // Drone ja sobe seu proprio executor/thread de spin (Drone.cpp) -- se ele
    // tambem entrar no executor abaixo, lanca em runtime ("Node '/Drone' has
    // already been added to an executor."). So o no da missao entra aqui.
    auto drone        = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Fase4ItjbxNode>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
