#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msgs/msg/multi_base_detection.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_geometry/ground_projector.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "stdstates/return_home_state.hpp"

// Estados desta missao. Crie em include/fase1_itjbx/states/ e inclua aqui.
#include "fase1_itjbx/states/retangular_search_state.hpp"

/**
 * @brief Maquina de estados da missao fase1_itjbx.
 *
 * Ja vem com o ciclo minimo que toda missao tem: armar, decolar, pousar.
 * Para estender, veja os blocos marcados com ACRESCENTE.
 */
class Fase1ItjbxFSM : public fsm::FSM {
public:
    Fase1ItjbxFSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // O drone fica na blackboard: todo estado o acessa por aqui.
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // "bases" e semeada UMA vez e so MUTADA depois via blackboard_get
        // (nunca reescrita com set()), porque set() troca o objeto e
        // invalidaria o ponteiro que RetangularSearchState guarda em on_enter().
        this->blackboard_set<std::vector<Eigen::Vector3d>>("bases", std::vector<Eigen::Vector3d>{});

        // TakeoffState(true) reancora o mundo em (0,0,0) na decolagem.
        this->blackboard_set<Eigen::Vector3d>("home_position", Eigen::Vector3d(0.0, 0.0, 0.0));

        // Parametros do ROS 2 (vindos do YAML) viram entradas da blackboard.
        for (const auto &[key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // ========================= ESTADOS =========================
        this->add_state("ARMING",  std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETANGULAR_SEARCH", std::make_unique<RetangularSearchState>());
        this->add_state("RETURN_HOME", std::make_unique<ReturnHomeState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        // ACRESCENTE aqui os estados desta missao, ex.:
        // this->add_state("SEARCH", std::make_unique<SearchState>());

        // ======================= TRANSICOES ========================
        // Cada linha e: {outcome retornado pelo estado, proximo estado}.
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            // ACRESCENTE: troque "LANDING" pelo primeiro estado da sua missao.
            {"TAKEOFF COMPLETED", "RETANGULAR_SEARCH"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RETANGULAR_SEARCH", {
            {"FAAAHHH", "RETURN_HOME"},
            {"SEARCH COMPLETED", "RETURN_HOME"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RETURN_HOME", {
            {"AT HOME", "LANDING"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

/**
 * @brief No ROS 2 que executa a FSM da missao fase1_itjbx.
 *
 * Declara os parametros (sobrescritos pelo YAML no launch), monta a FSM e a
 * executa a 20 Hz.
 */
class Fase1ItjbxNode : public rclcpp::Node {
public:
    explicit Fase1ItjbxNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("fase1_itjbx_node"), drone_(drone) {

        // Timestamp unico (inicio da missao) pro CSV e a pasta de fotos
        // compartilharem o mesmo nome.
        std::time_t agora = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&agora));
        session_stamp_ = stamp;

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e trocar
        // de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Decolagem (lidos pelo TakeoffState)
            {"takeoff_height",          -2.5},   // metros, NED: negativo e para cima
            {"max_vertical_velocity",    1.2},
            {"position_tolerance",       0.15},

            // Pouso (lidos pelo LandingState)
            {"landing_velocity_max",     0.5},
            {"landing_velocity_min",     0.15},
            {"max_base_height",          0.5},
            {"landing_timeout",          5.0},

            // Movimento horizontal
            {"max_horizontal_velocity",  1.5},

            // Retangular Search
            {"abs_x", 6.0},
            {"abs_y", 6.0},
            {"diagonal_step", 1.5},
            {"total_bases", 5.0},

            // Suavizacao das curvas de 90 graus (ver rampa de velocidade em
            // RetangularSearchState::act()): dentro deste raio do vertice a
            // velocidade cai ate corner_min_velocity.
            {"corner_decel_radius", 1.0},
            {"corner_min_velocity", 0.3}
            // ACRESCENTE aqui os parametros desta missao, e replique-os nos
            // dois YAML de config/.
        };

        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase1ItjbxFSM>(drone_, params);

        declareVisionParameters();
        projector_ = std::make_unique<vision_geometry::GroundProjector>(buildCalibration());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&Fase1ItjbxNode::executeFSM, this));

        // Trajetoria para o RViz2 (convertida de NED para ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // Marcadores das bases detectadas, para o RViz2 (ver rviz/trajectory.rviz).
        base_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/fase1_itjbx/base_markers", 10);

        // Assinatura do no de visao (base_detector_itjbx2026, em
        // src/cv_nodes/base_detector_itjbx2026). O callback escreve na
        // blackboard; os estados so leem de la.
        bases_sub_ = this->create_subscription<custom_msgs::msg::MultiBaseDetection>(
            "/base_detector_itjbx2026/detections", 10,
            std::bind(&Fase1ItjbxNode::basesCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FSM da missao fase1_itjbx iniciada");
    }

private:
    /// Parametros so' usados por este no' (calibracao de camera, projecao,
    /// dedup de bases) -- separados do default_params/blackboard porque
    /// nenhum estado precisa deles.
    void declareVisionParameters() {
        // ── Intrinsecos ────────────────────────────────────────────────
        // fx=0.0 (padrao) deriva os intrinsecos do FOV; informe camera_fx
        // para usar uma calibracao de verdade. Ver buildCalibration().
        this->declare_parameter<double>("camera_fx", 0.0);
        this->declare_parameter<double>("camera_fy", 0.0);
        this->declare_parameter<double>("camera_cx", 0.0);
        this->declare_parameter<double>("camera_cy", 0.0);
        this->declare_parameter<double>("camera_hfov", 1.047);
        this->declare_parameter<int>("camera_width", 640);
        this->declare_parameter<int>("camera_height", 480);
        this->declare_parameter<int>("published_size", 0);

        // ── Extrinsecos ────────────────────────────────────────────────
        // yaw = pi/2 NAO e arbitrario: e' a montagem em que o topo da imagem
        // aponta pra frente do drone (extrinsecos nulos dariam uma camera
        // olhando pra baixo, mas girada 90 graus).
        this->declare_parameter<double>("camera_roll", 0.0);
        this->declare_parameter<double>("camera_pitch", 0.0);
        this->declare_parameter<double>("camera_yaw", M_PI / 2.0);
        this->declare_parameter<double>("camera_tx", 0.0);
        this->declare_parameter<double>("camera_ty", 0.0);
        this->declare_parameter<double>("camera_tz", 0.0);

        // ── Projecao e deduplicacao ───────────────────────────────────
        // Altura do plano das bases, em mundo FRD (positivo = abaixo da
        // origem). 0.0 e' correto se as bases estao no mesmo nivel do chao
        // de decolagem.
        this->declare_parameter<double>("base_plane_z", 0.0);
        base_plane_z_ = this->get_parameter("base_plane_z").as_double();

        // Raio (plano XY) abaixo do qual duas deteccoes contam como a MESMA
        // base fisica. Pequeno demais duplica por ruido; grande demais funde
        // bases vizinhas de verdade.
        this->declare_parameter<double>("base_dedup_radius", 0.75);
        base_dedup_radius_ = this->get_parameter("base_dedup_radius").as_double();

        // Fracao da imagem, de cada lado, tratada como "borda": deteccoes com
        // centro normalizado fora de [margem, 1-margem] em X ou Y sao
        // descartadas antes de projetar (ver basesCallback).
        this->declare_parameter<double>("border_margin_frac", 0.08);
        border_margin_frac_ = this->get_parameter("border_margin_frac").as_double();
    }

    /// Intrinsecos/extrinsecos da camera para o GroundProjector: fx/fy/cx/cy
    /// diretos se calibrados (camera_fx != 0), senao derivados do FOV.
    vision_geometry::CameraCalibration buildCalibration() {
        const double fx = this->get_parameter("camera_fx").as_double();
        const int width = this->get_parameter("camera_width").as_int();
        const int height = this->get_parameter("camera_height").as_int();

        vision_geometry::CameraCalibration calib;

        if (fx > 0.0) {
            calib.fx = fx;
            calib.fy = this->get_parameter("camera_fy").as_double();
            calib.cx = this->get_parameter("camera_cx").as_double();
            calib.cy = this->get_parameter("camera_cy").as_double();
            calib.image_width = width;
            calib.image_height = height;
        } else {
            int published = this->get_parameter("published_size").as_int();
            if (published <= 0) published = std::min(width, height);

            calib = vision_geometry::CameraCalibration::fromHorizontalFov(
                this->get_parameter("camera_hfov").as_double(), width, height, published);
            RCLCPP_INFO(this->get_logger(),
                "camera_fx nao informado: intrinsecos derivados do FOV (fx=%.1f, %dx%d)",
                calib.fx, calib.image_width, calib.image_height);
        }

        calib.roll = this->get_parameter("camera_roll").as_double();
        calib.pitch = this->get_parameter("camera_pitch").as_double();
        calib.yaw = this->get_parameter("camera_yaw").as_double();
        calib.tx = this->get_parameter("camera_tx").as_double();
        calib.ty = this->get_parameter("camera_ty").as_double();
        calib.tz = this->get_parameter("camera_tz").as_double();
        calib.bbox_is_normalized = true;  // base_detector_itjbx2026 so publica [0,1]

        return calib;
    }

    /// Indice da base acumulada mais proxima da candidata, ou -1 se nenhuma
    /// estiver a menos de base_dedup_radius_. Comparacao so' no plano XY --
    /// a componente Z carrega o erro de projecao.
    int matchingBaseIndex(const Eigen::Vector3d &candidata,
                           const std::vector<Eigen::Vector3d> &bases) const {
        for (size_t i = 0; i < bases.size(); ++i) {
            const double d = (candidata.head<2>() - bases[i].head<2>()).norm();
            if (d < base_dedup_radius_) return static_cast<int>(i);
        }
        return -1;
    }

    void basesCallback(const custom_msgs::msg::MultiBaseDetection::SharedPtr msg) {
        if (msg->bases.empty()) return;

        // Mutado por cima do ponteiro (ver o comentario no construtor da FSM),
        // e ACUMULADO -- nunca esvaziado aqui, porque RetangularSearchState
        // compara bases_->size() contra total_bases_ ao longo de toda a missao.
        auto *bases_ptr = fsm_->blackboard_get<std::vector<Eigen::Vector3d>>("bases");
        if (bases_ptr == nullptr) return;

        // Mesma pose para todas as deteccoes desta mensagem: vieram do
        // mesmo frame.
        vision_geometry::DronePose pose;
        pose.position = drone_->getLocalPosition();
        pose.rpy = drone_->getOrientation();

        bool mudou = false;
        for (const auto &b : msg->bases) {
            // Deteccao perto da borda do quadro tem angulo de visada mais
            // obliquo, e projectToPlane() erra mais proporcionalmente nesse
            // caso (sem guarda embutida contra isso). Descartar aqui ataca a
            // causa do ruido, em vez de so' alargar base_dedup_radius_ (que
            // arrisca fundir bases diferentes).
            if (b.position.x < border_margin_frac_ || b.position.x > 1.0 - border_margin_frac_ ||
                b.position.y < border_margin_frac_ || b.position.y > 1.0 - border_margin_frac_) {
                continue;
            }

            vision_geometry::BoundingBox bbox;
            bbox.center_x = b.position.x;  // pixel normalizado [0,1], nao metros
            bbox.center_y = b.position.y;

            const Eigen::Vector3d posicao_mundo =
                projector_->projectToPlane(pose, bbox, base_plane_z_);

            // Distancia do centro da deteccao ao centro do QUADRO (nao do
            // mundo): quanto mais perto do centro, mais de frente a foto --
            // e' a "melhor" pra guardar como retrato da base.
            const double dist_centro_quadro = std::hypot(b.position.x - 0.5, b.position.y - 0.5);

            const int idx = matchingBaseIndex(posicao_mundo, *bases_ptr);
            if (idx < 0) {
                bases_ptr->push_back(posicao_mundo);
                base_hit_counts_.push_back(1);
                base_best_center_dist_.push_back(dist_centro_quadro);
                mudou = true;
                drone_->log(
                    "Base nova em {" + std::to_string(posicao_mundo.x()) + ", " +
                    std::to_string(posicao_mundo.y()) + "} (" +
                    std::to_string(bases_ptr->size()) + " ate agora)");
                savePhoto(bases_ptr->size() - 1, msg->frame);
            } else {
                // Media movel, nao descarte simples: a mesma base vista de
                // angulos diferentes projeta em posicoes ligeiramente
                // distintas, entao refinar a cada deteccao repetida converge
                // pra um centroide mais estavel.
                base_hit_counts_[idx]++;
                const double peso_novo = 1.0 / static_cast<double>(base_hit_counts_[idx]);
                (*bases_ptr)[idx] += peso_novo * (posicao_mundo - (*bases_ptr)[idx]);
                mudou = true;

                // So troca a foto se este enquadramento for MELHOR (mais
                // perto do centro) que o guardado ate agora -- nao a cada
                // deteccao repetida, que na maioria das vezes e' pior (o
                // drone ja esta se afastando).
                if (dist_centro_quadro < base_best_center_dist_[idx]) {
                    base_best_center_dist_[idx] = dist_centro_quadro;
                    savePhoto(idx, msg->frame);
                }
            }
        }

        // So republica quando o conjunto muda -- os marcadores tem duracao
        // infinita no RViz2 (lifetime 0), entao nao precisam ser reenviados
        // a cada frame para continuar aparecendo.
        if (mudou) publishBaseMarkers(*bases_ptr);
    }

    /// Grava msg.frame como o retrato da base `idx`, em
    /// ~/evtol/mission_logs/fase1_itjbx_bases_<session_stamp_>/base_<idx>.jpg.
    /// Sobrescreve sem aviso (e' assim que um enquadramento melhor substitui o anterior).
    void savePhoto(size_t idx, const sensor_msgs::msg::CompressedImage &frame) {
        if (frame.data.empty()) return;  // no_detections nao embute frame (ver node.py)

        const std::string dir = photosDir();
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Nao consegui criar '%s': %s",
                         dir.c_str(), ec.message().c_str());
            return;
        }

        const std::string ext = frame.format.find("png") != std::string::npos ? ".png" : ".jpg";
        const std::string path = dir + "/base_" + std::to_string(idx) + ext;

        std::ofstream out(path, std::ios::binary);
        if (!out.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Nao consegui abrir '%s' para gravar a foto",
                         path.c_str());
            return;
        }
        out.write(reinterpret_cast<const char *>(frame.data.data()),
                  static_cast<std::streamsize>(frame.data.size()));
    }

    std::string photosDir() const {
        return std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") +
               "/evtol/mission_logs/fase1_itjbx_bases_" + session_stamp_;
    }

    /// Uma esfera + um texto com o indice para cada base acumulada, em
    /// /fase1_itjbx/base_markers. Mesma conversao NED->ENU do /drone_trajectory
    /// (ver executeFSM) -- os dois tem que concordar no RViz2.
    void publishBaseMarkers(const std::vector<Eigen::Vector3d> &bases) {
        visualization_msgs::msg::MarkerArray array;

        for (size_t i = 0; i < bases.size(); ++i) {
            const auto &p = bases[i];
            const auto stamp = this->now();

            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = stamp;
            sphere.ns = "bases";
            sphere.id = static_cast<int>(i);
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = p.y();    // East  = NED y
            sphere.pose.position.y = p.x();    // North = NED x
            sphere.pose.position.z = -p.z();   // Up    = -NED z
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
            sphere.color.r = 1.0f;
            sphere.color.g = 0.55f;
            sphere.color.b = 0.0f;
            sphere.color.a = 1.0f;
            sphere.lifetime = rclcpp::Duration(0, 0);  // 0 = nunca expira
            array.markers.push_back(sphere);

            visualization_msgs::msg::Marker label;
            label.header = sphere.header;
            label.ns = "bases_id";
            label.id = static_cast<int>(i);
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = sphere.pose.position.x;
            label.pose.position.y = sphere.pose.position.y;
            label.pose.position.z = sphere.pose.position.z + 0.5;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.35;
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
            label.text = std::to_string(i);
            label.lifetime = sphere.lifetime;
            array.markers.push_back(label);
        }

        base_markers_pub_->publish(array);
    }

    /// Grava as bases acumuladas num CSV em ~/evtol/mission_logs/, uma vez,
    /// quando a FSM termina (ver executeFSM).
    void writeBasesReport() {
        auto *bases_ptr = fsm_->blackboard_get<std::vector<Eigen::Vector3d>>("bases");

        const std::string dir =
            std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") +
            "/evtol/mission_logs";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);

        const std::string path = dir + "/fase1_itjbx_bases_" + session_stamp_ + ".csv";

        std::ofstream out(path);
        if (!out.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Nao consegui abrir '%s' para escrever as bases",
                         path.c_str());
            return;
        }

        const size_t total = bases_ptr != nullptr ? bases_ptr->size() : 0;
        out << "# fase1_itjbx -- bases detectadas (FRD, metros)\n";
        out << "# encontradas: " << total << "\n";
        out << "# fotos em: " << photosDir() << "/base_<id>.jpg\n";
        out << "id,x,y,z\n";
        if (bases_ptr != nullptr) {
            for (size_t i = 0; i < bases_ptr->size(); ++i) {
                const auto &p = (*bases_ptr)[i];
                out << i << "," << p.x() << "," << p.y() << "," << p.z() << "\n";
            }
        }
        out.close();

        RCLCPP_INFO(this->get_logger(), "Bases detectadas (%zu) gravadas em %s",
                    total, path.c_str());
    }

    void executeFSM() {
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
            if (!report_written_) {
                writeBasesReport();
                report_written_ = true;
            }
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
    std::unique_ptr<Fase1ItjbxFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr base_markers_pub_;
    rclcpp::Subscription<custom_msgs::msg::MultiBaseDetection>::SharedPtr bases_sub_;
    std::unique_ptr<vision_geometry::GroundProjector> projector_;
    double base_plane_z_ = 0.0;
    double base_dedup_radius_ = 0.75;
    double border_margin_frac_ = 0.08;
    // Paralelo a bases_ptr (mesmo indice, mesmo tamanho) para a media movel
    // em basesCallback -- fica so aqui, nao na blackboard, porque nenhum
    // estado precisa saber quantas vezes uma base foi vista.
    std::vector<int> base_hit_counts_;
    // Tambem paralelo a bases_ptr: a menor distancia ao centro do quadro ja
    // vista para cada base, para savePhoto() so trocar a foto quando um
    // enquadramento melhor aparecer.
    std::vector<double> base_best_center_dist_;
    std::string session_stamp_;
    nav_msgs::msg::Path trajectory_;
    int log_counter_ = 0;
    bool report_written_ = false;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // Drone ja sobe seu proprio executor/thread de spin (Drone.cpp) -- se ele
    // tambem entrar no executor abaixo, lanca em runtime ("Node '/Drone' has
    // already been added to an executor."). So o no da missao entra aqui.
    auto drone        = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Fase1ItjbxNode>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
