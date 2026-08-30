#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "custom_msgs/msg/bouncing_detection.hpp"
#include "custom_msgs/msg/read_base_number_result.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"

// Estados desta missao -- portados de sae2026/mission_1, que resolve
// essencialmente o mesmo problema (achar um ArUco, identificar a forma ao
// redor dele, procurar a base numerada correspondente e pousar nela). A
// unica mudanca de logica em relacao ao original e' o padrao de varredura:
// SearchArucoState/SearchBaseState usam a varredura retangular de
// itajuba_2026/fase1_itjbx (RetangularSearchState) em vez do spiral quadrado
// que a mission_1 usava -- ver a conversa que motivou esta migracao.
//
// CONFIRM_NUMBER/RETURN_TO_SEARCH sao NOVOS em relacao a mission_1: a
// Raspberry Pi nao aguenta rodar EasyOCR a cada frame, entao RDPformas.py
// agora so' detecta FORMA continuamente (leve) e a leitura de digito vira
// um pedido pontual (1 foto), disparado so' depois que o drone ja alinhou
// com uma base do mesmo formato do ArUco -- ver a conversa que motivou essa
// mudanca.
#include "fase2_itjbx/states/initial_aruco_search_state.hpp"
#include "fase2_itjbx/states/search_aruco_state.hpp"
#include "fase2_itjbx/states/go_to_aruco_state.hpp"
#include "fase2_itjbx/states/search_base_state.hpp"
#include "fase2_itjbx/states/go_to_base_state.hpp"
#include "fase2_itjbx/states/descend_for_shape_state.hpp"
#include "fase2_itjbx/states/confirm_number_state.hpp"
#include "fase2_itjbx/states/return_to_search_state.hpp"

/**
 * @brief Maquina de estados da missao fase2_itjbx.
 *
 * Portada de sae2026/mission_1::Mission1FSM. Fluxo: sobe procurando o ArUco
 * (INITIAL_ARUCO_SEARCH); se nao achar so subindo, varre o campo em
 * retangulo (SEARCH_ARUCO); alinha sobre o ArUco (GO_TO_ARUCO); se a forma
 * ao redor nao for identificavel na altitude de busca, desce pra
 * identificar (DESCEND_FOR_SHAPE); com o alvo calculado, procura a base
 * numerada certa (SEARCH_BASE) e pousa nela (GO_TO_BASE -> LANDING).
 */
class Fase2ItjbxFSM : public fsm::FSM {
public:
    Fase2ItjbxFSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // O drone fica na blackboard: todo estado o acessa por aqui.
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // Parametros do ROS 2 (vindos do YAML) viram entradas da blackboard.
        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // ========================= ESTADOS =========================
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("INITIAL_ARUCO_SEARCH", std::make_unique<InitialArucoSearchState>());
        this->add_state("SEARCH_ARUCO", std::make_unique<SearchArucoState>());
        this->add_state("GO_TO_ARUCO", std::make_unique<GoToArucoState>());
        this->add_state("DESCEND_FOR_SHAPE", std::make_unique<DescendForShapeState>());
        this->add_state("SEARCH_BASE", std::make_unique<SearchBaseState>());
        this->add_state("GO_TO_BASE", std::make_unique<GoToBaseState>());
        this->add_state("CONFIRM_NUMBER", std::make_unique<ConfirmNumberState>());
        this->add_state("RETURN_TO_SEARCH", std::make_unique<ReturnToSearchState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // ======================= TRANSICOES ========================
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "INITIAL_ARUCO_SEARCH"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("INITIAL_ARUCO_SEARCH", {
            {"ARUCO_FOUND", "GO_TO_ARUCO"},
            {"MAX_ALTITUDE_REACHED", "SEARCH_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_ARUCO", {
            {"ARUCO_FOUND", "GO_TO_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GO_TO_ARUCO", {
            {"ARUCO_LOST", "SEARCH_ARUCO"},
            {"SHAPE_UNKNOWN", "DESCEND_FOR_SHAPE"},
            // KNOWN_BASE ia direto pra GO_TO_BASE quando uma base do mesmo
            // formato ja estava a vista no instante do alinhamento com o
            // ArUco -- pulava a volta pro centro e podia perseguir uma base
            // errada vista de relance no caminho, sem nunca ter comecado a
            // varredura de verdade. Agora os dois casos passam por
            // SEARCH_BASE, que centraliza em (0,0) e SO' reage a deteccoes
            // depois de chegar la' (ver SearchBaseState::going_to_center_).
            {"UNKNOWN_BASE", "SEARCH_BASE"},
            {"KNOWN_BASE", "SEARCH_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("DESCEND_FOR_SHAPE", {
            {"SHAPE_FOUND", "GO_TO_ARUCO"},
            {"ARUCO_LOST", "SEARCH_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_BASE", {
            {"BASE_FOUND", "GO_TO_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GO_TO_BASE", {
            {"ALIGNED", "CONFIRM_NUMBER"},
            {"BASE_LOST", "SEARCH_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("CONFIRM_NUMBER", {
            {"NUMBER_CONFIRMED", "LANDING"},
            {"NUMBER_WRONG", "RETURN_TO_SEARCH"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RETURN_TO_SEARCH", {
            {"AT_START", "SEARCH_BASE"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("TAKEOFF");
    }
};

/**
 * @brief No ROS 2 que executa a FSM da missao fase2_itjbx.
 *
 * Portado de sae2026/mission_1::Mission1Node. Assina o mesmo detector de
 * visao (RDPformas, topico "bouncing_detection", msg custom_msgs/BouncingDetection)
 * que a mission_1 usa -- ele ja publica ArUco + forma + bases numeradas no
 * formato que este no espera.
 */
class Fase2ItjbxNode : public rclcpp::Node {
public:
    Fase2ItjbxNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("fase2_itjbx_node"), drone_(drone) {

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e
        // trocar de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Decolagem (NED: altura negativa e para cima)
            {"takeoff_height",         -2.5},
            {"max_vertical_velocity",  1.2},
            {"position_tolerance",     0.15},

            // Pouso
            {"landing_velocity_max",   0.5},
            {"landing_velocity_min",   0.15},
            {"max_base_height",        0.5},
            {"landing_timeout",        5.0},

            // Movimento horizontal
            {"max_horizontal_velocity", 1.5},

            // Busca do ArUco / identificacao da forma
            {"z_max_search", -2.5},
            {"search_aruco_velocity", 0.4},
            {"aruco_persistence_frames", 3.0},
            {"aruco_tolerance", 0.15},
            {"aruco_kp_x", 0.6},
            {"aruco_kp_y", 0.6},
            {"aruco_kd_x", 0.05},
            {"aruco_kd_y", 0.05},
            {"aruco_align_frames", 5.0},
            {"shape_id_altitude", -1.0},
            {"shape_id_velocity", 0.3},

            // Busca/aproximacao da base numerada alvo
            {"base_tolerance", 0.10},
            {"base_kp_x", 0.5},
            {"base_kp_y", 0.5},
            {"base_kd_x",  0.03},
            {"base_kd_y",  0.03},
            {"base_persistence_frames", 3.0},
            {"base_cam_scale", 0.7},
            {"base_max_err_radius", 0.7},
            {"base_dedup_radius", 1.5},
            {"aruco_exclusion_radius", 0.5},

            // Confirmacao de numero sob demanda (CONFIRM_NUMBER) e bases
            // ja descartadas (ver ConfirmNumberState/cv_callback).
            {"ocr_timeout_ticks", 200.0},   // 10s @ 20Hz esperando resposta do OCR
            {"ignored_base_radius", 1.0},   // raio (m) pra considerar "a mesma base ja rejeitada"

            // Varredura retangular (RetangularSearchState, fase1_itjbx) --
            // usada tanto por SEARCH_ARUCO quanto por SEARCH_BASE. A busca de
            // ArUco cobre o campo inteiro a partir de casa; a de base e um
            // padrao mais apertado, local, em torno de uma posicao em cache.
            {"aruco_search_abs_x", 6.0},
            {"aruco_search_abs_y", 6.0},
            {"aruco_search_step", 1.0},
            {"base_search_abs_x", 3.0},
            {"base_search_abs_y", 3.0},
            {"base_search_step", 0.5},
            {"corner_decel_radius", 1.0},
            {"corner_min_velocity", 0.3},
        };

        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase2ItjbxFSM>(drone_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Fase2ItjbxNode::executeFSM, this));

        // Trajetoria para o RViz2 (NED -> ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // Publica sempre que uma base e vista pela primeira vez (vai pro rosbag).
        discovered_bases_pub_ = this->create_publisher<std_msgs::msg::String>("/discovered_bases", 10);

        // Marcadores das bases descobertas, para o RViz2.
        base_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/fase2_itjbx/base_markers", 10);
        base_markers_pub_->publish(visualization_msgs::msg::MarkerArray{});

        // Assinatura do no de visao. RDPformas (src/cv_nodes/RDPformas)
        // publica BouncingDetection em "bouncing_detection" -- mesmo topico
        // e mesma mensagem que a mission_1 do sae2026 consome.
        cv_sub_ = this->create_subscription<custom_msgs::msg::BouncingDetection>(
            "bouncing_detection", 10,
            std::bind(&Fase2ItjbxNode::cv_callback, this, std::placeholders::_1)
        );

        // Leitura de digito SOB DEMANDA (ver ConfirmNumberState): pedido
        // sai por aqui, resposta chega por ocr_result_sub_. RDPformas.py
        // assina/publica esses dois mesmos topicos.
        ocr_request_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/read_base_number_request", 10);
        ocr_result_sub_ = this->create_subscription<custom_msgs::msg::ReadBaseNumberResult>(
            "/read_base_number_result", 10,
            std::bind(&Fase2ItjbxNode::ocr_result_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "FSM da missao fase2_itjbx iniciada");
    }

private:
    /// Resposta do pedido de leitura de digito sob demanda (ver
    /// ConfirmNumberState) -- so' escreve na blackboard; quem interpreta o
    /// resultado (compara com o numero esperado) e' o proprio estado.
    void ocr_result_callback(const custom_msgs::msg::ReadBaseNumberResult::SharedPtr msg) {
        fsm_->blackboard_set<bool>("ocr_result_ready", true);
        fsm_->blackboard_set<bool>("ocr_result_success", msg->success);
        fsm_->blackboard_set<std::string>("ocr_result_digit", msg->digit);
        fsm_->blackboard_set<float>("ocr_result_confidence", msg->confidence);
        RCLCPP_INFO(this->get_logger(), "[OCR] resultado: success=%d digit=%s conf=%.2f",
            msg->success, msg->digit.c_str(), msg->confidence);
    }

    void cv_callback(const custom_msgs::msg::BouncingDetection::SharedPtr msg) {
        // Info do ArUco
        fsm_->blackboard_set<bool>("aruco_detected", msg->aruco_detected);
        if (msg->aruco_detected) {
            fsm_->blackboard_set<int>("aruco_id", msg->aruco_id);
            fsm_->blackboard_set<std::string>("aruco_shape", msg->aruco_shape);

            // Filtro EMA na posicao do ArUco: suaviza ruido de um frame so e
            // evita o pico de derivada que aparece quando a deteccao
            // "pisca" (perde e reaparece).
            if (aruco_first_detection_) {
                aruco_smoothed_x_ = msg->aruco_x_error;
                aruco_smoothed_y_ = msg->aruco_y_error;
                aruco_first_detection_ = false;
            } else {
                constexpr float kAlpha = 0.45f;
                aruco_smoothed_x_ = kAlpha * msg->aruco_x_error
                                  + (1.0f - kAlpha) * aruco_smoothed_x_;
                aruco_smoothed_y_ = kAlpha * msg->aruco_y_error
                                  + (1.0f - kAlpha) * aruco_smoothed_y_;
            }
            fsm_->blackboard_set<float>("aruco_x_error", aruco_smoothed_x_);
            fsm_->blackboard_set<float>("aruco_y_error", aruco_smoothed_y_);
        } else {
            aruco_first_detection_ = true;
        }
        if (msg->aruco_detected != prev_aruco_detected_) {
            if (msg->aruco_detected)
                RCLCPP_INFO(this->get_logger(), "[CV] ArUco DETECTADO id=%d shape=%s",
                    msg->aruco_id, msg->aruco_shape.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "[CV] ArUco PERDIDO");
            prev_aruco_detected_ = msg->aruco_detected;
        }

        // Info do alvo -- travado: uma vez identificado, nunca reseta sozinho
        // (a forma + regra de divisibilidade sao deterministicas pro ArUco ID fixo).
        if (msg->target_calculated) {
            fsm_->blackboard_set<bool>("target_calculated", true);
            fsm_->blackboard_set<std::string>("target_base", msg->target_base);
        }
        if (msg->target_calculated && !prev_target_calculated_) {
            RCLCPP_INFO(this->get_logger(), "[CV] Alvo identificado: %s",
                msg->target_base.c_str());
            prev_target_calculated_ = true;
        }

        // Bases visiveis
        fsm_->blackboard_set<bool>("target_base_in_sight", msg->target_base_in_sight);
        if (msg->target_base_in_sight) {
            fsm_->blackboard_set<float>("target_base_x_error", msg->target_base_x_error);
            fsm_->blackboard_set<float>("target_base_y_error", msg->target_base_y_error);
        }
        if (msg->target_base_in_sight != prev_target_base_in_sight_) {
            if (msg->target_base_in_sight)
                RCLCPP_INFO(this->get_logger(), "[CV] Base alvo A VISTA (err=%.2f,%.2f)",
                    msg->target_base_x_error, msg->target_base_y_error);
            else
                RCLCPP_INFO(this->get_logger(), "[CV] Base alvo PERDIDA");
            prev_target_base_in_sight_ = msg->target_base_in_sight;
        }

        // Cache de posicao das bases, com deduplicacao espacial e pontuacao
        // de confianca -- identico a sae2026/mission_1::cv_callback.
        {
            float* cam_scale_ptr = fsm_->blackboard_get<float>("base_cam_scale");
            float cam_scale = cam_scale_ptr ? *cam_scale_ptr : 0.7f;
            float alt = -static_cast<float>(drone_->getLocalPosition().z());
            float yaw = static_cast<float>(drone_->getOrientation()[2]);
            auto  dpos = drone_->getLocalPosition();

            float* max_err_ptr = fsm_->blackboard_get<float>("base_max_err_radius");
            float max_err_radius = max_err_ptr ? *max_err_ptr : 0.7f;

            float* dedup_ptr = fsm_->blackboard_get<float>("base_dedup_radius");
            float dedup_r = dedup_ptr ? *dedup_ptr : 1.5f;

            // So trava a posicao mundial do ArUco depois que o alvo foi
            // calculado (drone alinhado, erro ~0 -> estimativa boa). Durante
            // a varredura distante o erro de projecao e grande demais.
            if (msg->aruco_detected && !aruco_world_valid_) {
                bool* tc_ptr = fsm_->blackboard_get<bool>("target_calculated");
                if (tc_ptr && *tc_ptr) {
                    float ldx_a = -msg->aruco_y_error * alt * cam_scale;
                    float ldy_a =  msg->aruco_x_error * alt * cam_scale;
                    aruco_world_x_ = static_cast<float>(dpos.x())
                                     + ldx_a * std::cos(yaw) - ldy_a * std::sin(yaw);
                    aruco_world_y_ = static_cast<float>(dpos.y())
                                     + ldx_a * std::sin(yaw) + ldy_a * std::cos(yaw);
                    aruco_world_valid_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "[ARUCO_POS] Posicao mundial travada @ (%.2f, %.2f)",
                        aruco_world_x_, aruco_world_y_);
                }
            }

            float* excl_ptr = fsm_->blackboard_get<float>("aruco_exclusion_radius");
            float aruco_excl_r = excl_ptr ? *excl_ptr : 0.5f;

            // Suprime bases ja' REJEITADAS por CONFIRM_NUMBER (numero lido
            // nao batia com o alvo) -- calcula a posicao no mundo do
            // candidato ATUAL e verifica se cai perto de alguma base
            // registrada em "ignored_base_*" (ver
            // ConfirmNumberState::registrar_base_ignorada). Se cair, este
            // tick nao conta como "base alvo a vista" pra' SEARCH_BASE/
            // GO_TO_BASE nao voltarem a perseguir a mesma base rejeitada.
            if (msg->target_base_in_sight) {
                float ldx_t = -msg->target_base_y_error * alt * cam_scale;
                float ldy_t =  msg->target_base_x_error * alt * cam_scale;
                float twx = static_cast<float>(dpos.x())
                            + ldx_t * std::cos(yaw) - ldy_t * std::sin(yaw);
                float twy = static_cast<float>(dpos.y())
                            + ldx_t * std::sin(yaw) + ldy_t * std::cos(yaw);

                int* ignored_count_ptr = fsm_->blackboard_get<int>("ignored_base_count");
                int ignored_count = ignored_count_ptr ? *ignored_count_ptr : 0;
                float* ignore_r_ptr = fsm_->blackboard_get<float>("ignored_base_radius");
                float ignore_r = ignore_r_ptr ? *ignore_r_ptr : 1.0f;

                for (int i = 0; i < ignored_count; ++i) {
                    float* ix = fsm_->blackboard_get<float>("ignored_base_" + std::to_string(i) + "_x");
                    float* iy = fsm_->blackboard_get<float>("ignored_base_" + std::to_string(i) + "_y");
                    if (!ix || !iy) continue;
                    if (std::hypot(twx - *ix, twy - *iy) < ignore_r) {
                        fsm_->blackboard_set<bool>("target_base_in_sight", false);
                        break;
                    }
                }
            }

            auto store_base_pos = [&](const std::string& label, float ex, float ey, float det_conf) {
                if (label.empty()) return;
                if (std::hypot(ex, ey) > max_err_radius) return;

                float ldx = -ey * alt * cam_scale;
                float ldy =  ex * alt * cam_scale;
                float wx  = static_cast<float>(dpos.x())
                            + ldx * std::cos(yaw) - ldy * std::sin(yaw);
                float wy  = static_cast<float>(dpos.y())
                            + ldx * std::sin(yaw) + ldy * std::cos(yaw);
                float r   = std::max(0.5f, std::hypot(ex, ey) * alt * cam_scale + 0.5f);

                if (std::hypot(wx, wy) < 0.30f) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[IGNORE_LAUNCHPAD] %s @ (%.2f, %.2f) -- perto demais da origem",
                        label.c_str(), wx, wy);
                    return;
                }

                if (aruco_world_valid_) {
                    float d = std::hypot(wx - aruco_world_x_, wy - aruco_world_y_);
                    if (d < aruco_excl_r) {
                        RCLCPP_DEBUG(this->get_logger(),
                            "[IGNORE_ARUCO] %s @ (%.2f, %.2f) -- %.2f m do ArUco (excl=%.2f)",
                            label.c_str(), wx, wy, d, aruco_excl_r);
                        return;
                    }
                }

                float pos_q = 1.0f - std::min(1.0f, std::hypot(ex, ey));
                float conf  = det_conf * pos_q;

                std::string closest_label;
                float closest_dist = dedup_r + 1.0f;
                std::string weakest_nearby_label;
                float weakest_conf = 1.1f;
                int nearby_count = 0;

                for (const auto& lbl : known_base_labels_) {
                    float* kx = fsm_->blackboard_get<float>("known_base_" + lbl + "_x");
                    float* ky = fsm_->blackboard_get<float>("known_base_" + lbl + "_y");
                    if (!kx || !ky) continue;
                    float dist = std::hypot(wx - *kx, wy - *ky);
                    if (dist < dedup_r) {
                        nearby_count++;
                        if (dist < closest_dist) {
                            closest_dist = dist;
                            closest_label = lbl;
                        }
                        float* kc = fsm_->blackboard_get<float>("known_base_" + lbl + "_conf");
                        float known_c = kc ? *kc : 0.0f;
                        if (known_c < weakest_conf) {
                            weakest_conf = known_c;
                            weakest_nearby_label = lbl;
                        }
                    }
                }

                auto write_entry = [&](const std::string& key_label) {
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_x", wx);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_y", wy);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_r", r);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_conf", conf);
                    known_base_labels_.insert(key_label);
                };

                if (nearby_count == 0) {
                    write_entry(label);
                    std::string discovery = "[DISCOVERY] " + label +
                        " @ (" + std::to_string(wx) + ", " + std::to_string(wy) +
                        ") r=" + std::to_string(r) + " conf=" + std::to_string(conf);
                    RCLCPP_INFO(this->get_logger(), "%s", discovery.c_str());
                    std_msgs::msg::String pub_msg;
                    pub_msg.data = discovery;
                    discovered_bases_pub_->publish(pub_msg);

                } else if (nearby_count == 1) {
                    float* kc = fsm_->blackboard_get<float>("known_base_" + closest_label + "_conf");
                    float known_c = kc ? *kc : 0.0f;

                    if (label == closest_label) {
                        if (conf > known_c) {
                            write_entry(label);
                            RCLCPP_INFO(this->get_logger(),
                                "[UPDATE] %s conf %.2f→%.2f", label.c_str(), known_c, conf);
                        }
                    } else {
                        if (conf > known_c) {
                            write_entry(label);
                            RCLCPP_INFO(this->get_logger(),
                                "[NEW_CONF] %s (conf %.2f) perto de %s (conf %.2f) -- adicionada",
                                label.c_str(), conf, closest_label.c_str(), known_c);
                        }
                    }

                } else {
                    if (conf > weakest_conf && label == weakest_nearby_label) {
                        write_entry(label);
                        RCLCPP_INFO(this->get_logger(),
                            "[UPDATE_MULTI] %s conf %.2f→%.2f",
                            label.c_str(), weakest_conf, conf);
                    } else if (conf > weakest_conf && label != weakest_nearby_label) {
                        write_entry(label);
                        RCLCPP_INFO(this->get_logger(),
                            "[NEW_CONF_MULTI] %s (conf %.2f) adicionada perto do cluster",
                            label.c_str(), conf);
                    }
                }
            };

            size_t n = msg->visible_bases.size();
            for (size_t i = 0; i < n; ++i) {
                float det_conf = (i < msg->visible_bases_confidence.size())
                    ? msg->visible_bases_confidence[i] : 0.3f;
                store_base_pos(msg->visible_bases[i],
                               msg->visible_bases_x_error[i],
                               msg->visible_bases_y_error[i],
                               det_conf);
            }
            if (msg->target_base_in_sight)
                store_base_pos(msg->target_base,
                               msg->target_base_x_error,
                               msg->target_base_y_error,
                               0.7f);  // o alvo e sempre confiante (match exato)
        }
    }

    void executeFSM() {
        // ConfirmNumberState nao publica ROS diretamente (estados so' tem
        // Drone/blackboard) -- so' seta esse flag em on_enter(); quem
        // publica de fato o pedido de leitura de digito e' o No.
        bool* ocr_pending_ptr = fsm_->blackboard_get<bool>("ocr_request_pending");
        if (ocr_pending_ptr && *ocr_pending_ptr) {
            ocr_request_pub_->publish(std_msgs::msg::Empty());
            fsm_->blackboard_set<bool>("ocr_request_pending", false);
            RCLCPP_INFO(this->get_logger(), "[OCR] pedido de leitura de numero disparado");
        }

        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp      = this->now();
        ps.header.frame_id   = "map";
        ps.pose.position.x   =  static_cast<float>(pos.y());   // East  = NED y
        ps.pose.position.y   =  static_cast<float>(pos.x());   // North = NED x
        ps.pose.position.z   = -static_cast<float>(pos.z());   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        // Marcadores das bases no RViz2 -- republica a cada 2s (array vazio
        // se nenhuma base foi vista ainda).
        if (pos_log_counter_ % 40 == 0) {
            visualization_msgs::msg::MarkerArray ma;
            std::string* target_label = fsm_->blackboard_get<std::string>("target_base");
            int mid = 0;
            for (const auto& lbl : known_base_labels_) {
                float* kx = fsm_->blackboard_get<float>("known_base_" + lbl + "_x");
                float* ky = fsm_->blackboard_get<float>("known_base_" + lbl + "_y");
                float* kc = fsm_->blackboard_get<float>("known_base_" + lbl + "_conf");
                if (!kx || !ky) continue;
                bool is_target = target_label && (*target_label == lbl);

                float cr = 0.5f, cg = 0.5f, cb = 1.0f;  // azul (padrao)
                if (lbl.find("TRIANGULO") != std::string::npos) { cr=1.0f; cg=0.2f; cb=0.2f; }
                else if (lbl.find("ESTRELA")   != std::string::npos) { cr=1.0f; cg=0.9f; cb=0.0f; }
                if (is_target) { cr=0.0f; cg=1.0f; cb=0.3f; }  // alvo = verde

                visualization_msgs::msg::Marker sphere;
                sphere.header.frame_id = "map";
                sphere.header.stamp    = this->now();
                sphere.ns = "base_spheres";
                sphere.id = mid++;
                sphere.type   = visualization_msgs::msg::Marker::SPHERE;
                sphere.action = visualization_msgs::msg::Marker::ADD;
                sphere.pose.position.x = *ky;   // ENU East  = NED y
                sphere.pose.position.y = *kx;   // ENU North = NED x
                sphere.pose.position.z = 0.05f;
                sphere.pose.orientation.w = 1.0;
                sphere.scale.x = sphere.scale.y = sphere.scale.z = is_target ? 0.4f : 0.25f;
                sphere.color.r = cr; sphere.color.g = cg; sphere.color.b = cb;
                sphere.color.a = kc ? (0.4f + *kc * 0.6f) : 0.7f;
                sphere.lifetime = rclcpp::Duration(0, 0);
                ma.markers.push_back(sphere);

                visualization_msgs::msg::Marker txt;
                txt.header = sphere.header;
                txt.ns = "base_labels";
                txt.id = mid++;
                txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                txt.action = visualization_msgs::msg::Marker::ADD;
                txt.pose.position.x = sphere.pose.position.x;
                txt.pose.position.y = sphere.pose.position.y;
                txt.pose.position.z = 0.40f;
                txt.pose.orientation.w = 1.0;
                txt.scale.z = 0.20f;
                txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
                txt.text = lbl + (kc ? (" c=" + std::to_string(*kc).substr(0,4)) : "");
                txt.lifetime = rclcpp::Duration(0, 0);
                ma.markers.push_back(txt);
            }
            base_markers_pub_->publish(ma);
        }

        // Log de estado e posicao a cada 2s (40 ticks a 20 Hz).
        if (pos_log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] pos=(%.2f,%.2f,%.2f) yaw=%.2f rad",
                fsm_->get_current_state().c_str(),
                static_cast<float>(pos.x()), static_cast<float>(pos.y()),
                static_cast<float>(pos.z()), static_cast<float>(orient[2]));
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
        const std::map<std::string, std::variant<double, std::string>>& defaults) {

        std::map<std::string, std::variant<double, std::string>> result;

        for (const auto& [name, default_value] : defaults) {
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
    std::unique_ptr<Fase2ItjbxFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::BouncingDetection>::SharedPtr cv_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ocr_request_pub_;
    rclcpp::Subscription<custom_msgs::msg::ReadBaseNumberResult>::SharedPtr ocr_result_sub_;

    bool  prev_aruco_detected_       = false;
    bool  prev_target_base_in_sight_ = false;
    bool  prev_target_calculated_    = false;
    float aruco_smoothed_x_          = 0.0f;
    float aruco_smoothed_y_          = 0.0f;
    bool  aruco_first_detection_     = true;
    float aruco_world_x_             = 0.0f;
    float aruco_world_y_             = 0.0f;
    bool  aruco_world_valid_         = false;
    int   pos_log_counter_           = 0;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr discovered_bases_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr base_markers_pub_;
    nav_msgs::msg::Path trajectory_;
    std::set<std::string> known_base_labels_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // O Drone ja sobe o proprio executor e a propria thread de spin
    // (drone_lib/src/Drone.cpp) -- adiciona-lo a um executor aqui lanca em
    // tempo de execucao ("Node '/Drone' has already been added to an
    // executor."). So o no da missao entra no executor deste main.
    auto drone = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Fase2ItjbxNode>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
