#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <std_msgs/msg/empty.hpp>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"
#include "drone/transformations.hpp"

// Segue a linha usando os dados de /lane_detection (custom_msgs/msg/LaneDirection),
// já traduzidos pro blackboard em fase4_itjbx.cpp:
//   line_detected   (bool)  — false quando msg->lost == true
//   line_theta      (float) — heading error da curva na base da imagem (rad);
//                              ~0 = linha reta à frente, sinal indica pra que lado ela curva
//   line_x_centroid (int)   — offset lateral do lookahead point em relação ao centro (px)
//   line_y_centroid (int)   — y médio dos pontos detectados (px, relativo ao centro)
//   line_area       (int)   — pixels azuis na máscara (proxy de confiança)
//   line_is_circle  (bool)  — true se o blob for circular (ex: marcador de base) — não usado aqui ainda
//
// Controle: PID lateral (a partir de line_x_centroid) corrige vy; PID angular
// (a partir de line_theta) corrige yaw_rate. Se o ângulo da curva passar de
// max_angle_for_translation, o drone só gira no lugar (vx=0) em vez de andar
// pra frente, pra não cortar caminho pra fora de curvas fechadas.
//
// Sai daqui de duas formas: "LINE LOST" (linha azul sumiu, volta a procurar)
// ou "RED LINE FOUND" (achou a mangueira vermelha entre os postes -- o
// red_line_detector escreve red_line_detected/theta/x_centroid/y_centroid
// na blackboard, ver fase4_itjbx.cpp; esse nó precisa estar rodando desde o
// início da missão, não só a partir daqui, senão o quadro em que a mangueira
// aparece pela primeira vez nunca é visto).
class FollowLineState : public fsm::State {
public:
    FollowLineState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        drone_ = *bb.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: FOLLOW LINE");

        // Publisher do aviso de encerramento do circle_detector -- publicado
        // com atraso, ver circle_shutdown_delay_s_ em act(). reliable +
        // transient_local: a mensagem fica retida ate' o assinante conectar.
        if (!circle_shutdown_pub_) {
            rclcpp::QoS shutdown_qos(1);
            shutdown_qos.reliable().transient_local();
            circle_shutdown_pub_ = drone_->create_publisher<std_msgs::msg::Empty>(
                "/circle_detector/shutdown", shutdown_qos);
        }
        circle_shutdown_sent_ = false;
        circle_shutdown_delay_s_ = bb.contains("circle_shutdown_delay_s")
            ? *bb.get<float>("circle_shutdown_delay_s") : 2.0f;

        max_velocity_ = *bb.get<float>("max_horizontal_velocity");
        timeout_s_ = *bb.get<float>("follow_timeout");

        max_yaw_rate_ = bb.contains("max_yaw_rate")
            ? *bb.get<float>("max_yaw_rate") : 0.8f;
        max_angle_for_translation_ = bb.contains("max_angle_for_translation")
            ? *bb.get<float>("max_angle_for_translation") : 0.6f;

        float kp_lat = bb.contains("pid_lateral_kp") ? *bb.get<float>("pid_lateral_kp") : 0.0025f;
        float ki_lat = bb.contains("pid_lateral_ki") ? *bb.get<float>("pid_lateral_ki") : 0.0f;
        float kd_lat = bb.contains("pid_lateral_kd") ? *bb.get<float>("pid_lateral_kd") : 0.001f;
        float kp_ang = bb.contains("pid_angular_kp") ? *bb.get<float>("pid_angular_kp") : 0.9f;
        float ki_ang = bb.contains("pid_angular_ki") ? *bb.get<float>("pid_angular_ki") : 0.0f;
        float kd_ang = bb.contains("pid_angular_kd") ? *bb.get<float>("pid_angular_kd") : 0.15f;

        // setpoint 0: queremos x_centroid e theta convergindo pra zero (linha centralizada e reta à frente)
        lateral_pid_ = std::make_unique<PidController>(kp_lat, ki_lat, kd_lat, 0.0f);
        angular_pid_ = std::make_unique<PidController>(kp_ang, ki_ang, kd_ang, 0.0f);

        // EMA aqui, antes do PID: PidController::compute() usa derivada crua
        // sem filtro, e em curvas fechadas qualquer flicker do lane_detector
        // vira tremor amplificado no yaw_rate.
        theta_ema_alpha_ = bb.contains("follow_theta_ema_alpha")
            ? *bb.get<float>("follow_theta_ema_alpha") : 0.3f;
        has_smoothed_theta_ = false;

        start_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& bb) override {
        if (drone_ == nullptr) return "ERROR";

        // Avisa o circle_detector que pode se encerrar, mas so' depois de
        // circle_shutdown_delay_s_: um desligamento imediato deixaria o
        // lane_detector recortando a base com a ULTIMA posicao conhecida
        // (congelada) bem no momento em que o drone comeca a se mover de
        // verdade, gerando theta/x_centroid ruidosos.
        float elapsed_since_enter = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start_).count();
        if (!circle_shutdown_sent_ && elapsed_since_enter >= circle_shutdown_delay_s_) {
            circle_shutdown_pub_->publish(std_msgs::msg::Empty());
            circle_shutdown_sent_ = true;
        }

        // Checado antes de "line_detected" de proposito: a mangueira e' o
        // alvo final, a linha azul e' so' o caminho ate ela.
        if (bb.contains("red_line_detected") && *bb.get<bool>("red_line_detected")) {
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "RED LINE FOUND";
        }

        if (!*bb.get<bool>("line_detected")) {
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "LINE LOST";
        }

        float theta = *bb.get<float>("line_theta");
        int x_centroid = *bb.get<int>("line_x_centroid");

        if (!has_smoothed_theta_) {
            smoothed_theta_ = theta;
            has_smoothed_theta_ = true;
        } else {
            smoothed_theta_ = theta_ema_alpha_ * theta + (1.0f - theta_ema_alpha_) * smoothed_theta_;
        }

        float lateral_correction = lateral_pid_->compute(static_cast<float>(x_centroid));
        float angular_correction = angular_pid_->compute(smoothed_theta_);
        lateral_correction = std::clamp(lateral_correction, -max_velocity_, max_velocity_);
        angular_correction = std::clamp(angular_correction, -max_yaw_rate_, max_yaw_rate_);

        float vy = -lateral_correction;
        float yaw_rate = angular_correction;

        // vx cai suavemente com |theta| em vez de ligar/desligar no threshold:
        // evita bang-bang e reduz velocidade naturalmente em curvas fechadas.
        float speed_scale = std::clamp(1.0f - std::abs(smoothed_theta_) / max_angle_for_translation_, 0.0f, 1.0f);
        float vx = max_velocity_ * speed_scale;

        // (vx, vy) são relativos ao heading atual do drone, não ao heading inicial
        Eigen::Vector3d local_velocity(vx, vy, 0.0);
        local_velocity = adjust_velocity_using_yaw(local_velocity, drone_->getOrientation()[2]);
        drone_->setLocalVelocity(
            static_cast<float>(local_velocity.x()),
            static_cast<float>(local_velocity.y()),
            static_cast<float>(local_velocity.z()),
            yaw_rate);

        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_).count();
        if (elapsed > timeout_s_) {
            drone_->log("FOLLOW LINE: timeout, indo pousar");
            return "TIMEOUT";
        }

        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void)bb;
        if (drone_ != nullptr) drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        // Rede de seguranca: se saiu antes do atraso terminar, manda o aviso
        // agora mesmo assim.
        if (!circle_shutdown_sent_ && circle_shutdown_pub_) {
            circle_shutdown_pub_->publish(std_msgs::msg::Empty());
            circle_shutdown_sent_ = true;
        }
    }

private:
    std::shared_ptr<Drone> drone_;
    std::unique_ptr<PidController> lateral_pid_;
    std::unique_ptr<PidController> angular_pid_;
    float max_velocity_ = 0.0f;
    float max_yaw_rate_ = 0.8f;
    float max_angle_for_translation_ = 0.6f;
    float timeout_s_ = 60.0f;
    std::chrono::steady_clock::time_point start_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr circle_shutdown_pub_;
    bool circle_shutdown_sent_ = false;
    float circle_shutdown_delay_s_ = 2.0f;

    float theta_ema_alpha_ = 0.3f;
    float smoothed_theta_ = 0.0f;
    bool has_smoothed_theta_ = false;
};
