#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Portado de sae2026/mission_1 (GoToBaseState) sem alteracoes de logica —
 * PD visual servo sobre o erro normalizado da base alvo
 * (target_base_x_error/y_error).
 */
class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State(),
        kd_x_(0.0f), kd_y_(0.0f), entry_z_(0.0f), cam_scale_(0.7f),
        err_x_prev_(0.0f), err_y_prev_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GO_TO_BASE");

        if (blackboard.contains("base_tolerance")) {
            tolerance_ = *blackboard.get<float>("base_tolerance");
        } else {
            tolerance_ = 0.05f;
        }

        kp_x_ = blackboard.get<float>("base_kp_x") ? *blackboard.get<float>("base_kp_x") : 0.5f;
        kp_y_ = blackboard.get<float>("base_kp_y") ? *blackboard.get<float>("base_kp_y") : 0.5f;

        cam_scale_ = blackboard.contains("base_cam_scale")
            ? *blackboard.get<float>("base_cam_scale") : 0.7f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());

        kd_x_ = blackboard.get<float>("base_kd_x") ? *blackboard.get<float>("base_kd_x") : 0.0f;
        kd_y_ = blackboard.get<float>("base_kd_y") ? *blackboard.get<float>("base_kd_y") : 0.0f;
        err_x_prev_ = 0.0f;
        err_y_prev_ = 0.0f;

        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_         = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("target_base_in_sight")) {
            is_detected = *blackboard.get<bool>("target_base_in_sight");
        }

        if (!is_detected) {
            miss_counter_++;
            if (miss_counter_ > max_misses_) {
                drone_->log("Base lost! Returning to search.");
                return "BASE_LOST";
            }

            // Predicao cinematica (mesmo esquema do GoToArucoState): a base e'
            // fixa no chao, entao sua posicao aparente na camera se move na
            // direcao oposta a velocidade do drone. Sem isso, um lapso curto
            // de deteccao (blur, sombra, ruido de grama) fazia o drone travar
            // parado ate' completar max_misses_ e jogar fora todo o progresso
            // de alinhamento -- descartando uma base ja identificada como a
            // certa e voltando pra busca do zero. Continuar seguindo a
            // predicao mantem a aproximacao viva durante o lapso.
            auto vel = drone_->getLocalVelocity();   // FRD frame
            float alt = std::max(0.3f, -static_cast<float>(drone_->getLocalPosition().z()));
            constexpr float dt = 0.05f;
            err_x_prev_ = std::clamp(err_x_prev_ - static_cast<float>(vel.y()) / (alt * cam_scale_) * dt, -1.0f, 1.0f);
            err_y_prev_ = std::clamp(err_y_prev_ - static_cast<float>(vel.x()) / (alt * cam_scale_) * dt, -1.0f, 1.0f);

            float max_v = 1.0f;
            float vx = std::clamp(-err_y_prev_ * kp_x_, -max_v * 0.5f, max_v * 0.5f);
            float vy = std::clamp( err_x_prev_ * kp_y_, -max_v * 0.5f, max_v * 0.5f);
            move_local_by_speed(drone_, vx, vy, 0.0f);
            return "";
        }

        miss_counter_ = 0;

        float err_x = *blackboard.get<float>("target_base_x_error");
        float err_y = *blackboard.get<float>("target_base_y_error");

        float vx = 0.0f, vy = 0.0f;
        float max_v = 1.0f;

        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        vx = -(err_y * kp_x_ + d_err_y * kd_x_);
        vy =  (err_x * kp_y_ + d_err_x * kd_y_);
        vx = std::clamp(vx, -max_v, max_v);
        vy = std::clamp(vy, -max_v, max_v);

        // vz=0: PX4 holds altitude natively in velocity-setpoint mode.
        move_local_by_speed(drone_, vx, vy, 0.0f);

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("Base err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                        + ") aligned=" + std::to_string(aligned_counter_) + "/20");

        if (aligned_counter_ > 10) {
            drone_->log("Aligned with Base! Ready to land.");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_;
    float kp_x_, kp_y_;
    float kd_x_, kd_y_;
    float entry_z_;
    float cam_scale_;
    float err_x_prev_, err_y_prev_;
    int aligned_counter_;
    int miss_counter_;
    int tick_;
    // 3s at 20Hz -- alargado de 1.5s (30) agora que a predicao cinematica
    // mantem a aproximacao ativa durante o lapso (ver bloco !is_detected em
    // act()); desistir cedo demais custava a busca inteira de novo.
    static constexpr int max_misses_ = 60;
};
