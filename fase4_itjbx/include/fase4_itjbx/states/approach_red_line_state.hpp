#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

// Primeira metade do alinhamento com a mangueira vermelha (ver AlignRedLineState
// pra segunda metade): antes de girar, corrige so' vx a partir de
// red_line_y_centroid ate o centro do quadro cair em cima da reta detectada.
//
// vx = +correction, sem negar (TESTADO EM SIMULACAO): red_line_y_centroid > 0
// significa a mangueira um pouco pra FRENTE, ao contrario do que a analogia
// com o lane_detector sugeria.
class ApproachRedLineState : public fsm::State {
public:
    ApproachRedLineState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        drone_ = *bb.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: APPROACH RED LINE");

        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        timeout_s_ = bb.contains("approach_red_timeout")
            ? *bb.get<float>("approach_red_timeout") : 20.0f;
        tolerance_px_ = bb.contains("approach_red_tolerance_px")
            ? *bb.get<float>("approach_red_tolerance_px") : 15.0f;
        max_forward_velocity_ = bb.contains("max_approach_red_forward_velocity")
            ? *bb.get<float>("max_approach_red_forward_velocity") : 0.3f;

        float kp = bb.contains("approach_red_forward_kp") ? *bb.get<float>("approach_red_forward_kp") : 0.0025f;
        float ki = bb.contains("approach_red_forward_ki") ? *bb.get<float>("approach_red_forward_ki") : 0.0f;
        float kd = bb.contains("approach_red_forward_kd") ? *bb.get<float>("approach_red_forward_kd") : 0.001f;

        // setpoint 0: queremos red_line_y_centroid convergindo pra zero (reta
        // cruzando exatamente pelo centro vertical do quadro).
        forward_pid_ = std::make_unique<PidController>(kp, ki, kd, 0.0f);

        start_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& bb) override {
        if (drone_ == nullptr) return "ERROR";

        bool detected = bb.contains("red_line_detected") && *bb.get<bool>("red_line_detected");

        if (!detected) {
            // Mangueira momentaneamente fora do quadro -- espera parado em vez de
            // corrigir com um erro que nao existe. Se nunca mais aparecer, o
            // timeout cuida.
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
        } else {
            int y_centroid = *bb.get<int>("red_line_y_centroid");

            float forward_correction = forward_pid_->compute(static_cast<float>(y_centroid));
            forward_correction = std::clamp(forward_correction, -max_forward_velocity_, max_forward_velocity_);
            float vx = forward_correction;

            drone_->setLocalVelocity(vx, 0.0f, 0.0f, 0.0f);

            if (std::abs(y_centroid) < tolerance_px_) {
                drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                drone_->log("APPROACH RED LINE: em cima da linha");
                return "ABOVE LINE";
            }
        }

        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_).count();
        if (elapsed > timeout_s_) {
            drone_->log("APPROACH RED LINE: timeout, indo pousar");
            return "TIMEOUT";
        }

        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void)bb;
        if (drone_ != nullptr) drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

private:
    std::shared_ptr<Drone> drone_;
    std::unique_ptr<PidController> forward_pid_;
    float timeout_s_ = 20.0f;
    float tolerance_px_ = 15.0f;
    float max_forward_velocity_ = 0.3f;
    std::chrono::steady_clock::time_point start_;
};
