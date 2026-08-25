#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

// Segunda metade do alinhamento com a mangueira vermelha -- entra depois de
// ApproachRedLineState ja ter colocado o drone em cima da linha.
//
// O GIRO e' SEM PID e SO' UMA medicao: mede align_red_theta_samples leituras
// de red_line_theta, tira a MEDIANA (ver _median), calcula o yaw alvo UMA vez
// (theta e' relativo a VERTICAL -- 0 = mangueira vertical no quadro = alinhado,
// mesma convencao do lane_detector) e avanca em passos pequenos na direcao
// dele a cada tick ("little yaw", capado por align_red_max_yaw_velocity
// rad/s). Nenhuma leitura de red_line_theta acontece mais depois disso.
//
// A TRANSLACAO (vx/vy) roda o tempo todo durante o giro, com PID + EMA (mesmo
// padrao do FollowLineState/SearchLineState): girar em torno do proprio eixo
// desloca o que a camera enxerga, entao x_centroid/y_centroid continuam sendo
// corrigidos pra manter a mangueira centralizada.
//
// Yaw e translacao saem juntos num so' setLocalVelocity por tick, porque
// setLocalPosition (yaw como setpoint) e setLocalVelocity (yaw como yawspeed)
// sao modos de offboard mutuamente exclusivos no Drone -- por isso o "little
// yaw" e' expresso como taxa (yaw_rate = clamp(yaw_error/kDt_, ±max_yaw_velocity_)).
//
// Depois de girar e assentar dentro da tolerancia por align_red_rotate_settle_s
// seguidos, volta pra base (ReturnHomeState) em vez de pousar em cima da
// mangueira -- ver transicoes em fase4_itjbx.cpp.
class AlignRedLineState : public fsm::State {
public:
    AlignRedLineState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        drone_ = *bb.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: ALIGN RED LINE");

        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        timeout_s_ = bb.contains("align_red_timeout")
            ? *bb.get<float>("align_red_timeout") : 20.0f;
        // 0.0873 rad ~= 5 graus. Se a mediana ja vier dentro disso, nem gira.
        tolerance_rad_ = bb.contains("align_red_tolerance_rad")
            ? *bb.get<float>("align_red_tolerance_rad") : 0.0873f;
        samples_needed_ = bb.contains("align_red_theta_samples")
            ? static_cast<int>(*bb.get<float>("align_red_theta_samples")) : 20;
        // Quao perto do yaw alvo o PROPRIO drone (nao a visao) precisa chegar
        // pra considerarmos o giro concluido.
        yaw_reach_tolerance_rad_ = bb.contains("align_red_yaw_reach_tolerance_rad")
            ? *bb.get<float>("align_red_yaw_reach_tolerance_rad") : 0.035f;
        // Teto de velocidade do giro (rad/s) -- ver "little yaw" em actRotating().
        max_yaw_velocity_ = bb.contains("align_red_max_yaw_velocity")
            ? *bb.get<float>("align_red_max_yaw_velocity") : 0.4f;
        // Fica DENTRO de yaw_reach_tolerance_rad_, parado, por esse tempo
        // seguido antes de aceitar -- da tempo do overshoot/inercia do PX4
        // amortecer, em vez de aceitar no instante que so' passa perto.
        rotate_settle_s_ = bb.contains("align_red_rotate_settle_s")
            ? *bb.get<float>("align_red_rotate_settle_s") : 0.8f;

        // Correcao de translacao (vx/vy) durante o giro -- ver comentario da
        // classe pro' porque disto e pro' porque tem EMA antes do PID.
        float translation_kp = bb.contains("align_red_translation_kp")
            ? *bb.get<float>("align_red_translation_kp") : 0.0025f;
        float translation_ki = bb.contains("align_red_translation_ki")
            ? *bb.get<float>("align_red_translation_ki") : 0.0f;
        float translation_kd = bb.contains("align_red_translation_kd")
            ? *bb.get<float>("align_red_translation_kd") : 0.001f;
        max_translation_velocity_ = bb.contains("align_red_max_translation_velocity")
            ? *bb.get<float>("align_red_max_translation_velocity") : 0.3f;
        translation_ema_alpha_ = bb.contains("align_red_translation_ema_alpha")
            ? *bb.get<float>("align_red_translation_ema_alpha") : 0.3f;

        // setpoint 0: queremos x_centroid e y_centroid convergindo pra zero
        // (mangueira centralizada no quadro).
        forward_pid_ = std::make_unique<PidController>(translation_kp, translation_ki, translation_kd, 0.0f);
        lateral_pid_ = std::make_unique<PidController>(translation_kp, translation_ki, translation_kd, 0.0f);
        has_smoothed_translation_ = false;

        phase_ = Phase::Measuring;
        theta_samples_.clear();
        reached_since_valid_ = false;

        start_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& bb) override {
        if (drone_ == nullptr) return "ERROR";

        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_).count();
        if (elapsed > timeout_s_) {
            drone_->log("ALIGN RED LINE: timeout, voltando pra base");
            return "TIMEOUT";
        }

        if (phase_ == Phase::Measuring) {
            return actMeasuring(bb);
        }
        return actRotating(bb);
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void)bb;
        if (drone_ != nullptr) drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

private:
    enum class Phase { Measuring, Rotating };

    std::string actMeasuring(fsm::Blackboard& bb) {
        // Parado enquanto mede -- nao faz sentido girar antes de saber, com
        // confianca, pra que lado.
        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        bool detected = bb.contains("red_line_detected") && *bb.get<bool>("red_line_detected");
        if (detected) {
            theta_samples_.push_back(*bb.get<float>("red_line_theta"));
        }

        if (static_cast<int>(theta_samples_.size()) < samples_needed_) {
            return "";
        }

        float filtered_theta = _median(theta_samples_);
        theta_samples_.clear();
        drone_->log("ALIGN RED LINE: angulo medido (mediana de " +
                    std::to_string(samples_needed_) + " leituras) = " +
                    std::to_string(filtered_theta) + " rad");

        if (std::fabs(filtered_theta) < tolerance_rad_) {
            drone_->log("ALIGN RED LINE: ja dentro da tolerancia, sem girar");
            return "ALIGNED";
        }

        // Calcula o yaw alvo UMA vez so'. Δyaw = -theta_medido (sinal
        // confirmado em teste real; mesma convencao do FollowLineState).
        float current_yaw = static_cast<float>(drone_->getOrientation()[2]);
        target_yaw_ = _wrapAngle(current_yaw - filtered_theta);
        reached_since_valid_ = false;

        drone_->log("ALIGN RED LINE: girando suavemente ate yaw=" + std::to_string(target_yaw_) +
                    " rad, corrigindo translacao por visao ate' la'");
        phase_ = Phase::Rotating;
        return "";
    }

    std::string actRotating(fsm::Blackboard& bb) {
        // target_yaw_ nunca muda aqui -- so' recalculado em actMeasuring().
        float current_yaw = static_cast<float>(drone_->getOrientation()[2]);
        float yaw_error = _wrapAngle(target_yaw_ - current_yaw);
        float yaw_rate = std::clamp(yaw_error / kDt_, -max_yaw_velocity_, max_yaw_velocity_);

        // Sem deteccao no frame, so' segura (0,0) em vez de corrigir com um
        // erro que nao existe.
        float vx = 0.0f;
        float vy = 0.0f;
        bool detected = bb.contains("red_line_detected") && *bb.get<bool>("red_line_detected");
        if (detected) {
            float x_centroid = static_cast<float>(*bb.get<int>("red_line_x_centroid"));
            float y_centroid = static_cast<float>(*bb.get<int>("red_line_y_centroid"));

            if (!has_smoothed_translation_) {
                smoothed_x_centroid_ = x_centroid;
                smoothed_y_centroid_ = y_centroid;
                has_smoothed_translation_ = true;
            } else {
                smoothed_x_centroid_ = translation_ema_alpha_ * x_centroid
                    + (1.0f - translation_ema_alpha_) * smoothed_x_centroid_;
                smoothed_y_centroid_ = translation_ema_alpha_ * y_centroid
                    + (1.0f - translation_ema_alpha_) * smoothed_y_centroid_;
            }

            // vx direto (mesma convencao do ApproachRedLineState); vy negado
            // (mesma convencao do lateral_pid_ do FollowLineState).
            vx = std::clamp(forward_pid_->compute(smoothed_y_centroid_),
                             -max_translation_velocity_, max_translation_velocity_);
            vy = std::clamp(-lateral_pid_->compute(smoothed_x_centroid_),
                             -max_translation_velocity_, max_translation_velocity_);
        } else {
            has_smoothed_translation_ = false;
        }

        drone_->setLocalVelocity(vx, vy, 0.0f, yaw_rate);

        bool reached = std::fabs(yaw_error) < yaw_reach_tolerance_rad_;

        auto now = std::chrono::steady_clock::now();

        if (reached) {
            if (!reached_since_valid_) {
                reached_since_ = now;
                reached_since_valid_ = true;
            }
            float reached_for = std::chrono::duration<float>(now - reached_since_).count();
            if (reached_for >= rotate_settle_s_) {
                drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                drone_->log("ALIGN RED LINE: alinhado e parado");
                return "ALIGNED";
            }
        } else {
            reached_since_valid_ = false;
        }

        return "";
    }

    static float _median(std::vector<float> values) {
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        return n % 2 == 0 ? (values[n / 2 - 1] + values[n / 2]) / 2.0f : values[n / 2];
    }

    /// Normaliza um angulo (rad) pro intervalo (-pi, pi].
    static float _wrapAngle(float angle) {
        while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
        while (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
        return angle;
    }

    std::shared_ptr<Drone> drone_;

    float timeout_s_ = 20.0f;
    float tolerance_rad_ = 0.0873f;
    std::chrono::steady_clock::time_point start_;

    Phase phase_ = Phase::Measuring;
    int samples_needed_ = 20;
    std::vector<float> theta_samples_;

    float target_yaw_ = 0.0f;
    float yaw_reach_tolerance_rad_ = 0.035f;
    float rotate_settle_s_ = 0.8f;
    bool reached_since_valid_ = false;
    std::chrono::steady_clock::time_point reached_since_;

    // Teto de velocidade do "little yaw" em actRotating() (rad/s).
    float max_yaw_velocity_ = 0.4f;
    // Periodo do tick do FSM (20 Hz, ver fase4_itjbx.cpp) -- usado so' pra'
    // converter max_yaw_velocity_ numa taxa equivalente ao passo por tick.
    static constexpr float kDt_ = 0.05f;

    // Correcao de translacao (vx/vy) durante o giro -- ver comentario da classe.
    std::unique_ptr<PidController> forward_pid_;  // vx a partir de y_centroid
    std::unique_ptr<PidController> lateral_pid_;  // vy a partir de x_centroid
    float max_translation_velocity_ = 0.3f;
    float translation_ema_alpha_ = 0.3f;
    float smoothed_x_centroid_ = 0.0f;
    float smoothed_y_centroid_ = 0.0f;
    bool has_smoothed_translation_ = false;
};
