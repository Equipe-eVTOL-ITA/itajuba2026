#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

// Gira até a linha aparecer em /lane_detection ("line_detected" no
// blackboard). Três modos, escolhidos a cada ciclo:
//
//   1. Vendo a base circular com o vetor de saída da linha disponível
//      (line_is_circle + line_circle_exit_valid): alinha proporcionalmente
//      esse vetor com "pra frente" (exit_theta -> 0) antes de liberar
//      FOLLOW_LINE, já que a linha só sai de um lado da base.
//   2. Base não aparece mais mas a linha já foi detectada (line_detected):
//      libera direto.
//   3. Nenhum dos dois sinais disponível: varre um arco em torno da direção
//      de decolagem (não 360°, pra não arriscar achar a linha "de costas" e
//      seguir a pista no sentido errado).
//
// Estoura o timeout -> vai pousar em segurança.
class SearchLineState : public fsm::State {
public:
    SearchLineState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        drone_ = *bb.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH LINE");

        timeout_s_ = *bb.get<float>("search_timeout");
        search_yaw_rate_ = bb.contains("search_yaw_rate")
            ? *bb.get<float>("search_yaw_rate") : 0.3f;
        search_sweep_angle_ = bb.contains("search_sweep_angle")
            ? *bb.get<float>("search_sweep_angle") : 0.6f;

        max_align_yaw_rate_ = bb.contains("max_align_yaw_rate")
            ? *bb.get<float>("max_align_yaw_rate") : 0.8f;
        align_tolerance_rad_ = bb.contains("align_tolerance_rad")
            ? *bb.get<float>("align_tolerance_rad") : 0.1745f;

        float align_kp = bb.contains("align_kp") ? *bb.get<float>("align_kp") : 0.9f;
        float align_ki = bb.contains("align_ki") ? *bb.get<float>("align_ki") : 0.0f;
        float align_kd = bb.contains("align_kd") ? *bb.get<float>("align_kd") : 0.1f;
        // setpoint 0: exit_theta convergindo pra zero (vetor circulo->linha
        // apontando "pra frente").
        align_pid_ = std::make_unique<PidController>(align_kp, align_ki, align_kd, 0.0f);

        // exit_theta e' instavel frame-a-frame (HoughCircles), e
        // PidController::compute() usa derivada crua sem filtro -- EMA aqui
        // suaviza o sinal antes do PID amplificar o flicker em tremor de yaw_rate.
        exit_theta_ema_alpha_ = bb.contains("exit_theta_ema_alpha")
            ? *bb.get<float>("exit_theta_ema_alpha") : 0.3f;
        has_smoothed_exit_theta_ = false;

        // Retoma a busca pro mesmo lado que o controle angular já estava
        // corrigindo quando a linha sumiu, em vez de sempre reiniciar pra +1.
        float last_theta = bb.contains("last_seen_line_theta")
            ? *bb.get<float>("last_seen_line_theta") : 0.0f;
        search_dir_ = (last_theta > 0.0f) ? -1.0f : 1.0f;

        start_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& bb) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_circle = bb.contains("line_is_circle") && *bb.get<bool>("line_is_circle");
        bool exit_valid = bb.contains("line_circle_exit_valid") && *bb.get<bool>("line_circle_exit_valid");

        if (is_circle && exit_valid) {
            // Alinha proporcionalmente (exit_theta -> 0) em vez de girar a taxa
            // fixa; so' libera FOLLOW_LINE dentro da tolerancia.
            float exit_theta = *bb.get<float>("line_circle_exit_theta");

            // Primeira leitura entra direto (senao o filtro comecaria de 0 e o
            // drone giraria na direcao errada por um instante).
            if (!has_smoothed_exit_theta_) {
                smoothed_exit_theta_ = exit_theta;
                has_smoothed_exit_theta_ = true;
            } else {
                smoothed_exit_theta_ = exit_theta_ema_alpha_ * exit_theta
                    + (1.0f - exit_theta_ema_alpha_) * smoothed_exit_theta_;
            }

            // negado: sinal "cru" do PidController girava pro lado errado pra esse vetor.
            float yaw_rate = -align_pid_->compute(smoothed_exit_theta_);
            yaw_rate = std::clamp(yaw_rate, -max_align_yaw_rate_, max_align_yaw_rate_);
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, yaw_rate);

            if (std::fabs(smoothed_exit_theta_) < align_tolerance_rad_) {
                drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                return "LINE FOUND";
            }
        } else if (*bb.get<bool>("line_detected")) {
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "LINE FOUND";
        } else {
            // getOrientation()[2] já e' o yaw relativo ao heading de decolagem
            // (0 no takeoff), entao serve direto como limite do arco.
            float yaw = drone_->getOrientation()[2];
            if (yaw > search_sweep_angle_) {
                search_dir_ = -1.0f;
            } else if (yaw < -search_sweep_angle_) {
                search_dir_ = 1.0f;
            }
            drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, search_dir_ * search_yaw_rate_);
        }

        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_).count();
        if (elapsed > timeout_s_) {
            drone_->log("SEARCH LINE: timeout, indo pousar");
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
    float search_yaw_rate_ = 0.3f;
    float search_sweep_angle_ = 0.6f;
    float search_dir_ = 1.0f;
    float timeout_s_ = 15.0f;
    std::chrono::steady_clock::time_point start_;

    std::unique_ptr<PidController> align_pid_;
    float max_align_yaw_rate_ = 0.8f;
    float align_tolerance_rad_ = 0.1745f;

    float exit_theta_ema_alpha_ = 0.3f;
    float smoothed_exit_theta_ = 0.0f;
    bool has_smoothed_exit_theta_ = false;
};
