#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Procura o marcador ArUco com uma varredura retangular: o drone visita os
 * 4 cantos de um retangulo em expansao centrado em onde entrou neste
 * estado, crescendo por "aruco_search_step" a cada volta. Mesmo algoritmo
 * de RetangularSearchState (fase1_itjbx), adaptado pra manter o filtro de
 * persistencia/miss-tolerance de deteccao de ArUco (portado de
 * sae2026/mission_1::SearchArucoState).
 *
 * Blackboard reads:
 *   "aruco_search_abs_x"/"aruco_search_abs_y" (float) — meia-extensao do
 *                                retangulo final (default 6.0 m cada)
 *   "aruco_search_step"        (float) — crescimento por volta (default 1.0 m)
 *   "corner_decel_radius"      (float) — raio de freio em cada canto (default 1.0 m)
 *   "corner_min_velocity"      (float) — velocidade minima dentro desse raio (default 0.3 m/s)
 *   "search_aruco_velocity"    (float) — velocidade de cruzeiro (default 0.5 m/s)
 *   "aruco_persistence_frames" (float) — frames pra confirmar o ArUco (default 3)
 *
 * Retorna: "ARUCO_FOUND"
 */
class SearchArucoState : public fsm::State {
public:
    SearchArucoState() : fsm::State(),
        contador_(1), laps_(0),
        detection_counter_(0), miss_counter_(0), last_detected_id_(-1),
        min_detections_(3), max_misses_(3) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_ARUCO (Rectangular Search)");

        abs_x_ = blackboard.contains("aruco_search_abs_x")
            ? *blackboard.get<float>("aruco_search_abs_x") : 6.0f;
        abs_y_ = blackboard.contains("aruco_search_abs_y")
            ? *blackboard.get<float>("aruco_search_abs_y") : 6.0f;
        step_size_ = blackboard.contains("aruco_search_step")
            ? *blackboard.get<float>("aruco_search_step") : 1.0f;
        corner_decel_radius_ = blackboard.contains("corner_decel_radius")
            ? *blackboard.get<float>("corner_decel_radius") : 1.0f;
        corner_min_velocity_ = blackboard.contains("corner_min_velocity")
            ? *blackboard.get<float>("corner_min_velocity") : 0.3f;

        velocity_ = 0.5f;
        if (blackboard.contains("search_aruco_velocity"))
            velocity_ = *blackboard.get<float>("search_aruco_velocity");

        if (blackboard.contains("aruco_persistence_frames"))
            min_detections_ = static_cast<int>(*blackboard.get<float>("aruco_persistence_frames"));

        detection_counter_ = 0;
        miss_counter_       = 0;
        last_detected_id_   = -1;

        center_z_ = blackboard.contains("z_max_search")
            ? *blackboard.get<float>("z_max_search")
            : static_cast<float>(drone_->getLocalPosition().z());

        const float diagonal = std::hypot(abs_x_, abs_y_);
        seno_    = abs_y_ / diagonal;
        cosseno_ = abs_x_ / diagonal;

        bool first_entry = !blackboard.contains("search_aruco_visited");
        blackboard.set<bool>("search_aruco_visited", true);

        if (first_entry) {
            initial_pos_ = drone_->getLocalPosition();
            initial_yaw_ = static_cast<float>(drone_->getOrientation()[2]);
            contador_ = 1;
            laps_     = 0;
            drone_->log("First entry — rectangle centred on current position");
        } else {
            // Re-entrada apos ARUCO_LOST: o centro/orientacao do retangulo
            // precisam ficar FIXOS (definem quais cantos ainda faltam
            // visitar), entao restaura da blackboard em vez de recentrar na
            // posicao atual do drone.
            initial_pos_.x() = blackboard.contains("search_aruco_center_x")
                ? *blackboard.get<float>("search_aruco_center_x") : initial_pos_.x();
            initial_pos_.y() = blackboard.contains("search_aruco_center_y")
                ? *blackboard.get<float>("search_aruco_center_y") : initial_pos_.y();
            initial_yaw_ = blackboard.contains("search_aruco_yaw")
                ? *blackboard.get<float>("search_aruco_yaw") : initial_yaw_;
            contador_ = blackboard.contains("search_aruco_contador")
                ? *blackboard.get<int>("search_aruco_contador") : 1;
            laps_ = blackboard.contains("search_aruco_laps")
                ? *blackboard.get<int>("search_aruco_laps") : 0;
            drone_->log("Re-entrada SEARCH_ARUCO: retomando lap="
                + std::to_string(laps_) + " canto=" + std::to_string(contador_));
        }
        initial_pos_.z() = center_z_;

        blackboard.set<float>("search_aruco_center_x", static_cast<float>(initial_pos_.x()));
        blackboard.set<float>("search_aruco_center_y", static_cast<float>(initial_pos_.y()));
        blackboard.set<float>("search_aruco_yaw", initial_yaw_);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("aruco_detected"))
            is_detected = *blackboard.get<bool>("aruco_detected");

        if (is_detected) {
            miss_counter_ = 0;
            int current_id = 0;
            if (blackboard.contains("aruco_id"))
                current_id = *blackboard.get<int>("aruco_id");

            if (current_id == last_detected_id_) {
                detection_counter_++;
            } else {
                detection_counter_ = 1;
                last_detected_id_  = current_id;
            }

            if (detection_counter_ >= min_detections_) {
                blackboard.set<int>("confirmed_aruco_id", last_detected_id_);
                move_local_constant_step(drone_, drone_->getLocalPosition(), 0.0f);
                drone_->log("ArUco ID " + std::to_string(last_detected_id_) +
                            " confirmed after " + std::to_string(detection_counter_) +
                            " frames. Transitioning.");
                return "ARUCO_FOUND";
            }
        } else {
            miss_counter_++;
            if (miss_counter_ >= max_misses_) {
                detection_counter_ = 0;
                last_detected_id_  = -1;
            }
        }

        // ── Varredura retangular (RetangularSearchState, fase1_itjbx) ──────
        const float diagonal = std::hypot(abs_x_, abs_y_);
        if ((laps_ + 1) * step_size_ > diagonal - 1.0f) {
            // Area esgotada sem achar o ArUco: reinicia do lap 0 em vez de
            // desistir -- a FSM nao tem outcome de "busca esgotada".
            laps_     = 0;
            contador_ = 1;
        }

        if (contador_ % 5 == 1 || contador_ % 5 == 0) {
            goal_ = Eigen::Vector3d(initial_pos_.x() - (laps_ + 1) * step_size_ * cosseno_,
                                     initial_pos_.y() + (laps_ + 1) * step_size_ * seno_, center_z_);
        } else if (contador_ % 5 == 2) {
            goal_ = Eigen::Vector3d(initial_pos_.x() - (laps_ + 1) * step_size_ * cosseno_,
                                     initial_pos_.y() - (laps_ + 1) * step_size_ * seno_, center_z_);
        } else if (contador_ % 5 == 3) {
            goal_ = Eigen::Vector3d(initial_pos_.x() + (laps_ + 1) * step_size_ * cosseno_,
                                     initial_pos_.y() - (laps_ + 1) * step_size_ * seno_, center_z_);
        } else if (contador_ % 5 == 4) {
            goal_ = Eigen::Vector3d(initial_pos_.x() + (laps_ + 1) * step_size_ * cosseno_,
                                     initial_pos_.y() + (laps_ + 1) * step_size_ * seno_, center_z_);
        }

        Eigen::Vector3d cur  = drone_->getLocalPosition();
        Eigen::Vector3d diff = goal_ - cur;

        if (diff.norm() < 0.15f) {
            contador_++;
            if (contador_ % 5 == 0) {
                laps_++;
                contador_ = 0;
            }
        }

        blackboard.set<int>("search_aruco_contador", contador_);
        blackboard.set<int>("search_aruco_laps",     laps_);

        const double dist_to_goal = diff.norm();
        float velocidade_alvo = velocity_;
        if (corner_decel_radius_ > 0.0f && dist_to_goal < corner_decel_radius_) {
            const float fracao = static_cast<float>(dist_to_goal) / corner_decel_radius_;
            velocidade_alvo = corner_min_velocity_ +
                (velocity_ - corner_min_velocity_) * fracao;
        }

        Eigen::Vector3d little_goal = cur + (dist_to_goal > velocidade_alvo
                                            ? diff.normalized() * velocidade_alvo
                                            : diff);
        drone_->setLocalPosition(
            static_cast<float>(little_goal.x()),
            static_cast<float>(little_goal.y()),
            static_cast<float>(little_goal.z()),
            initial_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d initial_pos_{0.0, 0.0, 0.0};
    Eigen::Vector3d goal_{0.0, 0.0, 0.0};
    float initial_yaw_{0.0f};

    float abs_x_{6.0f}, abs_y_{6.0f};
    float step_size_{1.0f};
    float velocity_{0.5f};
    float center_z_{0.0f};
    float corner_decel_radius_{1.0f}, corner_min_velocity_{0.3f};
    float seno_{0.0f}, cosseno_{0.0f};
    int   contador_, laps_;

    int detection_counter_;
    int miss_counter_;
    int last_detected_id_;
    int min_detections_;
    int max_misses_;
};
