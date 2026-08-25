#pragma once


#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "stdstates/blackboard_params.hpp"

class RetangularSearchState : public fsm::State {
public:
    RetangularSearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        ok_ = false;

        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: RETANGULAR SEARCH");

        if (!stdstates::require(blackboard, drone_, "abs_x", vx_)) return;
        if (!stdstates::require(blackboard, drone_, "abs_y", vy_)) return;
        if (!stdstates::require(blackboard, drone_, "diagonal_step", step_)) return;
        if (!stdstates::require(blackboard, drone_, "position_tolerance", position_tolerance_)) return;
        if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", max_horizontal_velocity_)) return;

        // Blackboard so' tem float; converte pra int uma vez aqui.
        float total_bases_f = 0.0f;
        if (!stdstates::require(blackboard, drone_, "total_bases", total_bases_f)) return;
        total_bases_ = static_cast<int>(total_bases_f);

        // Ponteiro pego uma vez: "bases" so' e' mutada por cima do mesmo
        // objeto depois disso, nunca reescrita (ver construtor de Fase1ItjbxFSM).
        bases_ = blackboard.get<std::vector<Eigen::Vector3d>>("bases");
        if (bases_ == nullptr) {
            drone_->log("ERRO: parametro ausente na blackboard: 'bases'");
            return;
        }

        initial_pos_ = drone_->getLocalPosition();
        initial_yaw_ = drone_->getOrientation()[2];

        const float diagonal = std::sqrt(vx_ * vx_ + vy_ * vy_);
        seno_ = vy_ / diagonal;
        cosseno_ = vx_ / diagonal;
        contador_ = 1;
        laps_ = 0;

        ok_ = true;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        // bases_ foi guardado em on_enter(); nao precisa tocar a blackboard
        // de novo aqui.
        (void)blackboard;
        if (!ok_) return "ERROR";

        //if (bases_->size() >= static_cast<size_t>(total_bases_)) return "SEARCH COMPLETED";

        const float diagonal = std::sqrt(vx_ * vx_ + vy_ * vy_);

        if ((laps_ + 1) * step_ > diagonal - 1.0f) {
            return "SEARCH COMPLETED";
        } else if (contador_ % 5 == 1 || contador_ % 5 == 0) {
            goal_ = {-(laps_ + 1) * step_ * cosseno_, (laps_ + 1) * step_ * seno_, initial_pos_.z()};
        } else if (contador_ % 5 == 2) {
            goal_ = {-(laps_ + 1) * step_ * cosseno_, -(laps_ + 1) * step_ * seno_, initial_pos_.z()};
        } else if (contador_ % 5 == 3) {
            goal_ = {(laps_ + 1) * step_ * cosseno_, -(laps_ + 1) * step_ * seno_, initial_pos_.z()};
        } else if (contador_ % 5 == 4) {
            goal_ = {(laps_ + 1) * step_ * cosseno_, (laps_ + 1) * step_ * seno_, initial_pos_.z()};
        }

        pos_ = drone_->getLocalPosition();

        Eigen::Vector3d diff = goal_ - pos_;

        if (diff.norm() < position_tolerance_) {
            contador_++;
            if (contador_ % 5 == 0) {
            laps_++;
            contador_ = 0;
        }

        }

        // Move toward goal with velocity clamping
        Eigen::Vector3d little_goal = pos_ + (diff.norm() > max_horizontal_velocity_
                                            ? diff.normalized() * max_horizontal_velocity_
                                            : diff);

        drone_->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            initial_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_{nullptr};
    bool ok_{false};
    float vx_{0.0f}, vy_{0.0f}, step_{0.0f};
    float position_tolerance_{0.0f}, max_horizontal_velocity_{0.0f};
    float seno_{0.0f}, cosseno_{0.0f};
    int contador_{0}, laps_{0};
    Eigen::Vector3d initial_pos_{0.0, 0.0, 0.0}, pos_{0.0, 0.0, 0.0}, goal_{0.0, 0.0, 0.0};
    float initial_yaw_{0.0f};
    std::vector<Eigen::Vector3d> *bases_{nullptr};
    int total_bases_{0};
};
