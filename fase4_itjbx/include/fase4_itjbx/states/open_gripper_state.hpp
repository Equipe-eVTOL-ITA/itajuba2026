#pragma once

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

// Entra logo depois do AlignRedLineState ("ALIGNED"): abre a garra via script
// Python externo (gripper_script_path, ver scripts/abrirgarra.py), rodado
// com std::system() -- mesmo padrao do DropTheHookState (sae2026/mission_2).
//
// std::system() BLOQUEIA a thread da FSM ate' o script terminar (nenhum
// setpoint novo e' publicado nesse meio tempo, ~1s). Se o offboard do PX4
// comecar a dar timeout nessa chamada, e' o primeiro lugar pra olhar.
class OpenGripperState : public fsm::State {
public:
    OpenGripperState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        drone_ = *bb.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: OPEN GRIPPER");

        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        script_path_ = bb.contains("gripper_script_path")
            ? *bb.get<std::string>("gripper_script_path") : std::string("~/evtol/dev/scripts/abrirgarra.py");
        // Parado por esse tempo DEPOIS do script, antes de liberar a transicao.
        settle_s_ = bb.contains("gripper_settle_s")
            ? *bb.get<float>("gripper_settle_s") : 1.0f;

        executed_ = false;
    }

    std::string act(fsm::Blackboard& bb) override {
        (void)bb;
        if (drone_ == nullptr) return "ERROR";

        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);

        if (!executed_) {
            std::string command = "python3 " + script_path_;
            drone_->log("OPEN GRIPPER: executando " + command);

            int result = std::system(command.c_str());
            if (result == 0) {
                drone_->log("OPEN GRIPPER: garra aberta.");
            } else {
                drone_->log("OPEN GRIPPER: aviso -- script retornou codigo != 0.");
            }

            executed_ = true;
            settle_start_ = std::chrono::steady_clock::now();
            return "";
        }

        float settled_for = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - settle_start_).count();
        if (settled_for >= settle_s_) {
            return "GRIPPER OPENED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    std::string script_path_;
    float settle_s_ = 1.0f;
    bool executed_ = false;
    std::chrono::steady_clock::time_point settle_start_;
};
