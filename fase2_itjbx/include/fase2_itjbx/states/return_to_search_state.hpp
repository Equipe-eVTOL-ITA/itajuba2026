#pragma once

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Volta pro EXATO ponto de onde o drone saiu da varredura (SEARCH_BASE)
 * pra ir alinhar com a ultima base -- NAO pro centro do retangulo de busca.
 * Isso permite que, ao reentrar em SEARCH_BASE, a varredura CONTINUE de
 * onde parou (ver o ajuste em SearchBaseState::on_enter que so' recentraliza
 * na primeira entrada) em vez de reiniciar do zero.
 *
 * Se "search_departure_*" nunca foi setado (base encontrada direto do lado
 * do ArUco, sem passar por SEARCH_BASE -- ver GoToArucoState, transicao
 * KNOWN_BASE), nao ha' pra onde voltar: sinaliza AT_START na hora, e
 * SEARCH_BASE trata a proxima entrada como primeira entrada normalmente.
 */
class ReturnToSearchState : public fsm::State {
public:
    ReturnToSearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: RETURN_TO_SEARCH");

        tem_ponto_partida_ = blackboard.contains("search_departure_x");
        if (tem_ponto_partida_) {
            destino_ = Eigen::Vector3d(
                *blackboard.get<float>("search_departure_x"),
                *blackboard.get<float>("search_departure_y"),
                *blackboard.get<float>("search_departure_z"));
            destino_yaw_ = blackboard.contains("search_departure_yaw")
                ? *blackboard.get<float>("search_departure_yaw") : 0.0f;
            drone_->log("Voltando pro ponto de saida da varredura (" +
                std::to_string(destino_.x()) + ", " + std::to_string(destino_.y()) + ")");
        } else {
            drone_->log("Sem ponto de saida salvo (base achada direto perto do "
                "ArUco) -- SEARCH_BASE comeca do zero.");
        }

        velocity_ = blackboard.contains("search_aruco_velocity")
            ? *blackboard.get<float>("search_aruco_velocity") : 0.4f;
    }

    std::string act(fsm::Blackboard & /*blackboard*/) override {
        if (drone_ == nullptr) return "ERROR";

        if (!tem_ponto_partida_) return "AT_START";

        bool chegou = move_local_constant_step(drone_, destino_, velocity_, 0.15f, destino_yaw_);
        if (chegou) {
            drone_->log("De volta ao ponto de saida da varredura.");
            return "AT_START";
        }
        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    bool tem_ponto_partida_{false};
    Eigen::Vector3d destino_{0.0, 0.0, 0.0};
    float destino_yaw_{0.0f};
    float velocity_{0.4f};
};
