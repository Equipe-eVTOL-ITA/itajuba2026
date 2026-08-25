#ifndef FASE2_ITJBX__STATES__EXAMPLE_STATE_HPP_
#define FASE2_ITJBX__STATES__EXAMPLE_STATE_HPP_

#include <string>
#include <memory>

#include "fsm/state.hpp"
#include "drone/Drone.hpp"

/**
 * @brief Modelo de estado desta missao. Copie este arquivo para criar os seus.
 *
 * Um estado tem tres partes:
 *   on_enter  roda UMA vez ao entrar. Leia parametros e guarde o alvo aqui.
 *   act       roda a cada tick (20 Hz). Devolva "" para continuar no estado,
 *             ou o nome de um outcome para transitar.
 *   on_exit   roda UMA vez ao sair (opcional).
 *
 * Todo dado compartilhado passa pela blackboard -- nunca por variavel global
 * nem por ponteiro entre estados.
 */
class ExampleState : public fsm::State {
public:
    ExampleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: EXAMPLE");

        // Parametros vem do YAML, via blackboard. NUNCA escreva o numero aqui.
        // max_velocity_ = *blackboard.get<float>("max_horizontal_velocity");

        start_ = drone_->getLocalPosition();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ == nullptr) return "ERROR";

        // Lógica do estado, executada a 20 Hz.
        //
        // Leitura de um dado publicado por um no de visao:
        //   bool *detectado = blackboard.get<bool>("alvo_detectado");
        //   if (detectado && *detectado) return "ALVO_ENCONTRADO";

        return "CONCLUIDO";   // "" mantém no estado; nome de outcome transita
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    std::shared_ptr<Drone> drone_{nullptr};
    Eigen::Vector3d start_{0.0, 0.0, 0.0};
};

#endif  // FASE2_ITJBX__STATES__EXAMPLE_STATE_HPP_
