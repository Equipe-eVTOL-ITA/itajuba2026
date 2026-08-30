#pragma once

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Confirma o numero da base alinhada com UMA leitura de OCR sob demanda --
 * nao roda OCR a cada frame (ver RDPformas.py, que agora so' faz deteccao
 * de forma continuamente e leitura de digito so' quando pedida). Dispara o
 * pedido em on_enter() (seta "ocr_request_pending" na blackboard; quem
 * publica de fato e' Fase2ItjbxNode::executeFSM() -- estados nao tem acesso
 * direto a publishers ROS, so' a Drone/blackboard) e fica parado (hover)
 * esperando a resposta chegar em "ocr_result_ready"/"ocr_result_success"/
 * "ocr_result_digit" (escritos pelo callback do topico de resultado, ver
 * Fase2ItjbxNode::ocr_result_callback).
 *
 * Se o digito bate com o numero de target_base: NUMBER_CONFIRMED.
 * Se nao bate (ou estoura o timeout sem resposta): registra a posicao ATUAL
 * do drone como uma base "ignorada" (pra SEARCH_BASE/cv_callback nao
 * voltarem a tratar essa base como candidata) e retorna NUMBER_WRONG.
 */
class ConfirmNumberState : public fsm::State {
public:
    ConfirmNumberState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: CONFIRM_NUMBER");

        hold_pos_ = drone_->getLocalPosition();
        hold_yaw_ = static_cast<float>(drone_->getOrientation()[2]);

        timeout_ticks_ = blackboard.contains("ocr_timeout_ticks")
            ? static_cast<int>(*blackboard.get<float>("ocr_timeout_ticks")) : 200;  // 10s @ 20Hz

        // Limpa qualquer resposta antiga (de um pedido anterior) e sinaliza
        // o pedido novo -- quem publica de verdade e' o No, que fica de
        // olho nesse flag em executeFSM().
        blackboard.set<bool>("ocr_result_ready", false);
        blackboard.set<bool>("ocr_request_pending", true);

        tick_ = 0;
        drone_->log("Pedido de leitura de numero disparado -- segurando posicao.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        // Segura a posicao onde parou -- pedido explicito: "enquanto o easy
        // ocr roda o drone fica parado".
        move_local_constant_step(drone_, hold_pos_, 0.0f, 0.10f, hold_yaw_);

        bool result_ready = blackboard.contains("ocr_result_ready") &&
                             *blackboard.get<bool>("ocr_result_ready");

        if (!result_ready) {
            if (tick_++ >= timeout_ticks_) {
                drone_->log("Timeout esperando leitura de numero -- tratando como base errada.");
                registrar_base_ignorada(blackboard);
                return "NUMBER_WRONG";
            }
            return "";
        }

        bool success = blackboard.contains("ocr_result_success") &&
                       *blackboard.get<bool>("ocr_result_success");
        std::string digito = blackboard.contains("ocr_result_digit")
            ? *blackboard.get<std::string>("ocr_result_digit") : std::string();

        std::string target_base = blackboard.contains("target_base")
            ? *blackboard.get<std::string>("target_base") : std::string();
        std::string target_digito;
        auto sep = target_base.find('_');
        if (sep != std::string::npos) target_digito = target_base.substr(sep + 1);

        if (success && !digito.empty() && digito == target_digito) {
            drone_->log("Numero confirmado: " + digito + " == alvo (" + target_base + "). Pousando.");
            return "NUMBER_CONFIRMED";
        }

        drone_->log("Numero lido (" + (success ? digito : std::string("ilegivel")) +
                    ") != alvo (" + target_digito + "). Base descartada, retomando busca.");
        registrar_base_ignorada(blackboard);
        return "NUMBER_WRONG";
    }

private:
    /// Guarda a posicao atual do drone (== posicao da base, ja que esta
    /// alinhado sobre ela) numa lista crescente na blackboard --
    /// Fase2ItjbxNode::cv_callback usa essa lista pra suprimir
    /// target_base_in_sight quando a base "em vista" cai perto de alguma
    /// posicao ja rejeitada.
    void registrar_base_ignorada(fsm::Blackboard &blackboard) {
        int count = blackboard.contains("ignored_base_count")
            ? *blackboard.get<int>("ignored_base_count") : 0;
        auto pos = drone_->getLocalPosition();
        blackboard.set<float>("ignored_base_" + std::to_string(count) + "_x",
                               static_cast<float>(pos.x()));
        blackboard.set<float>("ignored_base_" + std::to_string(count) + "_y",
                               static_cast<float>(pos.y()));
        blackboard.set<int>("ignored_base_count", count + 1);
    }

    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d hold_pos_{0.0, 0.0, 0.0};
    float hold_yaw_{0.0f};
    int tick_{0};
    int timeout_ticks_{200};
};
