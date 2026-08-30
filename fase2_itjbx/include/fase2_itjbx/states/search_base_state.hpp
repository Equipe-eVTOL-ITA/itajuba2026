#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Procura a base alvo (ja identificada em GO_TO_ARUCO/DESCEND_FOR_SHAPE)
 * com a mesma varredura retangular de SearchArucoState. Portado de
 * sae2026/mission_1::SearchBaseState, preservando a logica de centrar a
 * busca numa posicao em cache (known_base_<alvo>_x/y) quando disponivel.
 *
 * Blackboard reads:
 *   "base_search_abs_x"/"base_search_abs_y" (float) — meia-extensao do
 *                                retangulo final (default 3.0 m cada)
 *   "base_search_step"     (float) — crescimento por volta (default 0.5 m,
 *                           mais apertado que a busca de ArUco porque roda
 *                           perto de uma posicao ja conhecida, nao do zero)
 *   "corner_decel_radius"/"corner_min_velocity" — rampa de freio nos cantos
 *   "base_persistence_frames" (float) — frames pra confirmar a base (default 3)
 *
 * Retorna: "BASE_FOUND"
 */
class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State(),
        center_z_(0.0f), contador_(1), laps_(0),
        base_detection_counter_(0), base_miss_counter_(0),
        min_base_detections_(3), going_to_center_(false) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_BASE (Rectangular Search)");

        // Altitude fixa: capturada na primeira entrada, reusada nas
        // reentradas pra evitar o "catraqueamento" causado por deriva do PX4.
        if (!blackboard.contains("search_base_altitude"))
            blackboard.set<float>("search_base_altitude",
                static_cast<float>(drone_->getLocalPosition().z()));
        center_z_ = *blackboard.get<float>("search_base_altitude");

        abs_x_ = blackboard.contains("base_search_abs_x")
            ? *blackboard.get<float>("base_search_abs_x") : 3.0f;
        abs_y_ = blackboard.contains("base_search_abs_y")
            ? *blackboard.get<float>("base_search_abs_y") : 3.0f;
        step_size_ = blackboard.contains("base_search_step")
            ? *blackboard.get<float>("base_search_step") : 0.5f;
        corner_decel_radius_ = blackboard.contains("corner_decel_radius")
            ? *blackboard.get<float>("corner_decel_radius") : 1.0f;
        corner_min_velocity_ = blackboard.contains("corner_min_velocity")
            ? *blackboard.get<float>("corner_min_velocity") : 0.3f;
        velocity_ = 0.4f;
        if (blackboard.contains("search_aruco_velocity"))
            velocity_ = *blackboard.get<float>("search_aruco_velocity");

        min_base_detections_ = blackboard.contains("base_persistence_frames")
            ? static_cast<int>(*blackboard.get<float>("base_persistence_frames")) : 3;
        base_detection_counter_ = 0;
        base_miss_counter_       = 0;

        bool first_entry = !blackboard.contains("search_base_visited");
        blackboard.set<bool>("search_base_visited", true);

        if (first_entry) {
            // Primeira entrada (logo depois de GO_TO_ARUCO calcular o
            // alvo): SEMPRE centraliza na ORIGEM (0,0) -- pedido explicito,
            // pra' varredura de base cobrir o campo a partir do ponto de
            // partida/decolagem, nao de onde o drone parou de alinhar com o
            // ArUco (que pode estar em qualquer canto do campo). NAO usa
            // "known_base_<alvo>_" aqui: com bases do mesmo formato
            // compartilhando rotulo (numero so' confirmado sob demanda,
            // ver ConfirmNumberState), essa posicao seria so' a de QUALQUER
            // candidata de mesma forma ja avistada de relance, nao
            // necessariamente a certa.
            initial_pos_ = Eigen::Vector3d(0.0, 0.0, center_z_);
            drone_->log("First entry — rectangle centred on origin (0,0)");
            initial_yaw_ = static_cast<float>(drone_->getOrientation()[2]);
            contador_ = 1;
            laps_     = 0;

            // Pedido explicito: entre o ArUco e a origem o drone NAO pode
            // reagir a bases vistas de relance (nem GO_TO_ARUCO pulando reto
            // pra GO_TO_BASE via KNOWN_BASE, nem esta varredura confirmando
            // uma deteccao antes de sequer ter comecado a cobrir a area).
            // going_to_center_ mantem o inicio de act() cego a
            // target_base_in_sight ate' o drone chegar fisicamente na
            // origem -- so' entao a varredura de verdade comeca a valer
            // deteccoes.
            going_to_center_ = true;
        } else {
            // Reentrada: SEMPRE retoma de onde a varredura parou -- NUNCA
            // recentraliza na posicao "known_base_<alvo>_" aqui. Com bases
            // do mesmo formato compartilhando rotulo (o numero so' e'
            // confirmado sob demanda agora, em CONFIRM_NUMBER -- ver
            // RDPformas.py/ConfirmNumberState), essa posicao pode ser
            // justamente a base que acabou de ser descartada, o que faria a
            // varredura reiniciar em cima dela em vez de continuar cobrindo
            // o resto da area (pedido explicito: "continue a espiral que ja
            // estava realizando"). Quem devolve o drone fisicamente pro
            // lugar certo antes de reentrar aqui e' ReturnToSearchState.
            initial_pos_.x() = blackboard.contains("search_base_center_x")
                ? *blackboard.get<float>("search_base_center_x") : drone_->getLocalPosition().x();
            initial_pos_.y() = blackboard.contains("search_base_center_y")
                ? *blackboard.get<float>("search_base_center_y") : drone_->getLocalPosition().y();
            contador_ = blackboard.contains("search_base_contador")
                ? *blackboard.get<int>("search_base_contador") : 1;
            laps_ = blackboard.contains("search_base_laps")
                ? *blackboard.get<int>("search_base_laps") : 0;
            initial_yaw_ = blackboard.contains("search_base_yaw")
                ? *blackboard.get<float>("search_base_yaw") : initial_yaw_;
            // Reentrada: ReturnToSearchState ja devolveu o drone pro ponto
            // exato onde a varredura parou -- nao ha' trecho "as cegas" pra
            // repetir aqui.
            going_to_center_ = false;
        }
        initial_pos_.z() = center_z_;

        const float diagonal = std::hypot(abs_x_, abs_y_);
        seno_    = abs_y_ / diagonal;
        cosseno_ = abs_x_ / diagonal;

        blackboard.set<float>("search_base_center_x", static_cast<float>(initial_pos_.x()));
        blackboard.set<float>("search_base_center_y", static_cast<float>(initial_pos_.y()));
        blackboard.set<float>("search_base_yaw", initial_yaw_);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        if (going_to_center_) {
            // Voo cego a deteccao: so navegacao ate' a origem, sem checar
            // target_base_in_sight. Raio de chegada mais largo que o das
            // pernas da varredura (0.15) porque aqui so' importa estar perto
            // o bastante da origem pra' varredura de verdade comecar -- nao
            // precisa do mesmo grau de precisao.
            Eigen::Vector3d cur = drone_->getLocalPosition();
            Eigen::Vector3d diff = initial_pos_ - cur;
            if (diff.norm() < 0.30) {
                going_to_center_ = false;
            } else {
                Eigen::Vector3d little_goal = cur + (diff.norm() > velocity_
                                                    ? diff.normalized() * velocity_
                                                    : diff);
                drone_->setLocalPosition(
                    static_cast<float>(little_goal.x()),
                    static_cast<float>(little_goal.y()),
                    static_cast<float>(little_goal.z()),
                    initial_yaw_);
                return "";
            }
        }

        bool is_detected = blackboard.contains("target_base_in_sight") &&
                           *blackboard.get<bool>("target_base_in_sight");

        if (is_detected) {
            base_miss_counter_ = 0;
            base_detection_counter_++;
            if (base_detection_counter_ >= min_base_detections_) {
                move_local_constant_step(drone_, drone_->getLocalPosition(), 0.0f);

                // Guarda o ponto EXATO de onde a varredura foi interrompida
                // -- e' pra ca' que ReturnToSearchState volta se
                // CONFIRM_NUMBER rejeitar essa base, pra' varredura
                // CONTINUAR daqui em vez de recomecar.
                auto pos = drone_->getLocalPosition();
                blackboard.set<float>("search_departure_x", static_cast<float>(pos.x()));
                blackboard.set<float>("search_departure_y", static_cast<float>(pos.y()));
                blackboard.set<float>("search_departure_z", static_cast<float>(pos.z()));
                blackboard.set<float>("search_departure_yaw",
                                       static_cast<float>(drone_->getOrientation()[2]));

                drone_->log("Target Base confirmed after " +
                    std::to_string(base_detection_counter_) + " frames!");
                return "BASE_FOUND";
            }
        } else {
            base_miss_counter_++;
            if (base_miss_counter_ >= base_miss_tolerance_)
                base_detection_counter_ = 0;
        }

        // ── Varredura retangular (RetangularSearchState, fase1_itjbx) ──────
        const float diagonal = std::hypot(abs_x_, abs_y_);
        if ((laps_ + 1) * step_size_ > diagonal - 1.0f) {
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

        blackboard.set<int>("search_base_contador", contador_);
        blackboard.set<int>("search_base_laps",     laps_);

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

    float abs_x_{3.0f}, abs_y_{3.0f};
    float step_size_{0.5f};
    float velocity_{0.4f};
    float center_z_;
    float corner_decel_radius_{1.0f}, corner_min_velocity_{0.3f};
    float seno_{0.0f}, cosseno_{0.0f};
    int   contador_, laps_;

    int   base_detection_counter_;
    int   base_miss_counter_;
    int   min_base_detections_;
    static constexpr int base_miss_tolerance_ = 3;
    bool  going_to_center_;
};
