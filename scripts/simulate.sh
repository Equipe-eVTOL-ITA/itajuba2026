#!/usr/bin/env bash
# =============================================================================
# Template: simulate.sh — sobe o PX4 SITL + Gazebo para um mundo.
# =============================================================================
#
#   ./scripts/simulate.sh <mundo>
#
# Copie para o scripts/ da sua competição e preencha o bloco `case` com os
# mundos, modelos e poses iniciais dela. É o ÚNICO arquivo dos três templates
# que exige customização de verdade.
# =============================================================================
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# <ws>/src/<competicao>/scripts/  ->  <ws>
ws_root="$(cd "$script_dir/../../.." && pwd)"

# Carrega a distro do perfil desta máquina + o install/ do workspace.
# Nunca escreva `source /opt/ros/humble/setup.bash` aqui: o time voa com
# Jetson (Humble) e Raspberry Pi (Jazzy), e o mesmo script tem que servir aos
# dois. Veja docs/ARCHITECTURE.md, "O Contrato de Ambiente".
source "$ws_root/scripts/ros_env.sh"

# MODELOS: resolvidos pelo Gazebo via GZ_SIM_RESOURCE_PATH, e nao precisam
# estar dentro da arvore do PX4. O proprio PX4 acrescenta a arvore dele ao
# final desta variavel (veja gz_env.sh), entao o que exportamos aqui vem
# ANTES e tem prioridade.
#
# A ordem importa e ja causou bug: quando a arvore do PX4 vinha primeiro, uma
# copia antiga de modelo escondia a deste repositorio, e a versao que rodava
# nao era a versionada. Com o repo da equipe na frente, ele e a fonte da
# verdade e pode ate sobrescrever um modelo do proprio PX4.
#
# (As variaveis GAZEBO_* sao do Gazebo classico e nao funcionam aqui.)
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-gazebo-models/models:${GZ_SIM_RESOURCE_PATH:-}"

cd "$HOME/PX4-Autopilot"

PX4_SYS_AUTOSTART=4001

case "${1:-}" in
    # ---- CUSTOMIZE: os mundos da sua competição -------------------------
    # fase1)
    #     PX4_GZ_WORLD=fase1_27                       # nome do .sdf
    #     PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"   # x,y,z,r,p,y
    #     PX4_SIM_MODEL=x500_sae                      # modelo do drone
    #     ;;
    # Mesmo mundo da SAE2026 missao 1 (sae1_26), so para testar a trajetoria
    # do fase1_itjbx sem depender de um mundo proprio do itajuba_2026 ainda.
    sae1)
        PX4_GZ_WORLD=sae1_26
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    # Mundo do fase4_itjbx: arena regulamentar da Missao 4 (Black Bee Challenge
    # 2026, 14x14m) -- linha azul da base de decolagem redonda ate o vao entre
    # os dois postes onde fica a mangueira vermelha. O modelo x500_itajuba ja
    # vem com as cameras vertical/horizontal usadas pelos detectores.
    fase4)
        PX4_GZ_WORLD=missao4_itjbx
        # Base afastada da origem (-2.5 em X) pra alongar a linha ate o vao dos
        # postes -- mantenha esta pose IGUAL a <pose> do include
        # sae_26/plataforma_decolagem e ao --start-x/--start-y abaixo, os tres
        # tem que apontar pro mesmo lugar (ver missao4_itjbx.sdf).
        PX4_GZ_MODEL_POSE="-2.5, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_itajuba

        # Regera a linha aleatoriamente a CADA carregamento do mundo -- sem isso
        # o Gazebo so enxerga o ultimo model.sdf que ficou salvo em
        # ~/PX4-gazebo-models/models/generated_path (podia ser uma reta de teste
        # antiga, esquecida ali para sempre). Sem --seed de proposito: e' o que
        # garante uma curva diferente a cada `simulate.sh fase4`.
        #
        # --start-x/--start-y: mesma pose da base de decolagem (acima e no
        # mundo). --target-x/--target-y: o centro da mangueira_m4
        # (missao4_itjbx.sdf), exatamente no meio dos dois postes -- e' o que
        # faz a linha terminar SEMPRE ali, curva diferente a cada vez mas o
        # destino nunca muda.
        #
        # --checkpoint-fractions: estacas amarelas em 1/3 e 2/3 do comprimento
        # de arco da curva gerada -- viram links do proprio generated_path,
        # entao acompanham a linha nova a cada regeneracao (ver missao4_itjbx.sdf).
        #
        # --waypoints/--jitter: 5 ancoras com ate 2.2m de desvio lateral (era
        # 3/1.0, o padrao do script) -- com o padrao a linha saia visualmente
        # quase reta (~0.5m de desvio maximo em 8.4m de percurso). Com estes
        # valores o desvio tipico fica em ~1-1.5m, curva de verdade pra testar
        # SEARCH_LINE/FOLLOW_LINE, e o afunilamento nas pontas (ver
        # generate_random_path.py) continua suave logo apos a decolagem.
        python3 "$ws_root/src/itajuba_2026/scripts/generate_random_path.py" \
            --output "$HOME/PX4-gazebo-models/models/generated_path/model.sdf" \
            --width 0.1 \
            --start-x -2.5 --start-y 0.0 \
            --target-x 5.673669 --target-y -0.099193 \
            --waypoints 5 --jitter 2.2 \
            --checkpoint-fractions 0.3333,0.6667
        echo "Linha aleatoria gerada em $HOME/PX4-gazebo-models/models/generated_path/model.sdf"
        ;;
    default)
        PX4_GZ_WORLD=default
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500
        ;;
    *)
        echo "Mundo desconhecido: '${1:-}'" >&2
        echo "Uso: $0 <mundo>" >&2
        echo "Disponíveis: default, sae1, fase4   (edite este script para adicionar os seus)" >&2
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Confere que o mundo e o modelo existem ANTES de chamar o PX4.
#
# Sem isto, o PX4 falha com uma mensagem que aponta para o lugar errado:
#
#     Unable to find or download file
#     ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH
#     ERROR [init] gz_bridge failed to start and spawn model
#
# ...o que faz todo mundo ir mexer no GZ_SIM_RESOURCE_PATH, quando a causa
# quase sempre e outra: o .sdf existe em ~/PX4-gazebo-models mas nao foi
# symlinkado para dentro do PX4. Os symlinks sao feitos uma vez; arquivo novo
# adicionado depois nao ganha symlink sozinho.
# ---------------------------------------------------------------------------
px4_gz="$HOME/PX4-Autopilot/Tools/simulation/gz"
faltando=0

# MUNDOS: o PX4 monta um caminho ABSOLUTO na arvore dele e passa ao gz sim
# (px4-rc.simulator: `gz sim -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf"`).
# Diferente dos modelos, aqui o arquivo precisa mesmo aparecer la dentro.
# Criamos o link do mundo que vamos lancar, e so dele -- assim ninguem
# precisa lembrar de refazer symlink quando um mundo novo entra no repo.
if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" \
   && -e "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "Criando link para o mundo '$PX4_GZ_WORLD' na arvore do PX4"
    ln -sf "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" "$px4_gz/worlds/"
fi

if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "ERRO: mundo '$PX4_GZ_WORLD.sdf' nao encontrado em" >&2
    echo "      $px4_gz/worlds/" >&2
    echo "      -> nao existe nem em ~/PX4-gazebo-models. Crie o .sdf la." >&2
    faltando=1
fi

# MODELOS: basta existir no repositorio da equipe ou na arvore do PX4 --
# os dois estao no GZ_SIM_RESOURCE_PATH. Nao ha symlink de modelo.
if [[ ! -e "$HOME/PX4-gazebo-models/models/$PX4_SIM_MODEL" \
   && ! -e "$px4_gz/models/$PX4_SIM_MODEL" ]]; then
    echo "ERRO: modelo '$PX4_SIM_MODEL' nao encontrado." >&2
    echo "      Procurado em ~/PX4-gazebo-models/models/ e em" >&2
    echo "      $px4_gz/models/" >&2
    echo "      -> crie o modelo em ~/PX4-gazebo-models/models/." >&2
    faltando=1
fi

if (( faltando )); then
    echo >&2
    echo "Veja docs/gazebo_models_setup.md." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Mata qualquer "gz sim" orfao do MESMO mundo antes de subir o PX4.
#
# O px4-rc.simulator faz pgrep pelo nome do mundo: se acha um "gz sim" ja
# rodando, ele NAO sobe um servidor novo -- so tenta anexar o gz_bridge nesse
# processo existente ("gazebo already running world: <mundo>" no log). Se a
# run anterior morreu sem levar o gz sim filho junto (Ctrl+C no lugar errado,
# terminal fechado, crash), esse servidor orfao fica de pe e o bridge da run
# nova trava em "timed out waiting for clock message" -- e o erro se repete
# em toda tentativa seguinte ate alguem matar o processo na mao.
# ---------------------------------------------------------------------------
orfaos="$(pgrep -f "gz sim.*worlds/${PX4_GZ_WORLD}\.sdf" || true)"
if [[ -n "$orfaos" ]]; then
    echo "Encerrando gz sim orfao do mundo '$PX4_GZ_WORLD' (pid: $orfaos)"
    kill $orfaos 2>/dev/null || true
    for _ in $(seq 1 10); do
        pgrep -f "gz sim.*worlds/${PX4_GZ_WORLD}\.sdf" >/dev/null || break
        sleep 0.5
    done
    pkill -9 -f "gz sim.*worlds/${PX4_GZ_WORLD}\.sdf" 2>/dev/null || true
fi

PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_GZ_MODEL_POSE=$PX4_GZ_MODEL_POSE \
PX4_GZ_WORLD=$PX4_GZ_WORLD \
PX4_SIM_MODEL=$PX4_SIM_MODEL \
./build/px4_sitl_default/bin/px4
