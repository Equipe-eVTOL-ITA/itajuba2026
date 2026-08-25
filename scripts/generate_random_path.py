#!/usr/bin/env python3
"""
Script para gerar caminhos aleatórios em formato SDF para simulação Gazebo
"""

import math
import random
import numpy as np
import argparse
import os
from pathlib import Path

def calculate_direction_change(current_direction, curve_intensity, direction_preference=None, segment_index=0):
    """Calcula a mudança de direção para o próximo segmento

    Args:
        current_direction: Direção atual em radianos
        curve_intensity: Intensidade das curvas
        direction_preference: Preferência direcional
        segment_index: Índice do segmento atual

    Returns:
        Nova direção em radianos
    """
    if direction_preference == 'straight':
        # Linha reta: não muda a direção
        direction_change = 0.0
    elif direction_preference == 'spiral_left':
        # Espiral no sentido anti-horário
        direction_change = curve_intensity * 0.5
    elif direction_preference == 'spiral_right':
        # Espiral no sentido horário
        direction_change = -curve_intensity * 0.5
    elif direction_preference == 'zigzag':
        # Padrão zigue-zague
        direction_change = curve_intensity * (1 if segment_index % 2 == 0 else -1)
    elif direction_preference == 'return':
        # Tenta voltar na direção oposta gradualmente
        target_direction = current_direction + math.pi
        direction_change = (target_direction - current_direction) * 0.3
    else:
        # Mudança aleatória (padrão)
        direction_change = random.uniform(-curve_intensity, curve_intensity)

    return current_direction + direction_change

def generate_bezier_curve(start, control1, control2, end, num_points=50):
    """Gera pontos ao longo de uma curva de Bézier cúbica"""
    points = []
    for t in np.linspace(0, 1, num_points):
        # Fórmula da curva de Bézier cúbica
        point = (1-t)**3 * start + 3*(1-t)**2*t * control1 + 3*(1-t)*t**2 * control2 + t**3 * end
        points.append(point)
    return points

def generate_smooth_path(start_pos=np.array([0, 0, 0]),
                        total_length=20,
                        num_segments=4,
                        curve_intensity=2.0,
                        initial_direction=None,
                        direction_preference=None):
    """Gera um caminho suave usando múltiplas curvas de Bézier

    Args:
        start_pos: Posição inicial (x, y, z)
        total_length: Comprimento total do caminho
        num_segments: Número de segmentos principais
        curve_intensity: Intensidade das curvas (0-3)
        initial_direction: Direção inicial em radianos (None = aleatória)
                          0 = Norte (+Y), π/2 = Leste (+X), π = Sul (-Y), 3π/2 = Oeste (-X)
        direction_preference: Preferência direcional ('north', 'south', 'east', 'west', 'random', None)
    """

    # Determinar direção inicial
    if initial_direction is not None:
        direction = initial_direction
    elif direction_preference == 'north':
        direction = 0  # +Y
    elif direction_preference == 'east':
        direction = math.pi / 2  # +X
    elif direction_preference == 'south':
        direction = math.pi  # -Y
    elif direction_preference == 'west':
        direction = 3 * math.pi / 2  # -X
    else:
        # Direção inicial aleatória
        direction = random.uniform(0, 2 * math.pi)

    # Dividir o comprimento total em segmentos
    segment_length = total_length / num_segments

    all_points = [start_pos]
    current_pos = start_pos.copy()
    current_direction = direction

    for i in range(num_segments):
        # Ponto final do segmento
        end_x = current_pos[0] + segment_length * math.cos(current_direction)
        end_y = current_pos[1] + segment_length * math.sin(current_direction)
        end_pos = np.array([end_x, end_y, 0.001])  # Ligeiramente acima do chão

        # Pontos de controle para a curva de Bézier
        control_distance = segment_length * 0.3

        # Primeiro ponto de controle (continua a direção atual)
        control1_x = current_pos[0] + control_distance * math.cos(current_direction)
        control1_y = current_pos[1] + control_distance * math.sin(current_direction)
        control1 = np.array([control1_x, control1_y, 0.001])

        # Calcular próxima direção usando a função controlada
        next_direction = calculate_direction_change(
            current_direction, curve_intensity, direction_preference, i
        )

        # Segundo ponto de controle (aponta para a nova direção)
        control2_x = end_pos[0] - control_distance * math.cos(next_direction)
        control2_y = end_pos[1] - control_distance * math.sin(next_direction)
        control2 = np.array([control2_x, control2_y, 0.001])

        # Gerar pontos da curva de Bézier
        curve_points = generate_bezier_curve(current_pos, control1, control2, end_pos, 15)
        all_points.extend(curve_points[1:])  # Pular o primeiro ponto para evitar duplicação

        current_pos = end_pos
        current_direction = next_direction

    return all_points

def generate_path_to_target(start_pos=np.array([0, 0, 0]),
                             target_pos=np.array([5, 0, 0]),
                             num_waypoints=3,
                             jitter=1.0,
                             exit_angle_deg=None,
                             exit_distance=None,
                             exit_angle_exclusion_deg=20.0,
                             exit_angle_max_deg=90.0,
                             points_per_segment=15):
    """Gera um caminho curvo e aleatorio que comeca EXATAMENTE em start_pos e
    termina EXATAMENTE em target_pos (ex.: o vao entre dois postes fixos no
    mundo -- ver missao4_itjbx.sdf).

    Diferente de generate_smooth_path (que so' controla direcao/comprimento e
    pode terminar em qualquer lugar), aqui o ponto final e' um requisito, nao
    um resultado do passeio aleatorio. A tecnica: gera pontos-ancora
    aleatorios ao longo da reta exit_point->target (com deslocamento lateral
    aleatorio, perpendicular a essa reta) e interpola uma curva Catmull-Rom
    por eles. Catmull-Rom passa exatamente por TODOS os pontos-ancora
    (diferente de uma Bezier aproximando pontos de controle), entao o
    primeiro e o ultimo trecho da curva tem que bater exatamente com
    start_pos e target_pos.

    exit_angle_deg / exit_distance: a linha sai da base numa perna reta, numa
    direcao propositalmente DIFERENTE da direcao direta ate o alvo -- e' o
    ponto de SearchLineState (a base e' redonda, a linha sai de um lado
    qualquer, e o drone tem que descobrir de qual). Sem isso, o caminho saia
    sempre mais ou menos apontado pro alvo desde o primeiro segmento, o que
    testava so a metade do SEARCH_LINE (o alinhamento fino via circle_detector,
    nunca o caso de uma saida bem fora do eixo).

    Args:
        start_pos: Posicao inicial (x, y, z)
        target_pos: Posicao final, fixa (x, y, z)
        num_waypoints: Quantas ancoras aleatorias entre a perna de saida e o
            alvo (mais ancoras = mais curvas)
        jitter: Deslocamento lateral maximo (m) de cada ancora em relacao a
            reta exit_point->target
        exit_angle_deg: Direcao da perna de saida, em graus (convencao
            atan2: 0 = mesmo sentido do eixo X do mundo, cresce anti-horario).
            None (padrao) sorteia um angulo aleatorio, EXCLUINDO um cone em
            torno da direcao direta ate o alvo (ver exit_angle_exclusion_deg)
            -- garante que a linha nunca sai "por acaso" quase apontada pro
            alvo.
        exit_distance: Comprimento (m) da perna de saida. None (padrao) usa
            18% da distancia reta start->target (minimo 1.0m).
        exit_angle_exclusion_deg: Meia-largura (graus) do cone, em torno da
            direcao direta ate o alvo, que o sorteio de exit_angle_deg evita.
            So' vale quando exit_angle_deg=None.
        exit_angle_max_deg: Maior offset (graus) que o sorteio de
            exit_angle_deg permite, pra qualquer lado da direcao direta.
            Limita o quanto a linha tem que curvar de volta pra alcancar o
            alvo -- sem isso um sorteio proximo de 180 (saindo quase de
            costas pro alvo) forcava uma curva bem mais fechada no meio do
            caminho pra recuperar. So' vale quando exit_angle_deg=None.
        points_per_segment: Resolucao de cada trecho da curva
    """
    start_pos = np.asarray(start_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)

    total_vec = target_pos - start_pos
    total_dist = float(np.linalg.norm(total_vec[:2]))
    if total_dist < 1e-6:
        return [start_pos, target_pos]

    direct_bearing_rad = math.atan2(total_vec[1], total_vec[0])

    if exit_angle_deg is None:
        # Sorteia a MAGNITUDE do offset entre exclusao e max (ex.: 20-90 graus)
        # e um lado (esquerda/direita) aleatorio -- ou seja, sempre claramente
        # fora do eixo do alvo, mas nunca perto de sair de costas pra ele.
        # Antes o offset sorteava direto ate 360-exclusao (quase 180 no pior
        # caso), o que forcava o resto do caminho a curvar bem mais fechado
        # pra recuperar rumo ao alvo -- isso que estava deixando as curvas
        # "acentuadas".
        lo = max(0.0, exit_angle_exclusion_deg)
        hi = max(lo, exit_angle_max_deg)
        magnitude_deg = random.uniform(lo, hi)
        sign = random.choice([-1.0, 1.0])
        offset_deg = sign * magnitude_deg
        exit_angle_rad = direct_bearing_rad + math.radians(offset_deg)
    else:
        exit_angle_rad = math.radians(exit_angle_deg)

    if exit_distance is None:
        exit_distance = max(1.0, 0.18 * total_dist)

    exit_point = start_pos + exit_distance * np.array(
        [math.cos(exit_angle_rad), math.sin(exit_angle_rad), 0.0])
    exit_point[2] = 0.001

    # Do ponto de saida em diante, o resto do caminho (ancoras aleatorias +
    # afunilamento) segue a mesma tecnica de antes, so' que relativa a reta
    # exit_point->target em vez de start->target.
    remaining_vec = target_pos - exit_point
    remaining_dist = float(np.linalg.norm(remaining_vec[:2]))
    if remaining_dist < 1e-6:
        return smooth_polyline([start_pos, exit_point, target_pos])

    direction = remaining_vec.copy()
    direction[:2] /= remaining_dist
    # perpendicular no plano XY (Z fica de fora do jitter -- o chao e' plano)
    perp = np.array([-direction[1], direction[0], 0.0])

    anchors = [start_pos, exit_point]
    for i in range(1, num_waypoints + 1):
        t = i / (num_waypoints + 1)
        base = exit_point + t * remaining_vec
        # Afunila o desvio lateral perto das pontas (sin(pi*t): 0 em t=0/1, 1 no
        # meio) -- sem isso a primeira ancora tem o MESMO jitter maximo que as do
        # meio, mas sem nenhuma ancora "antes" suavizando a tangente de chegada
        # (ver Catmull-Rom abaixo, p0=p1 no primeiro trecho). O resultado e' a
        # curva mais fechada logo apos a perna de saida -- justo onde o drone
        # ainda esta' de baixa velocidade saindo do alinhamento do
        # SearchLineState, e onde qualquer ruido de controle fica mais visivel.
        # Afunilar tambem no fim ajuda a chegar mais reto no vao dos postes (a
        # mangueira e' perpendicular a rota -- ver missao4_itjbx.sdf).
        # Potencia 4 (nao so' sin(pi*t)) porque com poucas ancoras (--waypoints
        # baixo, o normal aqui) elas caem longe das pontas em t -- ex.: com 3
        # ancoras, a mais proxima do inicio/fim already fica em t=0.25/0.75,
        # onde sin(pi*t) ainda vale ~0.71 (quase sem afunilar nada). Elevado a
        # 4, o mesmo t=0.25 cai pra ~0.25 -- afunilamento de verdade logo na
        # ancora mais proxima de cada ponta, sem mudar o meio (sin=1 em t=0.5
        # continua 1 em qualquer potencia).
        taper = math.sin(math.pi * t) ** 4
        offset = random.uniform(-jitter, jitter) * taper
        point = base + offset * perp
        point[2] = 0.001
        anchors.append(point)
    anchors.append(target_pos)

    all_points = [anchors[0]]
    n = len(anchors)
    for i in range(n - 1):
        p0 = anchors[i - 1] if i - 1 >= 0 else anchors[i]
        p1 = anchors[i]
        p2 = anchors[i + 1]
        p3 = anchors[i + 2] if i + 2 < n else anchors[i + 1]

        # Conversao Catmull-Rom -> Bezier cubica: os pontos de controle ficam
        # a 1/6 da tangente estimada por diferenca central (p2 - p0)/(p3 - p1)
        # -- e' o que faz a curva passar exatamente por p1 e p2 mantendo
        # continuidade suave de tangente entre trechos consecutivos.
        c1 = p1 + (p2 - p0) / 6.0
        c2 = p2 - (p3 - p1) / 6.0

        curve_points = generate_bezier_curve(p1, c1, c2, p2, points_per_segment)
        all_points.extend(curve_points[1:])

    return smooth_polyline(all_points)

def smooth_polyline(points, iterations=2):
    """Suaviza a polilinha com media ponderada de 3 pontos (Laplaciana), mantendo
    o primeiro e o ultimo ponto FIXOS (start_pos/target_pos nao podem mudar).

    A curva Catmull-Rom ja' e' suave no sentido matematico (continuidade C1),
    mas com poucas ancoras (--waypoints baixo) o angulo entre segmentos
    CONSECUTIVOS da polilinha final ainda pode ficar alto localmente --
    principalmente perto das pontas, onde a tangente e' estimada com um vizinho
    "fantasma" (p0=p1 no comeco, p3=p2 no fim, ver acima) em vez da media dos
    dois lados que os pontos do meio ganham. E' esse pico local de curvatura,
    nao a forma geral da curva, que faz o controle (PID reagindo a line_theta)
    tremer -- ver comentario em follow_line_state.hpp/search_line_state.hpp.
    Isso aqui limpa esses picos sem mudar o formato geral do caminho.
    """
    pts = [p.copy() for p in points]
    n = len(pts)
    if n < 3:
        return pts
    for _ in range(iterations):
        smoothed = [pts[0]]
        for i in range(1, n - 1):
            smoothed.append((pts[i - 1] + 2.0 * pts[i] + pts[i + 1]) / 4.0)
        smoothed.append(pts[-1])
        pts = smoothed
    return pts

def point_at_arc_fraction(points, fraction):
    """Acha o ponto (x, y, z) que fica a `fraction` (0-1) do comprimento de
    arco acumulado da polilinha `points`. Usado pra plantar os checkpoints
    SEMPRE em cima da linha gerada -- pontos fixos no mundo ficariam largados
    no meio do nada assim que a curva mudasse de uma execucao pra outra."""
    if fraction <= 0:
        return points[0]
    if fraction >= 1:
        return points[-1]

    seg_lengths = [float(np.linalg.norm(points[i + 1][:2] - points[i][:2]))
                   for i in range(len(points) - 1)]
    total = sum(seg_lengths)
    target_dist = fraction * total

    acc = 0.0
    for i, seg_len in enumerate(seg_lengths):
        if acc + seg_len >= target_dist or i == len(seg_lengths) - 1:
            t = 0.0 if seg_len < 1e-9 else (target_dist - acc) / seg_len
            return points[i] + t * (points[i + 1] - points[i])
        acc += seg_len
    return points[-1]

def create_path_sdf(points, width=0.3, color=[0, 0, 1, 1], model_name="generated_path", end_marker=True,
                     checkpoint_fractions=None):
    """Cria um arquivo SDF com o caminho gerado"""

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="{model_name}">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
"""

    # Criar segmentos do caminho
    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]

        # Calcular posição e orientação do segmento
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2
        mid_z = (start[2] + end[2]) / 2

        # Calcular comprimento e ângulo
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        if length > 0.001:  # Evitar segmentos muito pequenos
            sdf_content += f"""
    <link name="path_segment_{i}">
      <pose>{mid_x} {mid_y} {mid_z} 0 0 {angle}</pose>
      <visual name="path_visual_{i}">
        <geometry>
          <box>
            <size>{length} {width} 0.002</size>
          </box>
        </geometry>
        <material>
          <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
          <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
          <specular>{color[0]} {color[1]} {color[2]} {color[3]}</specular>
        </material>
      </visual>
      <collision name="path_collision_{i}">
        <geometry>
          <box>
            <size>{length} {width} 0.002</size>
          </box>
        </geometry>
      </collision>
    </link>
"""

    # Checkpoints (estacas amarelas) plantados EM CIMA da linha gerada, nas
    # fracoes de arco pedidas -- ex.: 1/3 e 2/3 (ver missao4_itjbx.sdf). Viram
    # links deste mesmo model, nao um model://checkpoint_N separado, porque
    # so' assim a posicao acompanha a curva a cada regeneracao: um model
    # fixo no mundo tem pose fixa, e a curva muda de uma execucao pra outra.
    if checkpoint_fractions:
        for i, fraction in enumerate(checkpoint_fractions):
            cx, cy, cz = point_at_arc_fraction(points, fraction)
            sdf_content += f"""
    <link name="checkpoint_{i + 1}">
      <pose>{cx} {cy} 0.100000 0 0 0</pose>
      <visual name="estaca_amarela">
        <geometry>
          <cylinder>
            <radius>0.025000</radius>
            <length>0.200000</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.85 0.0 1</ambient>
          <diffuse>1.0 0.85 0.0 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>
    </link>
"""

    # Marcador no final do caminho -- so' faz sentido quando o fim e' um
    # ponto qualquer do passeio aleatorio. Quando o caminho tem um destino
    # fixo (ver generate_path_to_target, ex.: os postes da missao4), o
    # obstaculo la' JA' e' o marcador; colocar outro em cima so' confundiria.
    if end_marker:
        final_point = points[-1]
        sdf_content += f"""
    <include>
      <uri>model://sae/plataforma_com_circulo</uri>
      <pose>{final_point[0]} {final_point[1]} {final_point[2]} 0 0 0</pose>
    </include>
"""

    sdf_content += """
  </model>
</sdf>"""

    return sdf_content

def main():
    parser = argparse.ArgumentParser(description='Gera um caminho aleatório para simulação Gazebo')
    parser.add_argument('--length', type=float, default=20.0, help='Comprimento total do caminho')
    parser.add_argument('--width', type=float, default=0.3, help='Largura do caminho')
    parser.add_argument('--segments', type=int, default=4, help='Número de segmentos principais')
    parser.add_argument('--curve-intensity', type=float, default=1.5, help='Intensidade das curvas')
    parser.add_argument('--color', nargs=4, type=float, default=[0, 0, 1, 1], help='Cor RGBA do caminho')
    parser.add_argument('--output', type=str, default='random_path.sdf', help='Arquivo de saída')
    parser.add_argument('--seed', type=int, help='Seed para reproduzir o mesmo caminho')

    # Novos parâmetros de direção
    parser.add_argument('--initial-direction', type=float, help='Direção inicial em graus (0=Norte, 90=Leste, 180=Sul, 270=Oeste)')
    parser.add_argument('--direction-preference', type=str, choices=['north', 'south', 'east', 'west', 'spiral_left', 'spiral_right', 'zigzag', 'return', 'random', 'straight'],
                       help='Preferência direcional do caminho (inclui "straight" para linha reta)')

    # Destino fixo: quando os dois forem dados, o caminho e' curvo e
    # aleatorio mas TERMINA exatamente nesse ponto (ex.: o vao entre dois
    # postes fixos no mundo) -- ignora --length/--segments/--curve-intensity
    # /--direction-preference, que so' fazem sentido no modo sem destino.
    parser.add_argument('--target-x', type=float, help='Posicao X final fixa (ativa o modo com destino)')
    parser.add_argument('--target-y', type=float, help='Posicao Y final fixa (ativa o modo com destino)')
    parser.add_argument('--start-x', type=float, default=0.0,
                       help='Posicao X inicial (modo com destino) -- afastar da origem alonga a linha')
    parser.add_argument('--start-y', type=float, default=0.0,
                       help='Posicao Y inicial (modo com destino)')
    parser.add_argument('--waypoints', type=int, default=3, help='Ancoras aleatorias entre a perna de saida e o destino (modo com destino)')
    parser.add_argument('--jitter', type=float, default=1.0, help='Deslocamento lateral maximo (m) de cada ancora (modo com destino)')
    parser.add_argument('--exit-angle', type=float, default=None,
                       help='Direcao (graus, convencao atan2) da perna reta que sai da base, '
                            'ANTES de comecar a curvar rumo ao destino. Sem isso (padrao), '
                            'sorteia um angulo aleatorio a cada execucao, excluindo o cone em '
                            'torno da direcao direta ate o destino (ver --exit-exclusion) -- '
                            'a linha nao sai "por acaso" ja apontada pro alvo.')
    parser.add_argument('--exit-distance', type=float, default=None,
                       help='Comprimento (m) da perna de saida. Padrao: 18%% da distancia reta '
                            'inicio->destino (minimo 1.0m).')
    parser.add_argument('--exit-exclusion', type=float, default=20.0,
                       help='Meia-largura (graus) do cone em torno da direcao direta ate o '
                            'destino que o sorteio de --exit-angle evita. So vale quando '
                            '--exit-angle nao e dado.')
    parser.add_argument('--exit-max-angle', type=float, default=90.0,
                       help='Maior offset (graus), pra qualquer lado, que o sorteio de '
                            '--exit-angle permite -- limita o quanto o caminho precisa curvar '
                            'de volta rumo ao destino. So vale quando --exit-angle nao e dado.')
    parser.add_argument('--end-marker', action='store_true', default=None,
                       help='Forca o marcador de fim de caminho mesmo no modo com destino')
    parser.add_argument('--no-end-marker', dest='end_marker', action='store_false',
                       help='Omite o marcador de fim de caminho')

    parser.add_argument('--checkpoint-fractions', type=str, default=None,
                       help='Fracoes (0-1) do comprimento de arco onde plantar estacas '
                            'amarelas, separadas por virgula (ex.: "0.3333,0.6667" para '
                            '1/3 e 2/3). Sem isso, nenhum checkpoint e\' gerado.')

    args = parser.parse_args()

    if args.seed:
        random.seed(args.seed)
        np.random.seed(args.seed)

    has_target = args.target_x is not None and args.target_y is not None

    if has_target:
        points = generate_path_to_target(
            start_pos=np.array([args.start_x, args.start_y, 0.0]),
            target_pos=np.array([args.target_x, args.target_y, 0.001]),
            num_waypoints=args.waypoints,
            jitter=args.jitter,
            exit_angle_deg=args.exit_angle,
            exit_distance=args.exit_distance,
            exit_angle_exclusion_deg=args.exit_exclusion,
            exit_angle_max_deg=args.exit_max_angle,
        )
        # Sem marcador por padrao: o obstaculo fixo no destino ja' marca o
        # fim do caminho. --end-marker forca ligar de volta se precisar.
        end_marker = bool(args.end_marker) if args.end_marker is not None else False
    else:
        # Converter direção inicial de graus para radianos se especificada
        initial_direction = None
        if args.initial_direction is not None:
            # Converter de graus para radianos e ajustar para sistema de coordenadas
            # 0° = Norte (+Y), 90° = Leste (+X), 180° = Sul (-Y), 270° = Oeste (-X)
            initial_direction = math.radians(90 - args.initial_direction)

        points = generate_smooth_path(
            total_length=args.length,
            num_segments=args.segments,
            curve_intensity=args.curve_intensity,
            initial_direction=initial_direction,
            direction_preference=args.direction_preference
        )
        end_marker = args.end_marker if args.end_marker is not None else True

    checkpoint_fractions = None
    if args.checkpoint_fractions:
        checkpoint_fractions = [float(f) for f in args.checkpoint_fractions.split(',') if f.strip()]

    # Criar SDF
    sdf_content = create_path_sdf(
        points=points,
        width=args.width,
        color=args.color,
        model_name="generated_path",
        end_marker=end_marker,
        checkpoint_fractions=checkpoint_fractions,
    )

    # Salvar arquivo
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        f.write(sdf_content)

    print(f"Caminho gerado e salvo em: {output_path}")
    print(f"Pontos no caminho: {len(points)}")
    if has_target:
        print(f"Origem: ({args.start_x}, {args.start_y}) -> Destino fixo: ({args.target_x}, {args.target_y}) "
              f"-- {args.waypoints} ancoras, jitter={args.jitter}m")
        if checkpoint_fractions:
            print(f"Checkpoints em: {checkpoint_fractions} do comprimento de arco")
    else:
        print(f"Comprimento aproximado: {args.length}m")
        if args.initial_direction is not None:
            print(f"Direção inicial: {args.initial_direction}° ({['Norte', 'Nordeste', 'Leste', 'Sudeste', 'Sul', 'Sudoeste', 'Oeste', 'Noroeste'][int((args.initial_direction + 22.5) % 360 // 45)]})")
        if args.direction_preference:
            print(f"Preferência direcional: {args.direction_preference}")

if __name__ == "__main__":
    main()
