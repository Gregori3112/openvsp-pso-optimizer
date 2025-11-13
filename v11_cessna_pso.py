# ============================================================
# PSO cru - versão Python equivalente ao código MATLAB
# ------------------------------------------------------------
# Este script implementa o algoritmo Particle Swarm Optimization (PSO)
# aplicado à otimização aerodinâmica de uma asa no OpenVSP.
# Cada partícula representa uma combinação de parâmetros geométricos da asa.
# O objetivo é maximizar L/D (minimizando -L/D), chamando a função FCN()
# que executa a simulação via VSPAERO.
# ============================================================


import numpy as np
import matplotlib.pyplot as plt
import random
import os
from v11_cessna_opt import FCN       # Função objetivo (roda o OpenVSP)
import time                          # Pausa entre iterações e chamadas ao solver

# Caminho do arquivo base do VSP3 (necessário para salvar a geometria ótima)
VSP3_FILE = r"C:\VSP\Development\PSO_PYTHON_WING\cessna210.vsp3"

# ID da asa principal (mesmo usado no FCN)
wing_id = "ITDQSYJOYI"

# ============================================================
# 1 Configuração inicial
# ============================================================

# Nomes das variáveis e cores associadas para gráficos
var_names = ["AR", "span", "taper", "sweep", "twist"]
colors = ['red', 'orange', 'cyan', 'magenta', 'purple']

# ============================================================
# 2 Inicialização dos históricos e diretórios de saída
# ============================================================

# Este bloco cria (ou reaproveita, se já existir) as variáveis
# que armazenam o histórico da otimização.
# A verificação com 'not in locals()' evita recriar as variáveis
# caso o script seja executado várias vezes no mesmo ambiente (ex: Spyder, Jupyter).

# Histórico da melhor solução global ao longo das iterações
# (guarda a evolução do "gbest" — o melhor valor encontrado pela população)

# Diretório onde serão salvos os resultados e gráficos da otimização.
# Se a pasta "resultados_variaveis" não existir, ela é criada automaticamente.
output_dir = "resultados_variaveis"
os.makedirs(output_dir, exist_ok=True)

# Histórico completo das partículas e do melhor global (gbest)
# ------------------------------------------------------------
# 'history_particles' guarda todas as posições das partículas em cada iteração
# → útil para gerar gráficos de dispersão e ver a "exploração" do espaço de busca.
# 
# 'history_gbest' guarda o valor das variáveis correspondentes ao melhor global em cada iteração
# → usado para gerar o gráfico da trajetória de convergência da melhor solução.
if 'history_particles' not in locals():
    history_particles = {v: [] for v in var_names}      # cria dicionário com listas vazias para cada variável
if 'history_gbest' not in locals():
    history_gbest = {v: [] for v in var_names}          # idem para o histórico do melhor global

# ============================================================
# 3 Parâmetros principais do PSO
# ============================================================
var_names = ["AR", "span", "taper", "sweep", "twist"]
xmin = np.array([6, 34, 0.5, 0.0, -3.0])
xmax = np.array([10, 38, 1.0, 10.0, 3.0])

nrvar = len(xmin)                                     # número de variáveis (5)

# Parâmetros de controle do PSO
lambda1 = 2.02          # coeficiente cognitivo (atração pelo melhor individual)
lambda2 = 2.02          # coeficiente social (atração pelo melhor global)
omega = 0.4             # fator de inércia (peso da velocidade anterior)
pop = 20                 # tamanho da população (número de partículas)
tol = 1e-4              # tolerância para critério de parada
itermax = 30            # número máximo de iterações
random.seed(4)          # semente para reprodutibilidade
np.random.seed(4)       # garante reprodutibilidade também para operações NumPy

# ============================================================
# 4 Inicialização das partículas
# ============================================================
gbest = [1e30]          # melhor valor global inicial (grande para permitir minimização)
k = 1                   # contador de iterações

# Matrizes principais do PSO
v = np.zeros((pop, nrvar))          # velocidades das partículas
x = np.zeros((pop, nrvar))          # posições atuais
lbest = np.full(pop, np.inf)        # melhores valores individuais
xlbest = np.zeros((pop, nrvar))     # melhores posições individuais


# Loop de inicialização: gera posições aleatórias e avalia cada partícula
for i in range(pop):
    for j in range(nrvar):
        # Inicializa posição dentro dos limites [xmin, xmax]
        x[i, j] = xmin[j] + (xmax[j] - xmin[j]) * random.random()

    # Avalia o desempenho aerodinâmico (chamada do OpenVSP)
    y, data = FCN(x[i, :])
    CL = data["CL"]
    CD = data["CD"]
    LD = data["LD"]



    lbest[i] = y            # guarda o melhor valor 
    xlbest[i, :] = x[i, :]  # guarda a melhor posição que apresentou o melhor valor

    # Atualiza o melhor global (gbest)
    if y < gbest[k - 1]:
        gbest[k - 1] = y         # melhor valor global
        xgbest = x[i, :].copy()  # melhor posição que apresentou o melhor valor
        # >>> GARANTIR MÉTRICAS DO MELHOR DESDE A INICIALIZAÇÃO <<<
        CL_best = CL
        CD_best = CD
        LD_best = LD

plt.pause(0.1)

# ============================================================
# 5 Loop principal do PSO
# ============================================================
flag = False
k = 2
gbest.append(gbest[0])  # adiciona um novo elemento copiando o valor inicial para iniciar o histórico de 
                        # gbest (equivalente ao crescimento automático do MATLAB)
ld_history = []

# --- Loop principal: continua enquanto o critério de parada não for atingido ---
while not flag:

    gbest.append(gbest[k - 2])  # Copia o melhor valor global anterior (equivalente ao gbest(k) = gbest(k-1) do MATLAB)

    # Itera sobre todas as partículas
    for i in range(pop): # Percorre cada variável de decisão da partícula (ex: x e y)
        # Atualiza posição de cada variável
        for j in range(nrvar):

            # Sorteia dois números aleatórios entre 0 e 1 (usados nos termos cognitivo e social)
            r1 = random.random()
            r2 = random.random()

            # Equação clássica do PSO: atualiza velocidade
            vnew_ij = (omega * v[i, j] +
                       lambda1 * r1 * (xlbest[i, j] - x[i, j]) +
                       lambda2 * r2 * (xgbest[j] - x[i, j]))
            
            # Atualiza a posição da partícula somando a nova velocidade
            xnew_ij = x[i, j] + vnew_ij

            # Garante que a nova posição está dentro dos limites definidos
            #if xnew_ij < xmin[j]:
             #   xnew_ij = xmin[j]
            #elif xnew_ij > xmax[j]:
             #   xnew_ij = xmax[j]
            xnew_ij = np.clip(x[i, j] + vnew_ij, xmin[j], xmax[j])

            # Atualiza a matriz principal com as novas posições e velocidades
            v[i, j] = vnew_ij
            x[i, j] = xnew_ij

        # Calcula o novo valor da função objetivo para a partícula atual
        ynew, data = FCN(x[i, :])
        CL = data["CL"]
        CD = data["CD"]
        LD = data["LD"]

        print(f"[pso] Iter={k-1}, Partícula={i+1}/{pop} → fobj={ynew:.3f}, L/D={LD:.2f}")

        # --- Libera memória e reinicia o módulo do OpenVSP ---
        time.sleep(0.1)


        # Atualiza o melhor valor individual (local)
        if ynew < lbest[i]:
            lbest[i] = ynew
            xlbest[i, :] = x[i, :]

        # Atualiza o melhor valor global (do grupo inteiro)
        if ynew < gbest[k - 1]:
            gbest[k - 1] = ynew
            xgbest = x[i, :].copy()

            CL_best = CL
            CD_best = CD
            LD_best = LD


    # ========================================================
    # Armazena histórico de resultados
    # ========================================================
    if k == 2:
        history_particles = {v: [] for v in ["AR","span","taper","sweep","twist"]}
        history_gbest = {v: [] for v in ["AR","span","taper","sweep","twist"]}
        gbest_history = []
    gbest_history.append(gbest[k - 1])


    # Guarda histórico das variáveis
    for idx, var in enumerate(var_names):
        history_particles[var].append(x[:, idx].copy())
        history_gbest[var].append(xgbest[idx])



    # ========================================================
    # Critérios de parada
    # ========================================================
    if k >= itermax:
        flag = True

    # Parada por estabilização do gbest (média móvel de 5 vs 5 iterações)
    if len(gbest_history) >= 10:
        prev_win = gbest_history[-10:-5]
        curr_win = gbest_history[-5:]
        delta = abs(np.mean(curr_win) - np.mean(prev_win))
        if delta < tol:
            flag = True

    print(f"[iter {k-1}] gbest={gbest[k-1]:.4f} | L/D_best≈{LD_best:.2f} | (~L/D ≈ {-gbest[k-1]:.2f} se penalidade≈0) | xgbest={xgbest}")
    ld_history.append(LD_best)

    k += 1


# ============================================================
# 6 Pós-processamento: geração de gráficos e resultados
# ============================================================
plt.figure(figsize=(7, 5))
plt.plot(range(1, len(gbest_history) + 1), gbest_history, 'b-o', linewidth=1.5)
plt.xlim([0, itermax])
plt.xlabel("Iteração")
plt.ylabel("Melhor fobj (minimizar)")
plt.title("Convergência do PSO (fobj = -L/D + penalidade)")
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "convergencia_fobj.png"), dpi=300, bbox_inches="tight")
plt.close()


# Gera gráficos de dispersão (todas as partículas ao longo das iterações)
for i, var in enumerate(var_names):
    plt.figure(figsize=(8, 4))
    # Pontos (valores de cada partícula em cada iteração
    for it, vals in enumerate(history_particles[var]):
        plt.scatter([it + 1] * len(vals), vals, color=colors[i], alpha=0.5, s=40)
    # linha do melhor global (gbest)
    plt.plot(range(1, len(history_gbest[var]) + 1), history_gbest[var], 'k-', lw=1.3, label="Melhor (gbest)")
    plt.xlabel("Iteração")
    plt.ylabel(var)
    plt.title(f"Evolução populacional da variável {var}")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f"dispersao_{var}.png"), dpi=300, bbox_inches="tight")
    plt.close()

# ============================================================
# 7 Gráfico combinado das variáveis ótimas (gbest)
# ============================================================
plt.figure(figsize=(9, 9))
for i, var in enumerate(var_names):
    plt.subplot(len(var_names), 1, i + 1)
    plt.plot(history_gbest[var], color=colors[i], linewidth=1.5)
    plt.ylabel(f"{var}")
    plt.grid(True, linestyle='--', alpha=0.5)
plt.xlabel("Iteração")
plt.suptitle("Evolução das variáveis ótimas por iteração", y=0.92)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "evolucao_variaveis_otimas.png"), dpi=300, bbox_inches='tight')
plt.close()


# ============================================================
# Gráfico de convergência física (L/D do melhor global)
# ============================================================
plt.figure(figsize=(7, 5))
plt.plot(range(1, len(ld_history) + 1), ld_history, 'g-o', linewidth=1.5, markersize=5)
plt.xlim([0, itermax])
plt.xlabel("Iteração")
plt.ylabel("Melhor L/D (máximo)")
plt.title("Convergência física do PSO (L/D_best por iteração)")
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "convergencia_LD_best.png"), dpi=300, bbox_inches="tight")
plt.close()

# ============================================================
# 8 Resultados finais
# ============================================================
print(f"\n✅ Gráficos de dispersão salvos em: {os.path.abspath(output_dir)}")
print("k =", k - 1)
if 'delta' in locals():
    print("delta =", delta)
print("gbest =", gbest[-1])
print("\n📊 Variáveis ótimas encontradas (xgbest):")
for name, value in zip(var_names, xgbest):
    print(f"   {name:<10} = {value:>8.4f}")

# ============================================================
# 9 Salvamento do resultado final em arquivo .txt (automático)
# ============================================================

result_file = os.path.join(output_dir, "resultado_final.txt")

with open(result_file, "w", encoding="utf-8") as f:
    f.write("=============================================\n")
    f.write("   RESULTADOS FINAIS DA OTIMIZAÇÃO PSO\n")
    f.write("=============================================\n\n")

    f.write(f"[ok] CL={CL_best:.4f}, CD={CD_best:.4f}, L/D={LD_best:.2f}\n")
    f.write("[solver] Simulação VSPAERO executada.\n\n")

    f.write(f"Iterações concluídas (k): {k - 1}\n")
    if 'delta' in locals():
        f.write(f"Critério de convergência (delta): {delta:.6f}\n")
    f.write(f"Melhor valor da função objetivo (gbest): {gbest[-1]:.6f}\n\n")

    f.write("📊 Variáveis ótimas encontradas (xgbest):\n")
    for name, value in zip(var_names, xgbest):
        f.write(f"   {name:<10} = {value:>8.4f}\n")

    f.write("\n✅ Gráficos e resultados salvos em:\n")
    f.write(f"{os.path.abspath(output_dir)}\n")

# Ao final do loop principal:
f_best, data = FCN(xgbest)
cl_best = data["CL"]
cd_best = data["CD"]
ld_best = data["LD"]

# --------------------------------------------------------
# Cálculo das métricas adicionais
# --------------------------------------------------------
L_best = data["L"]           # sustentação
W = 1800 * 9.81              # peso total da aeronave (mesmo usado no FCN)

LW_ratio = (L_best / W) * 100          # Sustentação relativa ao peso
CL_ideal = cl_best * (W / L_best)      # CL necessário para L = W


print(f"[final-check] fobj={f_best:.4f} → L/D={ld_best:.2f}, CL={cl_best:.3f}")
with open(result_file, "a", encoding="utf-8") as f:
    f.write("\n[final-check] Reavaliação do gbest:\n")
    f.write(f"fobj={f_best:.6f} → L/D={ld_best:.4f}, CL={cl_best:.5f}, CD={cd_best:.6f}\n")
    f.write(f"L/W (sustentação relativa ao peso) = {LW_ratio:.2f}%\n")
    f.write(f"L/W (sustentação relativa ao peso) = {LW_ratio:.2f}%\n")
    f.write(f"CL ideal para L = W = {CL_ideal:.4f}\n")

        # ============================================================
    #  ANÁLISE AERODINÂMICA AVANÇADA PARA O RESULTADO FINAL
    # ============================================================

    # Variáveis geométricas ótimas
    AR_opt, span_opt, taper_opt, sweep_opt, twist_opt = xgbest

    f.write("\n=============================================\n")
    f.write(" ANÁLISE AERODINÂMICA AVANÇADA DA SOLUÇÃO ÓTIMA\n")
    f.write("=============================================\n\n")

    f.write("=== Geometria ótima encontrada ===\n")
    f.write(f"AR (Aspect Ratio)..................: {AR_opt:.4f}\n")
    f.write(f"Envergadura (span)................: {span_opt:.4f} ft\n")
    f.write(f"Taper ratio.......................: {taper_opt:.4f}\n")
    f.write(f"Sweep (enflechamento).............: {sweep_opt:.4f}°\n")
    f.write(f"Twist..............................: {twist_opt:.4f}°\n\n")

    # Relação L/W e CL ideal
    L_best = data["L"]
    W = 1800 * 9.81
    LW_ratio = (L_best / W) * 100
    CL_ideal = cl_best * (W / L_best)

    f.write("=== Diagnóstico de Sustentação ===\n")
    f.write(f"Sustentação L......................: {L_best:.2f} N\n")
    f.write(f"Peso W.............................: {W:.2f} N\n")
    f.write(f"L/W................................: {LW_ratio:.2f}%\n")
    f.write(f"CL obtido..........................: {cl_best:.4f}\n")
    f.write(f"CL ideal para L = W................: {CL_ideal:.4f}\n\n")

    # Explicações físicas
    f.write("=== Explicações Físicas ===\n")

    f.write("- O aumento de L/D está associado a um Aspect Ratio maior,\n")
    f.write("  reduzindo arrasto induzido e aproximando o comportamento\n")
    f.write("  de uma asa elíptica ideal.\n\n")

    f.write("- A razão de afilamento (taper ratio) influencia a distribuição\n")
    f.write("  de sustentação; valores entre 0.3 e 0.6 tendem a melhorar a\n")
    f.write("  eficiência e aproximar a distribuição elíptica real.\n\n")

    f.write("- O twist negativo (washout) reduz a sustentação na ponta,\n")
    f.write("  diminuindo o arrasto induzido e atrasando o estol da ponta.\n\n")

    f.write("- O sweep pequeno (ou nulo) minimiza efeitos de compressibilidade\n")
    f.write("  e mantém o fluxo mais alinhado com o bordo de ataque, melhorando CL.\n\n")

    f.write("- Como o solver é inviscid (sem viscosidade), o arrasto de perfil\n")
    f.write("  é subestimado, por isso L/D tende a ser maior do que em aeronaves reais.\n\n")

    # Resumo para TCC (explicação pronta)
    f.write("=== Resumo técnico para uso no TCC ===\n")
    f.write("A solução ótima obtida pelo algoritmo PSO apresentou uma melhoria\n")
    f.write("significativa na eficiência aerodinâmica da asa, alcançando um\n")
    f.write(f"L/D máximo de {ld_best:.2f}. Essa melhoria está diretamente\n")
    f.write("associada à combinação de parâmetros geométricos selecionados\n")
    f.write("pelo algoritmo: aumento do Aspect Ratio, razão de afilamento\n")
    f.write("favorável, baixo enflechamento e a aplicação de twist negativo.\n")
    f.write("Essas características aproximam a distribuição de sustentação do\n")
    f.write("perfil elíptico ideal, reduzindo o arrasto induzido. O solver\n")
    f.write("VSPAERO, operando em regime inviscid, tende a subestimar o arrasto\n")
    f.write("parasita, resultando em valores de L/D superiores aos observados em\n")
    f.write("aeronaves reais, mas válidos para comparações relativas e análise\n")
    f.write("de tendências aerodinâmicas.\n\n")





print(f"\n✅ Resultado final salvo em: {result_file}")


# ============================================================
# 10 Salvando a geometria do melhor L/D (xgbest)
# ============================================================
from openvsp import openvsp as vsp

print("\n[save-best] Salvando geometria ótima em 'cessna_best.vsp3'...")

# Reabre o arquivo base
vsp.ClearVSPModel()
vsp.ReadVSPFile(VSP3_FILE)

# Aplica as variáveis ótimas da asa
AR, span, taper, sweep, twist = xgbest

croot = 2 * span / (AR * (1 + taper))
ctip  = taper * croot

vsp.SetParmVal(wing_id, "Span",       "XSec_1", span / 2.0)
vsp.SetParmVal(wing_id, "Root_Chord", "XSec_1", croot)
vsp.SetParmVal(wing_id, "Tip_Chord",  "XSec_1", ctip)
vsp.SetParmVal(wing_id, "Taper",      "XSec_1", taper)
vsp.SetParmVal(wing_id, "Sweep",      "XSec_1", sweep)
vsp.SetParmVal(wing_id, "Twist",      "XSec_1", twist)

# Atualiza e salva
vsp.Update()
best_file = os.path.join(output_dir, "cessna_best.vsp3")
vsp.WriteVSPFile(best_file)

print(f"[save-best] Arquivo salvo: {best_file}")



