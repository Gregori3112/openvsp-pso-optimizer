# ============================================================
# Script de análise e otimização da asa Cessna 210 via OpenVSP + VSPAERO
# ------------------------------------------------------------
# Autor: Gregori da Maia da Silva
# Função principal (FCN): calcula o L/D a partir de parâmetros geométricos
# ============================================================

import os, sys, numpy as np
from openvsp import openvsp as vsp


def FCN(x: np.ndarray) -> float:

    """
    Função objetivo que executa uma simulação VSPAERO
    com base em variáveis geométricas da asa e retorna o valor -L/D
    (para ser usado num algoritmo de otimização, como o PSO).
    """

    for f in os.listdir(r"C:\VSP\Development\PSO_PYTHON_WING"):
        if f.startswith("cessna_updated.") or f.startswith("temp_polar."):
            try:
                os.remove(os.path.join(r"C:\VSP\Development\PSO_PYTHON_WING", f))
            except PermissionError:
                pass



    # === 1️⃣ CARREGAMENTO DO MODELO BASE ===
    # Caminho do arquivo do modelo original (.vsp3)
    VSP3_FILE = r"C:\VSP\Development\PSO_PYTHON_WING\cessna210.vsp3"


    # Limpa qualquer modelo aberto anteriormente e lê o novo   
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(VSP3_FILE)

    # ID fixo da asa principal e nome do solver
    wing_id = "ITDQSYJOYI"
    solver_id = "VSPAEROSweep"


    # ============================================================
    # 2️⃣ APLICA AS VARIÁVEIS NO MODELO GEOMÉTRICO
    # ============================================================
    # As variáveis de entrada representam:
    # AR = razão de aspecto, span = envergadura total, taper = razão de afilamento,
    # sweep = enflechamento, twist = torção, alpha = ângulo de ataque.
    AR, span, taper, sweep, twist = x

    # --- Calcula cordas coerentes com o AR e taper definidos ---
    croot = 2 * span / (AR * (1 + taper))
    ctip  = taper * croot

    # --- Aplica parâmetros geométricos diretamente no modelo OpenVSP ---
    vsp.SetParmVal(wing_id, "Span",       "XSec_1", span / 2.0)  # Span/2 pois cada semi-asa
    vsp.SetParmVal(wing_id, "Root_Chord", "XSec_1", croot)
    vsp.SetParmVal(wing_id, "Tip_Chord",  "XSec_1", ctip)
    vsp.SetParmVal(wing_id, "Taper", "XSec_1", taper) 
    vsp.SetParmVal(wing_id, "Sweep",      "XSec_1", sweep)
    vsp.SetParmVal(wing_id, "Twist",      "XSec_1", twist)


    # Atualiza o modelo na memória do VSP
    vsp.Update()


    print(f"[geo] AR={AR:.2f}, Span={span:.2f}, Taper={taper:.2f}, Sweep={sweep:.2f}, Twist={twist:.2f}")
    print(f"[geo] Croot={croot:.3f}, Ctip={ctip:.3f}")


    # ============================================================
    # 3️⃣ EXECUÇÃO DO SOLVER AERODINÂMICO (VSPAERO)
    # ============================================================
    vsp.Update()
    vsp.WriteVSPFile(r"C:\VSP\Development\PSO_PYTHON_WING\cessna_updated.vsp3")
 
    # Configura o gerador de malha (VSPAEROComputeGeometry)    
    vsp.SetAnalysisInputDefaults("VSPAEROComputeGeometry")
    vsp.SetIntAnalysisInput("VSPAEROComputeGeometry", "GeomSet", [vsp.SET_ALL])
    vsp.ExecAnalysis("VSPAEROComputeGeometry")

    # ------------------------------------------------------------
    # 3.1 Configura o solver VSPAERO propriamente dito
    # ------------------------------------------------------------
    vsp.SetAnalysisInputDefaults(solver_id)  # reseta parâmetros do solver
    # === Desativa saídas opcionais (.polar e .slc) para evitar erros ===
    available_inputs = []
    try:
        available_inputs = vsp.GetAnalysisInputNames(solver_id)
    except:
        pass

    if "PolarFileName" in available_inputs:
        vsp.SetStringAnalysisInput(solver_id, "PolarFileName", [""])
    if "SliceFileName" in available_inputs:
        vsp.SetStringAnalysisInput(solver_id, "SliceFileName", [""])
    if "NumSlices" in available_inputs:
        vsp.SetIntAnalysisInput(solver_id, "NumSlices", [0])




    # ------------------------------------------------------------
    # 3.2 Define parâmetros principais de análise
    # ------------------------------------------------------------
    sref = vsp.GetParmVal(wing_id, "TotalArea", "WingGeom")
    bref = vsp.GetParmVal(wing_id, "TotalSpan", "WingGeom")
    cref = (2/3) * croot * ((1 + taper + taper**2) / (1 + taper))

    # --- Condições de voo e propriedades do ar ---
    W = 1800 * 9.81                     # peso [N]
    rho = 0.002377 * 515.379            # densidade do ar [kg/m³] (ISA nível do mar)
    T = 288.15                          # temperatura [K]
    gamma = 1.4
    R = 287.05
    M = 0.3                             # número de Mach desejado
    a = (gamma * R * T) ** 0.5          # velocidade do som [m/s]
    V = M * a                           # velocidade de voo [m/s]
    S = sref * (0.3048 ** 2)            # área da asa [m²]
    hist_path = r"C:\VSP\Development\PSO_PYTHON_WING\cessna_updated.history"

    print(f"[flight] Mach={M:.2f}  →  V={V:.2f} m/s  ({V/0.3048:.1f} ft/s)")


    # ------------------------------------------------------------
    # Ajuste automático do ângulo de ataque para L ≈ W
    # ------------------------------------------------------------
    target_L = W
    tol_L = 0.05 * W
    alpha = 0.0
    step = 0.5

    for _ in range(4):
        # Define o alpha atual e executa o solver
        vsp.SetIntAnalysisInput(solver_id, "NumWakeNodes", [32])
        vsp.SetIntAnalysisInput(solver_id, "NCPU", [4])
        vsp.SetDoubleAnalysisInput(solver_id, "Sref", [sref])
        vsp.SetDoubleAnalysisInput(solver_id, "Rho", [0.002377])
        vsp.SetDoubleAnalysisInput(solver_id, "Vinf", [V / 0.3048])
        vsp.SetDoubleAnalysisInput(solver_id, "MachStart", [M])
        vsp.SetDoubleAnalysisInput(solver_id, "MachEnd", [M])
        vsp.SetIntAnalysisInput(solver_id, "MachNpts", [1])
        vsp.SetDoubleAnalysisInput(solver_id, "AlphaStart", [alpha])
        vsp.SetDoubleAnalysisInput(solver_id, "AlphaEnd", [alpha])
        vsp.SetIntAnalysisInput(solver_id, "AlphaNpts", [1])
        vsp.SetIntAnalysisInput(solver_id, "GeomSet", [vsp.SET_ALL])


        vsp.ExecAnalysis(solver_id)


        import time
        for _ in range(10):
            if os.path.exists(hist_path):
                break
            time.sleep(0.5)

        # Lê o .history para pegar CL e CD
        with open(hist_path, "r") as f:
            lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith("#")]
        last_line = lines[-1].split()
        cl = float(last_line[6])
        cd = float(last_line[9])

        # Calcula sustentação e erro
        L = 0.5 * rho * V**2 * S * cl
        error = (L - target_L) / target_L

        if abs(error) < 0.05:
            break
        alpha -= step * error  # ajusta ângulo de ataque

    print(f"[auto-alpha] Alpha ajustado para {alpha:.2f}° com L={L:.1f} N")

    # ============================================================
    # 4️⃣ LEITURA E AVALIAÇÃO DOS RESULTADOS
    # ============================================================

    # Caminho do arquivo de resultados (.history)
    hist_path = r"C:\VSP\Development\PSO_PYTHON_WING\cessna_updated.history"

    # Lê o arquivo, ignorando linhas de cabeçalho
    with open(hist_path, "r") as f:
        lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith("#")]

    # Pega a última linha (resultado final)
    last_line = lines[-1].split()

    # Extrai CL, CD e calcula razão L/D
    cl = float(last_line[6])  # coluna CLtot
    cd = float(last_line[9])  # coluna CDtot
    ld = cl / cd

    # ------------------------------------------------------------
    # 4.1 Aplica penalização caso L saia da faixa aceitável
    # ------------------------------------------------------------

    print(f"[flight] Mach={M:.2f}  →  V={V:.2f} m/s ({V/0.3048:.1f} ft/s)")


    # Calcula sustentação real em newtons
    L = 0.5 * rho * V**2 * S * cl

    # Define margens de ±5%
    L_min = W * 0.95
    L_max = W * 1.05

    if L < L_min or L > L_max:
        penalty = 100 * abs((L - W) / W) ** 1.5  # penaliza proporcionalmente ao desvio relativo
        print(f"[penalty] L fora do intervalo: {L:.1f} N (peso={W:.1f} N, penalidade={penalty:.2f})")
    else:
        penalty = 0

    # Mostra resultados principais
    print(f"[ok] CL={cl:.4f}, CD={cd:.4f}, L={L:.2f}, L/D={ld:.2f}")
    print(f"[status] Sustentação {'OK' if penalty == 0 else 'fora da faixa'} | α={alpha:.2f}°, L/D={ld:.2f}")
    print("[solver] Simulação VSPAERO executada.")

    # ============================================================
    # 5️⃣ FUNÇÃO OBJETIVO PARA OTIMIZAÇÃO
    # ============================================================
    # O PSO tenta minimizar → então usamos -L/D para maximizar L/D
    fobj = -ld + penalty

    # --- Encerra e limpa completamente o modelo atual para liberar memória ---
    import gc
    import time
    vsp.ClearVSPModel()
    time.sleep(2)
    gc.collect()
    print(f"[done] Iteração finalizada: fobj={fobj:.4f}, L/D={ld:.2f}")
    print(f"[obj] fobj={fobj:.2f}, -L/D={-ld:.2f}, penalty={penalty:.2f}")


    # Retorna: função objetivo, CL, CD e L/D
    return fobj, {"CL": cl, "CD": cd, "LD": ld, "Alpha": alpha, "L": L}






# ============================================================
# Execução isolada (modo de teste)
# ============================================================
if __name__ == "__main__":
    # Vetor de teste: [AR, span, taper, sweep, twist]
    x_test = np.array([7.5, 36.0, 1, 0, 0])

    # Executa uma simulação e imprime o resultado final
    resultado = FCN(x_test)
    print("Resultado:", resultado)
