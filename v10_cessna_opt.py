import os, sys, numpy as np
from openvsp import openvsp as vsp

def FCN(x: np.ndarray) -> float:
    # === Carrega o modelo base (.vsp3) ===
    VSP3_FILE = r"C:\VSP\Development\PSO_PYTHON_WING\cessna210.vsp3"
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(VSP3_FILE)

    # ID fixo da asa principal
    wing_id = "ITDQSYJOYI"

    # ============================================================
    # 2️⃣ APLICA AS VARIÁVEIS NO MODELO CESSNA
    # ============================================================
    # Variáveis: AR, span, taper, sweep, twist, alpha
    AR, span, taper, sweep, twist, alpha = x

    # --- Define cordas coerentes ---
    croot = 2 * span / (AR * (1 + taper))
    ctip  = taper * croot

    # --- Aplica parâmetros geométricos ---
    vsp.SetParmVal(wing_id, "Span",       "XSec_1", span / 2.0)
    vsp.SetParmVal(wing_id, "Root_Chord", "XSec_1", croot)
    vsp.SetParmVal(wing_id, "Tip_Chord",  "XSec_1", ctip)
    vsp.SetParmVal(wing_id, "Sweep",      "XSec_1", sweep)
    vsp.SetParmVal(wing_id, "Twist",      "XSec_1", twist)

    vsp.Update()
    print(f"[geo] AR={AR:.2f}, Span={span:.2f}, Taper={taper:.2f}, Sweep={sweep:.2f}, Twist={twist:.2f}")
    print(f"[geo] Croot={croot:.3f}, Ctip={ctip:.3f}")

    # ============================================================
    # 3️⃣ RODAR VSPAERO (gerar malha e simulação)
    # ============================================================
    # --- Atualiza e salva o modelo atualizado ---
    vsp.Update()
    vsp.WriteVSPFile(r"C:\VSP\Development\PSO_PYTHON_WING\cessna_updated.vsp3")
    # --- Gera geometria degenerada (malha para o solver) ---
    vsp.SetAnalysisInputDefaults("VSPAEROComputeGeometry")
    vsp.SetIntAnalysisInput("VSPAEROComputeGeometry", "GeomSet", [vsp.SET_ALL])
    vsp.ExecAnalysis("VSPAEROComputeGeometry")
    
    # --- Define o solver principal ---
    solver_id = "VSPAEROSweep"

    # --- Define entradas de análise ---
    sref = vsp.GetParmVal(wing_id, "TotalArea", "WingGeom")
    bref = vsp.GetParmVal(wing_id, "TotalSpan", "WingGeom")

    # --- Corda média aerodinâmica (Cref) calculada analiticamente ---
    # Fórmula do MAC para asa trapezoidal: C_MAC = (2/3)*Croot*((1 + taper + taper²)/(1 + taper))
    # Usa as cordas reais para fornecer uma referência mais física ao solver VSPAERO.
    cref = (2/3) * croot * ((1 + taper + taper**2) / (1 + taper))


    vsp.SetAnalysisInputDefaults(solver_id)
    vsp.SetDoubleAnalysisInput(solver_id, "Sref", [sref])
    vsp.SetDoubleAnalysisInput(solver_id, "Bref", [bref])
    vsp.SetDoubleAnalysisInput(solver_id, "Cref", [cref])
    vsp.SetDoubleAnalysisInput(solver_id, "Rho", [0.002377])   # ar padrão (slug/ft³)
    vsp.SetDoubleAnalysisInput(solver_id, "Vinf", [100.0])     # velocidade em ft/s
    vsp.SetDoubleAnalysisInput(solver_id, "MachStart", [0.3])
    vsp.SetDoubleAnalysisInput(solver_id, "MachEnd", [0.3])
    vsp.SetIntAnalysisInput(solver_id, "MachNpts", [1])
    vsp.SetDoubleAnalysisInput(solver_id, "AlphaStart", [alpha])
    vsp.SetDoubleAnalysisInput(solver_id, "AlphaEnd", [alpha])
    vsp.SetIntAnalysisInput(solver_id, "AlphaNpts", [1])
    vsp.SetIntAnalysisInput(solver_id, "GeomSet", [vsp.SET_ALL])
    vsp.SetIntAnalysisInput(solver_id, "NCPU", [4])

    # --- Executa o solver ---
    vsp.Update()

    # Executa a análise
    vsp.ExecAnalysis(solver_id)

    # --- Leitura simples do arquivo .history ---
    hist_path = r"C:\VSP\Development\PSO_PYTHON_WING\cessna_updated.history"

    with open(hist_path, "r") as f:
        lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith("#")]

    # Pega a última linha (resultados finais)
    last_line = lines[-1].split()

    cl = float(last_line[6])  # coluna CLtot
    cd = float(last_line[9])  # coluna CDtot
    ld = cl / cd

    print(f"[ok] CL={cl:.4f}, CD={cd:.4f}, L/D={ld:.2f}")

    print("[solver] Simulação VSPAERO executada.")


    return -ld  # PSO minimiza, então inverte o sinal

if __name__ == "__main__":
    #AR, span, taper, sweep, twist, alpha
    x_test = np.array([3.75202, 36.0, 1, 0, 0, 2.0])
    resultado = FCN(x_test)
    print("Resultado (fobj):", resultado)
