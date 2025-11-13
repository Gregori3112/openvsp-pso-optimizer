# cessna_opt.py
import os, sys, shutil, math, time

# === CONFIGURAÇÃO PRINCIPAL ===
OPENVSP_PY = r"C:\VSP\OpenVSP\OpenVSP-3.45.2-win64\python"
VSP3_FILE  = r"C:\VSP\Development\PSO_PYTHON_WING\cessna210.vsp3"

MACH    = 0.26
RHO     = 0.002377   # use se quiser forças dimensionais; ok deixar None se GUI está inviscid
AOA_DEG = 2.0
BETA    = 0.0
NCPU    = 4
TAPER   = 1.014
AR_TARGET = None

# === OpenVSP Python API ===
sys.path.insert(0, OPENVSP_PY)
import openvsp.vsp as v

# === UTIL ===
def ensure_dir(p): os.makedirs(p, exist_ok=True)

def chdir_to_vsp():
    wd = os.path.dirname(VSP3_FILE)
    if wd:
        os.chdir(wd)

def delete_old_results(prefix_base):
    """
    Apaga arquivos antigos gerados pelo VSPAERO para evitar leituras de run anterior.
    """
    exts = [".polar", ".history", ".adb", ".dbg", ".log"]
    for ext in exts:
        path = os.path.join(os.getcwd(), f"{prefix_base}{ext}")
        try:
            if os.path.exists(path):
                os.remove(path)
        except Exception as e:
            print(f"[warn] Não removi {path}: {e}")

def parse_history_generic(history_path):
    """
    Parser robusto do .history:
    - encontra header (linha que contém nomes das colunas)
    - identifica índices de Alpha, CL, CD e L/D (se houver)
    - retorna última linha numérica com dict padronizado
    """
    if not os.path.exists(history_path):
        return None

    header = None
    data_rows = []
    try:
        with open(history_path, "r") as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith("#"):
                    continue
                parts = ln.split()
                # heurística: header costuma ter textos não-numéricos típicos (Alpha, CLtot, etc.)
                # e linhas de dados só números
                is_numeric = True
                for p in parts:
                    try:
                        float(p)
                    except ValueError:
                        is_numeric = False
                        break
                if not is_numeric and header is Non
                    header = parts
                elif is_numeric:
                    data_rows.append([float(p) for p in parts])

        if header is None or not data_rows:
            return None

        # mapeia colunas de interesse (flexível quanto ao nome)
        def find_col(*candidates):
            for name in candidates:
                if name in header:
                    return header.index(name)
            return None

        idx_alpha = find_col("Alpha", "alpha")
        idx_cl    = find_col("CLtot", "CL", "CLTotal", "CL_tot")
        idx_cd    = find_col("CDtot", "CD", "CDTotal", "CD_tot")
        idx_ld    = find_col("L/D", "L_over_D", "LoverD", "L_D")

        last = data_rows[-1]
        out = {}
        if idx_alpha is not None: out["alpha"] = last[idx_alpha]
        if idx_cl    is not None: out["CLtot"] = last[idx_cl]
        if idx_cd    is not None: out["CDtot"] = last[idx_cd]

        if idx_ld is not None:
            out["L_over_D"] = last[idx_ld]
        else:
            # calcula se não existir
            if ("CLtot" in out) and ("CDtot" in out) and out["CDtot"] != 0.0:
                out["L_over_D"] = out["CLtot"] / out["CDtot"]

        # válido se tiver L/D
        return out if ("L_over_D" in out) else None

    except Exception as e:
        print(f"[erro leitura history] {e}")
        return None

def read_polar_last_ld(polar_path):
    """
    Lê a última linha numérica do .polar e retorna L/D = CL/CD.
    Útil como fallback se o .history não estiver no formato esperado.
    """
    if not os.path.exists(polar_path):
        return None
    try:
        last = None
        with open(polar_path, "r") as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith("#"):
                    continue
                parts = ln.split()
                # aceita linhas numéricas com pelo menos Alpha, CL, CD...
                is_numeric = True
                for p in parts:
                    try: float(p)
                    except ValueError:
                        is_numeric = False
                        break
                if is_numeric:
                    last = [float(p) for p in parts]
        if last is None:
            return None

        # Heurística comum: colunas no .polar costumam vir como:
        # Alpha, CL, CD, CM, ... (varia conforme versão, mas CL/CD nas pos. 1 e 2 é frequente)
        # Vamos identificar pelo header? Nem sempre há header — então tentamos índices padrão.
        CL = last[1] if len(last) > 1 else None
        CD = last[2] if len(last) > 2 else None
        if CL is None or CD in (None, 0.0):
            return None
        return {"L_over_D": CL / CD, "CLtot": CL, "CDtot": CD}
    except Exception as e:
        print(f"[erro leitura polar] {e}")
        return None

def find_first_wing():
    ids = v.FindGeoms()
    for gid in ids:
        try:
            name = v.GetGeomName(gid) or ""
            if "wing" in name.lower():
                return gid, name
        except:
            continue
    return None, None

def collect_section_parm_ids(wing_id, names, limit_surfs=8):
    out = {n: [] for n in names}
    seen = set()
    for idx in range(limit_surfs):
        try:
            xsid = v.GetXSecSurf(wing_id, idx)
            nx = v.GetNumXSec(xsid)
            for i in range(nx):
                xid = v.GetXSec(xsid, i)
                for nm in names:
                    pid = v.GetXSecParm(xid, nm)
                    if pid and pid not in seen:
                        out[nm].append(pid)
                        seen.add(pid)
        except:
            continue
    return out

def set_uniform(pids, value):
    for pid in pids:
        try: v.SetParmVal(pid, float(value))
        except: pass

def set_taper(section_pids, taper):
    roots = section_pids.get("Root_Chord", [])
    tips  = section_pids.get("Tip_Chord", [])
    if not roots or not tips: return
    try:
        root_val = v.GetParmVal(roots[0])
        v.SetParmVal(tips[0], float(taper) * root_val)
    except: pass

def set_aspect_ratio(wing_id, ar_target):
    if ar_target is None: return
    try:
        S = v.GetParmVal(v.GetParm(wing_id, "TotalArea"))
        b_target = math.sqrt(float(ar_target) * float(S))
        v.SetParmVal(v.GetParm(wing_id, "TotalSpan"), b_target)
    except: pass
    v.Update()

def apply_geometry(wing_id, sweep_deg, twist_deg, taper, ar_target, section_pids):
    set_uniform(section_pids.get("Sweep", []), sweep_deg)
    set_uniform(section_pids.get("Twist", []), twist_deg)
    set_taper(section_pids, taper)
    set_aspect_ratio(wing_id, ar_target)
    v.Update()

def compute_vspaero_geom():
    v.SetAnalysisInputDefaults("VSPAEROComputeGeometry")
    v.SetIntAnalysisInput("VSPAEROComputeGeometry", "GeomSet", [v.SET_ALL])
    v.SetIntAnalysisInput("VSPAEROComputeGeometry", "ThinGeomSet", [v.SET_NONE])
    v.ExecAnalysis("VSPAEROComputeGeometry")

def force_refs_from_wing(wing_id):
    try:
        S = v.GetParmVal(v.GetParm(wing_id, "TotalArea"))
        b = v.GetParmVal(v.GetParm(wing_id, "TotalSpan"))
        c = S / b if b > 0 else None
        for (container, group, pname, val) in [
            ("VSPAERO", "VSPAERO", "Sref", S),
            ("VSPAERO", "VSPAERO", "Bref", b),
            ("VSPAERO", "VSPAERO", "Cref", c),
            ("Global",  "VSPAERO", "Sref", S),
            ("Global",  "VSPAERO", "Bref", b),
            ("Global",  "VSPAERO", "Cref", c),
        ]:
            pid = v.FindParm(container, pname, group)
            if pid:
                v.SetParmVal(pid, float(val))
    except: pass
    v.Update()

def run_vspaero(base_name, mach, aoa_deg, beta_deg=0.0, ncpu=4, rho=None, timeout=60):
    """
    Configura um sweep “degenerado” (1 ponto) exatamente como no GUI:
    MachNpts=1, AlphaNpts=1, BetaNpts=1.
    """
    v.SetAnalysisInputDefaults("VSPAEROSweep")
    v.SetIntAnalysisInput("VSPAEROSweep", "GeomSet",      [v.SET_ALL])
    v.SetIntAnalysisInput("VSPAEROSweep", "ThinGeomSet",  [v.SET_NONE])
    v.SetIntAnalysisInput("VSPAEROSweep", "NCPU",         [ncpu])

    v.SetDoubleAnalysisInput("VSPAEROSweep", "MachStart",  [mach])
    v.SetDoubleAnalysisInput("VSPAEROSweep", "MachEnd",    [mach])
    v.SetIntAnalysisInput(   "VSPAEROSweep", "MachNpts",   [1])
    v.SetDoubleAnalysisInput("VSPAEROSweep", "AlphaStart", [aoa_deg])
    v.SetDoubleAnalysisInput("VSPAEROSweep", "AlphaEnd",   [aoa_deg])
    v.SetIntAnalysisInput(   "VSPAEROSweep", "AlphaNpts",  [1])
    v.SetDoubleAnalysisInput("VSPAEROSweep", "BetaStart",  [beta_deg])
    v.SetDoubleAnalysisInput("VSPAEROSweep", "BetaEnd",    [beta_deg])
    v.SetIntAnalysisInput(   "VSPAEROSweep", "BetaNpts",   [1])

    # Se quiser dimensionais (como no GUI com densidade definida)
    if rho is not None:
        v.SetDoubleAnalysisInput("VSPAEROSweep", "Rho", [float(rho)])

    # Redirecionar log (opcional, igual ao GUI)
    v.SetStringAnalysisInput("VSPAEROSweep", "RedirectFile", ["vspaero_run.log"])

    # Executa
    v.ExecAnalysis("VSPAEROSweep")

    # Espera .history e/ou .polar
    wd = os.getcwd()
    history_path = os.path.join(wd, f"{base_name}.history")
    polar_path   = os.path.join(wd, f"{base_name}.polar")

    waited = 0.0
    while waited < timeout and not (os.path.exists(history_path) or os.path.exists(polar_path)):
        time.sleep(0.5)
        waited += 0.5

    paths = {"history": history_path if os.path.exists(history_path) else None,
             "polar":   polar_path   if os.path.exists(polar_path)   else None}

    if not paths["history"] and not paths["polar"]:
        print(f"[ERRO] Nenhum arquivo .history/.polar gerado após {timeout}s (prefixo: {base_name})")
        return None

    if paths["history"]:
        print(f"[✔] .history gerado: {paths['history']}")
    if paths["polar"]:
        print(f"[✔] .polar gerado:   {paths['polar']}")

    return paths

def avaliar_objetivo(x):
    sweep_deg, twist_deg = x
    try:
        # 1) Consistência de diretório (igual GUI)
        chdir_to_vsp()

        # 2) Carregar o mesmo .vsp3 do GUI
        v.ClearVSPModel()
        v.ReadVSPFile(VSP3_FILE)
        v.Update()

        base = os.path.splitext(os.path.basename(VSP3_FILE))[0]

        # 3) Limpa resultados antigos daquele prefixo
        delete_old_results(base)

        # 4) Seleciona primeira asa e coleta parâmetros
        wing_id, _ = find_first_wing()
        if not wing_id:
            print("[ERRO] Nenhuma asa encontrada.")
            return 1e6

        section_pids = collect_section_parm_ids(
            wing_id,
            ["Sweep", "Twist", "Root_Chord", "Tip_Chord", "Span"]
        )

        # 5) Aplica geometria e referenciais
        print(f"[INFO] Simulando: Sweep={sweep_deg:.2f}, Twist={twist_deg:.2f}")
        apply_geometry(wing_id, sweep_deg, twist_deg, TAPER, AR_TARGET, section_pids)
        compute_vspaero_geom()
        force_refs_from_wing(wing_id)

        # 6) Run VSPAERO (1 ponto, como no GUI)
        paths = run_vspaero(base, MACH, AOA_DEG, beta_deg=BETA, ncpu=NCPU, rho=RHO, timeout=90)
        if not paths:
            return 1e6

        # 7) Lê resultado: prioriza .history (mais rico), senão .polar
        res = None
        if paths["history"]:
            res = parse_history_generic(paths["history"])
        if not res and paths["polar"]:
            res = read_polar_last_ld(paths["polar"])

        if not res or "L_over_D" not in res:
            print(f"[⚠ penalidade] Sweep={sweep_deg:.2f}, Twist={twist_deg:.2f} → sem L/D legível.")
            return 1e6

        ld = float(res["L_over_D"])
        print(f"[✔] Sweep={sweep_deg:.2f}, Twist={twist_deg:.2f} → L/D = {ld:.4f}")
        return -ld

    except Exception as e:
        print(f"[erro] Sweep={sweep_deg:.2f}, Twist={twist_deg:.2f} → {e}")
        return 1e6

# Teste manual
if __name__ == "__main__":
    resultado = avaliar_objetivo([10, -3])
    print(f"Resultado da função objetivo: {resultado:.4f}")
