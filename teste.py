from openvsp import openvsp as vsp

# Limpa qualquer modelo em memória
vsp.ClearVSPModel()

print("\n=== Análises disponíveis ===")
for name in vsp.GetAnalysisNameList():
    print("-", name)
    try:
        print("  Parâmetros:", vsp.GetAnalysisInputNames(name))
    except:
        print("  (sem inputs ou erro ao listar)")


