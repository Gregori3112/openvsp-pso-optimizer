# 🛩️ OpenVSP Wing Optimization with PSO  
**Aerodynamic Optimization Pipeline • Python + OpenVSP + VSPAERO**

This repository contains a complete and automated framework for **aerodynamic optimization of a Cessna 210 wing**, integrating:

- **Python 3.10+**
- **OpenVSP 3.45.2+** (geometry engine)
- **VSPAERO** (aerodynamic solver)
- **Particle Swarm Optimization (PSO)** algorithm

The goal is to **maximize the lift-to-drag ratio (L/D)** by adjusting key geometric variables of the wing through iterative simulations.

---

## 📘 Project Overview

This framework was developed as part of an Aerospace Engineering graduation project.  
The system automates the entire aerodynamic workflow:

1. Modify wing geometry parameters in OpenVSP (AR, span, taper, sweep, twist)  
2. Regenerate the *degenerate geometry*  
3. Run a full VSPAERO aerodynamic simulation  
4. Extract CL, CDi, CD₀, L/D from `.history` files  
5. Automatically adjust the angle of attack to satisfy **L = W**  
6. Evaluate aerodynamic performance  
7. Use **PSO** to explore the design space and converge to an optimal geometry  

---

## 🚀 Key Features

### 🧬 Geometry Automation
- Programmatically updates OpenVSP wing parameters  
- Computes chord distribution (root, tip) from AR/taper  
- Automatically saves updated `.vsp3` files

### 🌪️ Aerodynamic Solver Integration
- Generates degenerate geometry (`VSPAEROComputeGeometry`)  
- Runs VSPAEROSweep with fixed Mach and single-alpha slices  
- Extracts CL, CDi, CDtot, L/D from `.history`

### 🔁 Auto-Alpha Loop
Ensures **lift equals weight** using a physics-based linear model:

```
CL = CL0 + CL_alpha * alpha
```

Implemented with:
- initial analytical alpha guess  
- iterative refinement  
- max 4 solver calls  
- ±10° bounds for stability  

### 🐦 Particle Swarm Optimization (PSO)
- Population-based search  
- Custom inertia + cognitive/social terms  
- Moving average convergence detection  
- Saves:
  - convergence of objective
  - evolution of each variable
  - best L/D per iteration  

### 📊 Output & Post-Processing
Generated automatically:
- scatter evolution plots (`dispersao_*`)
- convergence plots for L/D and fobj
- `resultado_final.txt`
- optimal geometry file: `cessna_best.vsp3`

---

## 🧠 Optimization Problem

### 🎯 Objective  
Maximize aerodynamic efficiency:

```
max ( L / D )
```

### 📌 Design Variables  
- Aspect Ratio (AR)  
- Span  
- Taper ratio  
- Sweep  
- Twist  

### 🧷 Constraints  
- Fixed cruise Mach number (M = 0.30)  
- Fixed weight  
- Auto-alpha enforces **L ≈ W**  
- Geometry bounds defined per variable  

---

## 📁 Repository Structure

```
/v15_cessna_opt.py      → Full OpenVSP + VSPAERO simulation function (FCN)
/v15_cessna_pso.py      → PSO optimizer integrated with FCN
/cessna210.vsp3         → Base geometry model
/resultados_variaveis/  → Saved runs, logs, plots, optimal geometries
```

---

## 🛠️ Installation

### 1. Install OpenVSP  
Download from: https://openvsp.org/download.html

### 2. Create a Python environment

```bash
conda create -n spyvsp python=3.10
conda activate spyvsp
pip install numpy matplotlib
```

### 3. Run any optimization or simulation test

```bash
python v15_cessna_pso.py
```

---

## 📊 Example Outputs

- Best L/D per iteration  
- Scatter plots of particle distribution per variable  
- Geometry exported automatically for analysis  
- Fully logged aerodynamic performance (CL, CDi, CD₀, L/D)  

---

## 📚 Academic Context

This project was developed as part of my **Final Course Project (PFC)** in Aerospace Engineering.  
It demonstrates:

- Integration of parametric geometry tools with numerical solvers  
- Automated CFD-like analysis for conceptual aircraft design  
- Application of nature-inspired optimization in aeronautical engineering  

---

## 📜 Full Technical Tutorial

A detailed appendix (LaTeX) explaining the entire integration pipeline  
(Python → OpenVSP → VSPAERO) is included in the academic document.

---

## 🤝 Contributions

Suggestions, improvements, and academic collaborations are welcome!  
Feel free to open an issue or submit a pull request.

---

## 🔗 Repository

**GitHub:**  
https://github.com/Ggregori3112/openvsp-pso-optimizer

---

## 📬 Contact

If you want help reproducing or extending this workflow, feel free to reach out.

**LinkedIn:**  
www.linkedin.com/in/gregorimaia3112
