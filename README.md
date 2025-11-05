# PFC
Optimization scripts for a Cessna 210 wing using OpenVSP and PSO algorithms.

# 🛩️ PSO_PYTHON_WING

Optimization scripts for a Cessna 210 wing using **Python**, **OpenVSP**, and **VSPAERO**.  
The main goal is to maximize the **lift-to-drag ratio (L/D)** through automated geometry and aerodynamic simulations.

---

## 📘 Overview

This project automates the aerodynamic optimization of a small regional aircraft wing (Cessna 210 model).  
It integrates OpenVSP’s API with a Python-based **Particle Swarm Optimization (PSO)** algorithm to evaluate and refine the wing geometry.

---

## ⚙️ Features

- Automatic OpenVSP geometry modification (sweep, twist, taper, span)
- Batch aerodynamic simulations with VSPAERO
- L/D computation and convergence tracking
- Support for `.vsp3`, `.history`, and `.polar` output files
- Optimizer implemented with custom PSO and moving average convergence

---

## 🧠 Objectives

- **Maximize:** aerodynamic efficiency (L/D ratio)  
- **Fixed parameters:** Mach number, airfoil type, and cruise conditions  
- **Tools:** OpenVSP 3.45.2, VSPAERO solver, Python 3.10+



