#!/usr/bin/env python3
"""用 acados v0.5.0 重新生成 hero_mpc_controller 的求解器代码(与 HERO OCP 配置一致)

依赖: casadi + acados_template(已安装); ACADOS_INSTALL_DIR 指向 v0.5.0 安装
"""
import os
import sys

import casadi as ca
import numpy as np

# HERO 模型定义
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'hero_mpc_controller', 'model'))
from omnidirectional_model import export_omnidirectional_dynamic_model  # noqa: E402

from acados_template import AcadosOcp, AcadosOcpSolver  # noqa: E402

model = export_omnidirectional_dynamic_model()

ocp = AcadosOcp()
ocp.model = model
nx, nu = 6, 3

# 预测时域(与 mpc_wrapper.cpp 一致)
ocp.solver_options.N_horizon = 40
ocp.solver_options.tf = 2.0

# 代价: 非线性最小二乘, y = [x; u]
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

Q = np.diag([400.0, 100.0, 20.0, 10.0, 10.0, 5.0])
R = np.diag([1.0, 1.0, 0.5])
W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
ocp.cost.W = W
ocp.cost.yref = np.zeros(nx + nu)

# 终端代价
ocp.cost.cost_type_e = 'NONLINEAR_LS'
ocp.model.cost_y_expr_e = model.x
ocp.cost.W_e = np.diag([200.0, 200.0, 50.0, 20.0, 20.0, 10.0])
ocp.cost.yref_e = np.zeros(nx)

# 控制约束(加速度)
a_max, alpha_max = 2.0, 1.0
ocp.constraints.lbu = np.array([-a_max, -a_max, -alpha_max])
ocp.constraints.ubu = np.array([a_max, a_max, alpha_max])
ocp.constraints.idxbu = np.array([0, 1, 2])

# 状态约束(速度, 状态 3/4/5)
v_max, omega_max = 2.0, 1.5
ocp.constraints.lbx = np.array([-v_max, -v_max, -omega_max])
ocp.constraints.ubx = np.array([v_max, v_max, omega_max])
ocp.constraints.idxbx = np.array([3, 4, 5])

ocp.constraints.x0 = np.zeros(nx)

# 求解器选项(与 HERO 生成代码一致)
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.sim_method_num_stages = 4
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.nlp_solver_max_iter = 1
ocp.solver_options.print_level = 0

# 导出目录
out_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'hero_mpc_controller', 'model', 'c_generated_code_v050')
os.makedirs(out_dir, exist_ok=True)
ocp.code_export_directory = out_dir

print(f"[GEN] 导出求解器代码到: {out_dir}")
print(f"[GEN] N={ocp.solver_options.N_horizon}, Tf={ocp.solver_options.tf}")
print(f"[GEN] qp={ocp.solver_options.qp_solver}, nlp={ocp.solver_options.nlp_solver_type}")

# 生成(不编译, 只导出 C 代码)
json_file = os.path.join(out_dir, 'acados_ocp.json')
solver = AcadosOcpSolver(ocp, json_file=json_file, build=False)
solver.code_export()
print("[GEN] 生成完成")
