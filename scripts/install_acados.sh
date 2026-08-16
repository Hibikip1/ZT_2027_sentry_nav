#!/usr/bin/env bash
# =============================================================================
# acados 安装脚本(hero_mpc_controller 的 MPC 求解器依赖)
#
# 默认安装到 <workspace>/src/third_party/acados(hero_mpc_controller/CMakeLists.txt
# 的默认查找位置)。也可通过环境变量 ACADOS_DIR 指定安装位置,
# 此时需在编译时设置 ACADOS_DIR 环境变量(CMake 优先读取)。
#
# 用法:
#   source /opt/ros/humble/setup.bash
#   bash scripts/install_acados.sh            # 安装到 $PWD/src/third_party/acados
#   ACADOS_DIR=/path/to/acados bash scripts/install_acados.sh
#
# 安装后重编译:
#   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# =============================================================================
set -euo pipefail

# ---- 1. 依赖 ---------------------------------------------------------------
echo "[acados] 安装构建依赖(需要 sudo)..."
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libblas-dev liblapack-dev libopenblas-dev libeigen3-dev \
    libgfortran5

# ---- 2. 安装路径 -----------------------------------------------------------
DEFAULT_ACADOS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/third_party/acados"
ACADOS_DIR="${ACADOS_DIR:-${DEFAULT_ACADOS_DIR}}"
ACADOS_VERSION="${ACADOS_VERSION:-v0.4.2}"
BUILD_DIR="$(mktemp -d)"
echo "[acados] 安装到: ${ACADOS_DIR} (版本 ${ACADOS_VERSION})"

# ---- 3. 下载源码 -----------------------------------------------------------
if [ -d "${ACADOS_DIR}/include/acados_c" ]; then
    echo "[acados] 已存在 ${ACADOS_DIR},跳过下载"
else
    echo "[acados] 克隆 acados 源码..."
    git clone --depth 1 --branch "${ACADOS_VERSION}" --recurse-submodules \
        https://github.com/acados/acados.git "${BUILD_DIR}/acados"
    mkdir -p "${ACADOS_DIR}"
    # 拷贝源码树(保留 git 以便后续更新)
    cp -r "${BUILD_DIR}/acados/." "${ACADOS_DIR}/"
fi

# ---- 4. 构建 ---------------------------------------------------------------
mkdir -p "${ACADOS_DIR}/build"
cd "${ACADOS_DIR}/build"
echo "[acados] CMake 配置..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DACADOS_WITH_QPOASES=OFF \
    -DACADOS_WITH_OSQP=OFF \
    -DACADOS_INSTALL_DIR="${ACADOS_DIR}" \
    -DCMAKE_INSTALL_PREFIX="${ACADOS_DIR}"

echo "[acados] 构建(并行 $(nproc))..."
make -j"$(nproc)"
make install

# ---- 5. 验证 ---------------------------------------------------------------
if [ -f "${ACADOS_DIR}/lib/libacados.so" ] && [ -d "${ACADOS_DIR}/include/acados_c" ]; then
    echo ""
    echo "[acados] ✅ 安装成功: ${ACADOS_DIR}"
    echo "[acados] 重新编译 hero_mpc_controller:"
    echo "    source /opt/ros/humble/setup.bash"
    echo "    colcon build --symlink-install --packages-select hero_mpc_controller --cmake-args -DCMAKE_BUILD_TYPE=Release"
    echo "    (若 CMake 仍找不到,请 export ACADOS_DIR=${ACADOS_DIR} 后重编)"
else
    echo "[acados] ❌ 安装失败,请检查上方日志"
    exit 1
fi
