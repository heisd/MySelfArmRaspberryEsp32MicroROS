#!/usr/bin/env bash
# setup_venv.sh — 一键创建虚拟环境并安装 Python 依赖
#
# 用法：
#   bash scripts/setup_venv.sh
#
# 完成后激活环境：
#   source .venv/bin/activate

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$REPO_ROOT/.venv"

echo "================================================"
echo "  MySelfArm — 虚拟环境初始化"
echo "================================================"

# 检查 Python
if ! command -v python3 &>/dev/null; then
    echo "[错误] 未找到 python3，请先安装。"
    exit 1
fi

PYTHON_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "[info] Python 版本: $PYTHON_VER"

# 创建虚拟环境（--system-site-packages 使 ROS2 系统包可用）
echo "[info] 创建虚拟环境: $VENV_DIR"
python3 -m venv --system-site-packages "$VENV_DIR"

# 激活并安装依赖
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "[info] 升级 pip..."
pip install --upgrade pip --quiet

echo "[info] 安装 requirements.txt..."
pip install -r "$REPO_ROOT/requirements.txt"

echo ""
echo "================================================"
echo "  安装完成！"
echo ""
echo "  激活环境："
echo "    source .venv/bin/activate"
echo ""
echo "  构建 ROS2 包："
echo "    colcon build --symlink-install"
echo "    source install/setup.bash"
echo "================================================"
