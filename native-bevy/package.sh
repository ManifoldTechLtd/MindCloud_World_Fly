#!/usr/bin/env bash
# Package MindCloud World Fly into a portable Linux folder + tarball.
# The TARGET machine needs NO Rust/cargo/build tools — only a Vulkan-capable GPU
# driver and a few common shared libraries (listed in the generated README.txt).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

APP="mindcloud-fly-bevy"
NAME="MindCloudFly"
ARCH="$(uname -m)"
OUT="dist/$NAME"
TARBALL="dist/${NAME}-linux-${ARCH}.tar.gz"

echo "==> Building release binary (first build can take several minutes)…"
cargo build --release

echo "==> Assembling $OUT/"
rm -rf "$OUT"
mkdir -p "$OUT/scene"
cp "target/release/$APP" "$OUT/$APP"

# Strip debug symbols to shrink the binary (optional; skipped if strip is missing).
if command -v strip >/dev/null 2>&1; then
    strip "$OUT/$APP" || true
fi

cp -r assets "$OUT/assets"
# Scenes live in scene/, not assets/ — drop any sample .ply bundled under assets/ to stay lean.
find "$OUT/assets" -name '*.ply' -delete 2>/dev/null || true

# Ship saved settings: the app reads/writes `config/` right next to `assets/`, so bundling it makes
# the package pre-configured (tuned drone, controller calibration, scene setups). Best-effort — the
# app creates `config/` at runtime if it's absent here.
if [ -d config ]; then
    cp -r config "$OUT/config"
fi

# RC-transmitter udev helper (repo root): target machines need it or Connect fails with
# "permission denied" on /dev/hidraw* (regular users can't open HID nodes by default).
if [ -f ../setup_udev.sh ]; then
    cp ../setup_udev.sh "$OUT/setup_udev.sh"
fi

# Launcher: cd into the app directory first so assets/ and scene/ resolve correctly.
cat > "$OUT/run.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
exec ./mindcloud-fly-bevy "$@"
EOF
chmod +x "$OUT/run.sh"

# Keep the (otherwise empty) scene folder in the archive + tell the user what goes there.
cat > "$OUT/scene/README.txt" <<'EOF'
把高斯泼溅场景文件 (*.ply) 放进这个文件夹。
启动后在菜单里选择,或直接:  ./run.sh -i scene/你的场景.ply
EOF

# End-user instructions (shipped inside the package).
cat > "$OUT/README.txt" <<'EOF'
MindCloud World Fly — 便携版 (Linux)
=====================================

运行 (不需要安装 Rust 或任何编译环境):
    ./run.sh                          # 打开菜单
    ./run.sh -i scene/你的场景.ply     # 直接进入某个场景
    ./run.sh --split                  # 双人分屏
    ./run.sh --world-up zup|colmap    # 指定场景的朝上轴

目标电脑必备条件:
  1) 支持 Vulkan 的显卡 + 驱动
       - NVIDIA:安装官方专有驱动
       - AMD / Intel:安装 Mesa (mesa-vulkan-drivers)
  2) 几个常见系统库。Debian/Ubuntu 上一次性安装:
       sudo apt install libvulkan1 mesa-vulkan-drivers \
            libasound2 libudev1 \
            libx11-6 libxcursor1 libxi6 libxrandr2 libxkbcommon0
     检查 Vulkan 是否就绪:  vulkaninfo | head   (来自 vulkan-tools 包)

场景文件:放进 scene/ 文件夹 (*.ply)。
设置与存档:保存在本程序目录的 config/ 文件夹 (与 assets/ 同级)，随包携带。

遥控器 (可选):
  直接点击 Connect 报 "permission denied"? 先运行一次 (需要 sudo):
      sudo bash setup_udev.sh
  它会写入 udev 规则、把你加入 plugdev 组并刷新设备权限。
  之后重新插拔遥控器即可 (若仍在旧会话,请先注销重登一次再启动程序)。

注意:本可执行文件要求目标系统的 glibc 版本不低于构建这台机器。
若启动时提示 "GLIBC_x.xx not found",请在更旧的发行版 (或旧容器) 上重新打包。
EOF

echo "==> Creating $TARBALL"
mkdir -p dist
tar czf "$TARBALL" -C dist "$NAME"

echo
echo "Done."
echo "  Folder : $HERE/$OUT   ($(du -sh "$OUT" | cut -f1))"
echo "  Tarball: $HERE/$TARBALL   ($(du -h "$TARBALL" | cut -f1))"
echo
echo "在目标机上:  tar xzf $(basename "$TARBALL") && cd $NAME && ./run.sh"
