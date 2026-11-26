#!/usr/bin/env bash
# Compila ZTrak Ojo para Windows desde Linux y monta la carpeta portable:
#   dist/ztrak-ojo-windows/{ZTrak Ojo.cmd, LEEME.txt, programa/}  y  dist/ztrak-ojo-windows.zip
# Misma pila que ZTrak en la Arc A310: DirectML para la IA y DirectX 12 (DXC) para las ventanas.
# Las dependencias que falten se descargan en third_party/.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
X="$HOME/.xwin"
rustup target add x86_64-pc-windows-msvc >/dev/null
if [ ! -d "$X/sdk" ]; then
  command -v xwin >/dev/null || cargo install xwin
  xwin --accept-license splat --output "$X"
fi
[ -e "$X/sdk/lib/um/x86_64/PathCch.lib" ] || ln -sf pathcch.lib "$X/sdk/lib/um/x86_64/PathCch.lib"

pieza() { # carpeta local reutilizable
  echo "third_party/$1"
}
D=$(pieza directml)
if [ ! -f "$D/pkg/bin/x64-win/DirectML.dll" ]; then
  D=third_party/directml; mkdir -p $D
  curl -fsSL -o $D/dml.zip https://www.nuget.org/api/v2/package/Microsoft.AI.DirectML/1.15.4
  unzip -q -o $D/dml.zip -d $D/pkg
fi
F=$(pieza ffmpeg-windows)
if [ ! -f "$F/ffmpeg.exe" ]; then
  F=third_party/ffmpeg-windows; mkdir -p $F
  curl -fsSL -o $F/ff.zip https://github.com/BtbN/FFmpeg-Builds/releases/download/latest/ffmpeg-master-latest-win64-gpl.zip
  unzip -q -o -j $F/ff.zip '*/bin/ffmpeg.exe' -d $F
fi
R=$(pieza vcruntime)
if [ ! -f "$R/vcruntime140_1.dll" ]; then
  R=third_party/vcruntime; mkdir -p $R
  curl -fsSL --retry 5 -o $R/w.zip https://files.pythonhosted.org/packages/98/33/be9ef68901200813fc174a06758b4ae48ca4eb58fa783280dd879d5fd048/msvc_runtime-14.51.36231-cp312-cp312-win_amd64.whl
  unzip -q -o -j $R/w.zip '*.dll' -d $R
fi
C=$(pieza dxc)
if [ ! -f "$C/pkg/bin/x64/dxcompiler.dll" ]; then
  C=third_party/dxc; mkdir -p $C
  curl -fsSL --retry 5 -o $C/dxc.zip https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2609/dxc_2026_09_29.zip
  unzip -q -o $C/dxc.zip -d $C/pkg || [ $? -eq 1 ] # el zip de Microsoft usa barras invertidas
fi

export CARGO_TARGET_X86_64_PC_WINDOWS_MSVC_LINKER=rust-lld
export CARGO_TARGET_X86_64_PC_WINDOWS_MSVC_RUSTFLAGS="-Lnative=$X/crt/lib/x86_64 -Lnative=$X/sdk/lib/um/x86_64 -Lnative=$X/sdk/lib/ucrt/x86_64 -Lnative=$PWD/$D/pkg/bin/x64-win"
cargo build --release --locked --target x86_64-pc-windows-msvc --bin ztrak-ojo

O=dist/ztrak-ojo-windows
P=$O/programa
T=target/x86_64-pc-windows-msvc/release
rm -rf "$O" && mkdir -p "$P/models"
cp $T/ztrak-ojo.exe "$P/"
find $T -maxdepth 1 -name 'onnxruntime*.dll' -exec cp {} "$P/" \;
cp "$D/pkg/bin/x64-win/DirectML.dll" "$F/ffmpeg.exe" "$C/pkg/bin/x64/dxcompiler.dll" "$C/pkg/bin/x64/dxil.dll" "$P/"
cp "$R"/vcruntime140*.dll "$R"/msvcp140*.dll "$R"/concrt140.dll "$P/"
cp models/yolox_nano_humanart_416.onnx models/rtmpose-m_body7_256x192.onnx models/face_detection_yunet_2026may.onnx models/YuNet-LICENSE "$P/models/"
printf '%s\r\n' '@echo off' 'cd /d "%~dp0programa"' 'ztrak-ojo.exe %*' 'if errorlevel 1 pause' > "$O/ZTrak Ojo.cmd"
printf '%s\r\n' 'ZTrak Ojo' '' \
  'Abre "ZTrak Ojo.cmd". Copia siempre la carpeta entera.' '' \
  '1. Arriba, en "Webcam del ojo", elige su webcam y comprueba la imagen.' \
  '   La webcam de la aplicación ZTrak aparece bloqueada.' \
  '2. En la pestana Ojo elige el puerto COM del Nano (CH340), 250000 baudios, y pulsa Aplicar.' \
  '3. Prueba primero Modo calibracion en la pestana Manual.' '' \
  'Todo lo que pasa en la ultima sesion queda en ztrak-ojo-log.txt.' > "$O/LEEME.txt"
(cd dist && rm -f ztrak-ojo-windows.zip && zip -qr ztrak-ojo-windows.zip ztrak-ojo-windows)
echo "Listo: $O ($(du -sh "$O" | cut -f1)) y dist/ztrak-ojo-windows.zip"
