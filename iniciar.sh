#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x target/release/ztrak-ojo ]]; then
  export PATH="$HOME/.cargo/bin:$PATH"
  cargo build --release --locked
fi
exec ./target/release/ztrak-ojo "$@"
