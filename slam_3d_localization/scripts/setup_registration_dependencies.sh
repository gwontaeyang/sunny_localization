#!/usr/bin/env bash
# Build only the pinned optional matcher dependency; never modify a user checkout.
set -euo pipefail

readonly FAST_GICP_COMMIT=0e7ec1441c99f7be453db2ea216d5de029387417
readonly FAST_GICP_URL=https://github.com/koide3/fast_gicp.git

usage() {
  printf '%s\n' \
    'Usage: bash setup_registration_dependencies.sh --workspace DIR --output-dir DIR [options]' \
    '' \
    '  --workspace DIR           Existing ROS workspace; dependency lives in deps/fast_gicp' \
    '  --output-dir DIR          Required analysis output directory for build, logs and manifest' \
    '  --cuda-architectures LIST Required for CUDA, e.g. 87 or 90-virtual (semicolon list allowed)' \
    '  --cuda-root DIR           CUDA toolkit root (default: /usr/local/cuda)' \
    '  --cpu-only                Build FastGICP/FastVGICP without CUDA' \
    '  --jobs N                  Build parallelism (default: 2)' \
    '  -h, --help                Show this help without changing anything' \
    '' \
    'Requires existing git, cmake, colcon, ROS 2 Humble, Eigen/PCL, and nvcc for CUDA.' \
    'No sudo, package installation, localization build, robot execution, or git push.' \
    'Existing dependency source is reused only when its pinned revision and patch match exactly.'
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
value_required() {
  [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "$1 requires a value"
}

workspace=''
output_dir=''
cuda_root=/usr/local/cuda
architectures=''
cuda=ON
jobs=2
while (($#)); do
  case "$1" in
    --workspace) value_required "$@"; workspace=$2; shift 2 ;;
    --output-dir) value_required "$@"; output_dir=$2; shift 2 ;;
    --cuda-root) value_required "$@"; cuda_root=$2; shift 2 ;;
    --cuda-architectures) value_required "$@"; architectures=$2; shift 2 ;;
    --jobs) value_required "$@"; jobs=$2; shift 2 ;;
    --cpu-only) cuda=OFF; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done
[[ -n "$workspace" ]] || fail '--workspace is required'
[[ -n "$output_dir" ]] || fail '--output-dir is required'
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || fail '--jobs must be a positive integer'
if [[ "$cuda" == ON ]]; then
  [[ -n "$architectures" ]] || fail '--cuda-architectures is required unless --cpu-only is used'
  [[ "$architectures" =~ ^[0-9]+(-real|-virtual)?(\;[0-9]+(-real|-virtual)?)*$ ]] ||
    fail '--cuda-architectures must contain numeric entries, optionally -real/-virtual, separated by semicolons'
else
  [[ -z "$architectures" ]] || fail '--cuda-architectures cannot be combined with --cpu-only'
fi

for tool in git cmake colcon sha256sum realpath mktemp tee; do
  command -v "$tool" >/dev/null 2>&1 || fail "required tool is missing: $tool (install separately)"
done
colcon_command=$(command -v colcon)
if [[ "$cuda" == ON ]]; then
  nvcc="$cuda_root/bin/nvcc"
  [[ -x "$nvcc" ]] || fail "nvcc is missing or not executable: $nvcc"
  supported_architectures=$("$nvcc" --list-gpu-arch) || fail 'nvcc --list-gpu-arch failed'
  IFS=';' read -r -a requested_architectures <<< "$architectures"
  for architecture in "${requested_architectures[@]}"; do
    number=${architecture%%-*}
    supported=false
    for available in $supported_architectures; do
      if [[ "$available" == "compute_$number" ]]; then supported=true; break; fi
    done
    [[ "$supported" == true ]] || fail "unsupported CUDA architecture $architecture for $nvcc"
  done
fi
[[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash'
[[ -d "$workspace" ]] || fail "workspace does not exist: $workspace"
workspace=$(realpath "$workspace")
[[ "$workspace" != / ]] || fail 'workspace must not be the filesystem root'
output_dir=$(realpath -m "$output_dir")
dependency_root="$workspace/deps/fast_gicp"
source_dir="$dependency_root/src"
install_dir="$dependency_root/install"
for path in "$workspace/deps" "$dependency_root" "$install_dir"; do
  [[ ! -L "$path" ]] || fail "dependency paths must not redirect through symlinks: $path"
done
[[ "$output_dir" != "$workspace" && "$output_dir" != / ]] || fail 'use a dedicated analysis output directory'
[[ "$output_dir" != "$workspace/deps" && "$output_dir" != "$workspace/deps/"* ]] ||
  fail '--output-dir must be outside the operational deps directory'
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
patch_file="$script_dir/../patches/fast_gicp_0e7ec144_cuda12.patch"
[[ -f "$patch_file" ]] || fail "pinned compatibility patch is missing: $patch_file"
patch_file=$(realpath "$patch_file")
mkdir -p -- "$output_dir"
exec > >(tee -a "$output_dir/setup.log") 2>&1
trap 'code=$?; if ((code)); then printf "Setup failed (exit %s); see %s/setup.log\n" "$code" "$output_dir" >&2; fi' EXIT
printf 'Dependency source: %s\nDependency install: %s\nEvidence: %s\n' \
  "$source_dir" "$install_dir" "$output_dir"

# Do not inherit a caller's redirected Git index, work tree or object database.
unset GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE GIT_OBJECT_DIRECTORY GIT_ALTERNATE_OBJECT_DIRECTORIES
export GIT_OPTIONAL_LOCKS=0
mkdir -p -- "$workspace/deps"
if [[ ! -e "$workspace/deps/COLCON_IGNORE" ]]; then
  touch -- "$workspace/deps/COLCON_IGNORE"
fi
[[ -f "$workspace/deps/COLCON_IGNORE" ]] || fail 'deps/COLCON_IGNORE is not a regular file'
mkdir -p -- "$dependency_root"
if [[ ! -e "$source_dir" && ! -L "$source_dir" ]]; then
  git clone --no-checkout -- "$FAST_GICP_URL" "$source_dir"
  git -C "$source_dir" checkout --detach "$FAST_GICP_COMMIT"
  git -C "$source_dir" submodule update --init --recursive -- thirdparty/Eigen thirdparty/nvbio
  git -C "$source_dir" apply --unidiff-zero --check "$patch_file"
  git -C "$source_dir" apply --unidiff-zero "$patch_file"
else
  printf 'Existing source found; validating without checkout, reset, patching or submodule updates.\n'
fi

[[ ! -L "$source_dir" && -d "$source_dir/.git" ]] || fail "source must be a dedicated clone, not a symlink/worktree: $source_dir"
[[ "$(git -C "$source_dir" rev-parse --show-toplevel)" == "$source_dir" ]] || fail 'source is not its own git root'
[[ "$(git -C "$source_dir" rev-parse HEAD)" == "$FAST_GICP_COMMIT" ]] || fail 'existing source HEAD does not match the pinned commit; no changes made to it'
git -C "$source_dir" diff --cached --quiet HEAD -- || fail 'source has staged changes; refusing to overwrite them'
[[ -z "$(git -C "$source_dir" ls-files --others)" ]] || fail 'source contains untracked files (including ignored files); refusing reuse'

# Derive the exact expected patched tree in isolated index/objects. Even blob
# creation stays out of the existing clone's .git directory.
verification_dir=$(mktemp -d "$output_dir/dependency_verification.XXXXXX")
mkdir -- "$verification_dir/objects"
original_objects=$(git -C "$source_dir" rev-parse --path-format=absolute --git-path objects)
expected_git() {
  GIT_INDEX_FILE="$verification_dir/index" \
  GIT_OBJECT_DIRECTORY="$verification_dir/objects" \
  GIT_ALTERNATE_OBJECT_DIRECTORIES="$original_objects" git -C "$source_dir" "$@"
}
expected_git read-tree "$FAST_GICP_COMMIT"
expected_git apply --cached --unidiff-zero "$patch_file"
expected_git diff --quiet --ignore-submodules=none -- || fail 'source differs from exactly the pinned commit plus compatibility patch'
for submodule in thirdparty/Eigen thirdparty/nvbio; do
  expected_commit=$(git -C "$source_dir" rev-parse "$FAST_GICP_COMMIT:$submodule")
  [[ -e "$source_dir/$submodule/.git" ]] || fail "required submodule is not initialized: $submodule"
  [[ "$(git -C "$source_dir/$submodule" rev-parse HEAD)" == "$expected_commit" ]] || fail "submodule revision mismatch: $submodule"
  [[ -z "$(git -C "$source_dir/$submodule" status --porcelain=v1 --untracked-files=all --ignore-submodules=none)" ]] ||
    fail "submodule has local changes: $submodule"
  [[ -z "$(git -C "$source_dir/$submodule" ls-files --others)" ]] || fail "submodule contains untracked files: $submodule"
done
submodule_status=$(git -C "$source_dir" submodule status --recursive -- thirdparty/Eigen thirdparty/nvbio)
while IFS= read -r status; do
  [[ "$status" == ' '* ]] || fail "submodule is missing or at the wrong revision: $status"
done <<< "$submodule_status"

{
  printf 'fast_gicp_url=%s\nfast_gicp_commit=%s\n' "$FAST_GICP_URL" "$FAST_GICP_COMMIT"
  printf 'patch=%s\n' "$patch_file"
  printf 'patch_sha256=%s\n' "$(sha256sum "$patch_file" | cut -d ' ' -f 1)"
  printf 'workspace=%s\nsource=%s\ninstall=%s\n' "$workspace" "$source_dir" "$install_dir"
  printf 'cuda=%s\ncuda_root=%s\ncuda_architectures=%s\njobs=%s\n' "$cuda" "$cuda_root" "$architectures" "$jobs"
  printf 'ros_underlay=/opt/ros/humble\n'
  git --version
  cmake --version
  uname -a
  printf '%s\n' "$submodule_status"
  if [[ "$cuda" == ON ]]; then "$nvcc" --version; printf '%s\n' "$supported_architectures"; fi
} > "$output_dir/dependency_manifest.txt"

# An empty environment avoids recording unrelated workspace underlays in the
# installed setup.bash. Keep only the normal home/path and explicit inputs.
env -i HOME="$HOME" USER="${USER:-}" LANG=C.UTF-8 \
  COLCON_DEFAULTS_FILE=/dev/null \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    source_dir=$1; output_dir=$2; install_dir=$3; cuda=$4
    cuda_root=$5; architectures=$6; jobs=$7; colcon_command=$8
    export MAKEFLAGS="-j$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
    arguments=(--log-base "$output_dir/dependency_logs" build
      --base-paths "$source_dir" --packages-select fast_gicp
      --build-base "$output_dir/dependency_build" --install-base "$install_dir"
      --executor sequential --event-handlers console_direct+
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_apps=OFF
      -DBUILD_test=OFF -DBUILD_PYTHON_BINDINGS=OFF "-DBUILD_VGICP_CUDA=$cuda")
    if [[ "$cuda" == ON ]]; then
      arguments+=("-DCUDA_TOOLKIT_ROOT_DIR=$cuda_root" "-DCMAKE_CUDA_ARCHITECTURES=$architectures")
    fi
    printf "%s\n" "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}" "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-}" \
      "COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}" "PYTHONPATH=${PYTHONPATH:-}" \
      "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}" "PATH=$PATH" > "$output_dir/build_environment.txt"
    printf "%q " "$colcon_command" "${arguments[@]}" > "$output_dir/dependency_build_command.txt"
    printf "\n" >> "$output_dir/dependency_build_command.txt"
    "$colcon_command" "${arguments[@]}" 2>&1 | tee "$output_dir/dependency_build.log"
  ' bash "$source_dir" "$output_dir" "$install_dir" "$cuda" \
  "$cuda_root" "$architectures" "$jobs" "$colcon_command"
printf 'Dependency build complete. Source %s/setup.bash before building localization.\n' "$install_dir"
