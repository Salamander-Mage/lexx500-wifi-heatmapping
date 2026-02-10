#!/usr/bin/env bash
set -euo pipefail

# Offline pack builder for lexx500-wifi-heatmapping.
# Produces an offline_bundle/ directory with:
#   - wheelhouse/ (pip wheels for requirements.txt)
#   - debs/ (downloaded .deb packages for SNMP tools)
#   - image.tar (optional Docker image build + save)
#   - repo.tar.gz (source archive without the offline bundle itself)

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUNDLE="$ROOT/offline_bundle"
WHEELHOUSE="$BUNDLE/wheelhouse"
DEB_DIR="$BUNDLE/debs"
REPO_ARCHIVE="$BUNDLE/repo.tar.gz"
IMAGE_TAG="lexx500-wifi:offline"
IMAGE_TAR="$BUNDLE/lexx500-wifi-offline.tar"
PYTHON_BIN="python3"
DOCKER_BUILD=1
BUILD_WHEELS=1
DOWNLOAD_DEBS=1
DEB_PKGS=(snmp)

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --image-tag TAG      Docker tag to build/save (default: $IMAGE_TAG)
  --python BIN        Python executable for wheel build (default: $PYTHON_BIN)
  --no-docker         Skip Docker build/save
  --no-wheels         Skip building wheelhouse
  --no-debs           Skip downloading .deb packages
  --deb pkga,pkgb     Comma-separated list of deb packages to fetch (default: snmp)
  -h, --help          Show this help

Artifacts are written to: $BUNDLE
EOF
}

log() { echo "[offline-pack] $*"; }

die() { echo "[offline-pack] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image-tag) IMAGE_TAG="$2"; shift 2 ;;
    --python) PYTHON_BIN="$2"; shift 2 ;;
    --no-docker) DOCKER_BUILD=0; shift ;;
    --no-wheels) BUILD_WHEELS=0; shift ;;
    --no-debs) DOWNLOAD_DEBS=0; shift ;;
    --deb) IFS=',' read -r -a DEB_PKGS <<< "$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

mkdir -p "$BUNDLE"

log "Bundle dir: $BUNDLE"

if [[ $BUILD_WHEELS -eq 1 ]]; then
  command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "Python not found: $PYTHON_BIN"
  log "Building wheels with $PYTHON_BIN ..."
  mkdir -p "$WHEELHOUSE"
  "$PYTHON_BIN" -m pip wheel -r "$ROOT/requirements.txt" -w "$WHEELHOUSE"
  log "Wheelhouse ready: $WHEELHOUSE"
fi

if [[ $DOWNLOAD_DEBS -eq 1 ]]; then
  if command -v apt-get >/dev/null 2>&1 && command -v apt-cache >/dev/null 2>&1; then
    mkdir -p "$DEB_DIR"
    log "Refreshing apt metadata before downloading debs ..."
    sudo apt-get update -y >/dev/null 2>&1 || log "apt-get update failed (continuing to attempt downloads)."
    for pkg in "${DEB_PKGS[@]}"; do
      log "Downloading .deb for $pkg ..."
      (cd "$DEB_DIR" && apt-get download "$pkg") || die "Failed to download $pkg"
    done
    log "Debs stored in: $DEB_DIR"
  else
    log "Skipping deb download (apt-get not available on this host)."
  fi
fi

if [[ $DOCKER_BUILD -eq 1 ]]; then
  command -v docker >/dev/null 2>&1 || die "docker not found"
  log "Building Docker image: $IMAGE_TAG"
  docker build -f "$ROOT/Dockerfile.noetic" -t "$IMAGE_TAG" "$ROOT"
  log "Saving image to: $IMAGE_TAR"
  docker save "$IMAGE_TAG" -o "$IMAGE_TAR"
fi

log "Archiving repository (excluding offline_bundle) ..."
if command -v git >/dev/null 2>&1 && git -C "$ROOT" rev-parse >/dev/null 2>&1; then
  git -C "$ROOT" archive --format=tar.gz -o "$REPO_ARCHIVE" HEAD
else
  tar --exclude="offline_bundle" -czf "$REPO_ARCHIVE" -C "$ROOT" .
fi
log "Repo archive: $REPO_ARCHIVE"

log "Done. Bundle contents:"
find "$BUNDLE" -maxdepth 2 -type f | sed "s|$ROOT/||"
