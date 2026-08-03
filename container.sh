#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_DIR}"

SERVICE_NAME="jinsoo_docker"
CONTAINER_NAME="jinsoo_pc"
XAUTH_FILE="${PROJECT_DIR}/.docker.xauth"

prepare_x11() {
    if [ -z "${DISPLAY:-}" ]; then
        echo "오류: DISPLAY가 설정되어 있지 않습니다."
        exit 1
    fi

    if command -v xhost >/dev/null 2>&1; then
        xhost +local: >/dev/null
    fi

    export DISPLAY
}

show_help() {
    cat <<HELP
사용법:
  ./container.sh build      Docker 이미지 빌드
  ./container.sh rebuild    캐시 없이 이미지 재빌드
  ./container.sh start      컨테이너 시작
  ./container.sh enter      컨테이너 접속
  ./container.sh stop       컨테이너 정지
  ./container.sh restart    컨테이너 재시작
  ./container.sh down       컨테이너 제거
  ./container.sh logs       컨테이너 로그 확인
  ./container.sh status     컨테이너 상태 확인
HELP
}

COMMAND="${1:-help}"

case "${COMMAND}" in
    build)
        docker compose build "${SERVICE_NAME}"
        ;;

    rebuild)
        docker compose build --no-cache "${SERVICE_NAME}"
        ;;

    start)
        prepare_x11
        docker compose up -d "${SERVICE_NAME}"
        docker compose ps
        ;;

    enter)
        prepare_x11
        docker compose exec \
            -e DISPLAY="${DISPLAY}" \
            "${SERVICE_NAME}" bash
        ;;

    stop)
        docker compose stop "${SERVICE_NAME}"
        ;;

    restart)
        prepare_x11
        docker compose up -d --force-recreate "${SERVICE_NAME}"
        docker compose ps
        ;;

    down)
        docker compose down
        ;;

    logs)
        docker compose logs -f "${SERVICE_NAME}"
        ;;

    status)
        docker compose ps
        docker ps --filter "name=${CONTAINER_NAME}"
        ;;

    help|-h|--help)
        show_help
        ;;

    *)
        echo "알 수 없는 명령: ${COMMAND}"
        show_help
        exit 1
        ;;
esac
