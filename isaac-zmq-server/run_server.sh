xhost +local:appuser

# Detect host HiDPI scale and forward into the container (override via ISAAC_ZMQ_DPI_SCALE)
if [ -z "${ISAAC_ZMQ_DPI_SCALE}" ] && [ -f "${HOME}/.config/monitors.xml" ]; then
    ISAAC_ZMQ_DPI_SCALE=$(grep -m1 -oE '<scale>[^<]+' "${HOME}/.config/monitors.xml" 2>/dev/null | head -n1 | sed 's/<scale>//')
fi
if [ -z "${ISAAC_ZMQ_DPI_SCALE}" ] && command -v xrdb >/dev/null 2>&1; then
    XFT_DPI=$(xrdb -query 2>/dev/null | awk '/^Xft\.dpi:/ {print $2; exit}')
    if [ -n "${XFT_DPI}" ]; then
        ISAAC_ZMQ_DPI_SCALE=$(awk "BEGIN { printf \"%.3f\", ${XFT_DPI}/96.0 }")
    fi
fi
ISAAC_ZMQ_DPI_SCALE="${ISAAC_ZMQ_DPI_SCALE:-1.0}"
echo "Using ISAAC_ZMQ_DPI_SCALE=${ISAAC_ZMQ_DPI_SCALE}"

docker run --gpus all --network host \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -e XAUTHORITY=$XAUTHORITY \
       -v $XAUTHORITY:$XAUTHORITY \
       -e ISAAC_ZMQ_DPI_SCALE="${ISAAC_ZMQ_DPI_SCALE}" \
       -e ISAAC_ZMQ_UI_SCALE="${ISAAC_ZMQ_UI_SCALE}" \
       -v ./src:/isaac-zmq-server/src \
       --device /dev/input \
       --device /dev/input/event21 \
       --device /dev/input/event22 \
       --device /dev/input/event23 \
       --privileged \
       -it --rm \
        isaac-zmq-server bash
