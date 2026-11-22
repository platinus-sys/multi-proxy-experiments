#!/usr/bin/env bash
set -Eeuo pipefail

# One run per mode by default; REPEATS=3 runs the nine trials required by the task.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$ROOT"
REPEATS=1
SETTLE_SECONDS=${SETTLE_SECONDS:-120}
SAVE_TIMEOUT=${SAVE_TIMEOUT:-900}
MIN_FREE_GB=${MIN_FREE_GB:-7}
KEEP_WORKING_PCD=${KEEP_WORKING_PCD:-0}
[[ $REPEATS =~ ^[1-9][0-9]*$ && $SETTLE_SECONDS =~ ^[0-9]+$ && $SAVE_TIMEOUT =~ ^[1-9][0-9]*$ && $MIN_FREE_GB =~ ^[1-9][0-9]*$ && $KEEP_WORKING_PCD =~ ^[01]$ ]] || { echo 'Invalid configuration' >&2; exit 2; }

SESSION=$(date -u +%Y%m%dT%H%M%SZ)_$$
OUT="$ROOT/results/part3/$SESSION"
STAGE_ROOT="$ROOT/src/multi_proxy/part3_work/$SESSION"
CONTAINER_ROOT="/ws_code/src/multi_proxy/part3_work/$SESSION"
MASTER=http://172.0.0.2:11311
NET="$ROOT/experiments/part2/network.sh"
bag=/datasets/S3E_College_robot_odom.bag
containers=(multi-proxy0 multi-proxy1 multi-proxy2)
pids=()
recorder_pid=''
network_active=0
sim_changed=0
old_sim=''
current_out=''

ros() {
  docker exec multi-proxy0 bash -c 'source /ws_code/devel/setup.bash; export ROS_MASTER_URI=http://172.0.0.2:11311; "$@"' _ "$@"
}
stop_nodes() {
  if [[ -n $recorder_pid ]]; then
    ros rosnode kill /part3_markers >/dev/null 2>&1 || true
    wait "$recorder_pid" 2>/dev/null || true
    recorder_pid=''
  fi
  (( ${#pids[@]} )) || return 0
  for i in 0 1 2; do ros rosnode kill "/part3_robot_$i" >/dev/null 2>&1 || true; done
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
  pids=()
}
cleanup() {
  code=$?
  trap - EXIT INT TERM
  stop_nodes
  if (( network_active )); then bash "$NET" clear || echo 'WARNING: inspect tc settings manually' >&2; fi
  if (( sim_changed )); then
    if [[ $old_sim == absent ]]; then ros rosparam delete /use_sim_time >/dev/null 2>&1 || true
    else ros rosparam set /use_sim_time "$old_sim" >/dev/null 2>&1 || true; fi
  fi
  if (( code != 0 )); then
    [[ -n $current_out ]] && printf 'FAILED exit=%s at=%s\n' "$code" "$(date -Is)" > "$current_out/FAILED.txt"
    echo "Run interrupted or failed. Results: $OUT; working files: $STAGE_ROOT" >&2
  fi
  exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for x in docker python3 sha256sum df; do command -v "$x" >/dev/null || { echo "Missing: $x" >&2; exit 1; }; done
for c in "${containers[@]}"; do docker inspect -f '{{.State.Running}}' "$c" | grep -qx true || { echo "$c is not running" >&2; exit 1; }; done
docker exec multi-proxy0 test -r "$bag"
ros rosnode list >/dev/null
for c in "${containers[@]}"; do
  qdisc=$(docker exec "$c" tc qdisc show dev eth0)
  [[ $qdisc == 'qdisc noqueue 0: root'* ]] || { echo "$c has existing tc configuration: $qdisc" >&2; exit 1; }
done
mkdir -p "$OUT" "$STAGE_ROOT"
{
  echo "session=$SESSION"; echo "started_utc=$(date -u -Is)"; echo "repeats=$REPEATS"; echo "settle_seconds=$SETTLE_SECONDS"; echo "save_timeout=$SAVE_TIMEOUT"; echo "keep_working_pcd=$KEEP_WORKING_PCD"; echo "master=$MASTER"; echo "bag=$bag"; echo 'bag_rate=1.0'
  uname -a
  docker version --format 'docker_server={{.Server.Version}}'
  for c in "${containers[@]}"; do docker inspect -f '{{.Name}} image_id={{.Image}}' "$c"; done
  git -C src/multi_proxy rev-parse HEAD
  git -C src/multi_proxy status --short
  sha256sum datasets/S3E/S3E_College_robot_odom.bag
} > "$OUT/environment.txt"
cp src/multi_proxy/config/submapfusion_s3e.yaml "$OUT/config.yaml"
cp src/multi_proxy/launch/part3_isolated.launch "$OUT/launch.xml"
cp "$NET" "$OUT/network.sh"
ros rosbag info "$bag" > "$OUT/bag_info.txt"
git -C src/multi_proxy diff -- . ':!log' > "$OUT/source_changes.diff" || true

old_sim=$(ros rosparam get /use_sim_time 2>/dev/null || echo absent)
ros rosparam set /use_sim_time false
sim_changed=1
# The existing three mapping nodes must stop so every run starts with empty state.
for i in 0 1 2; do ros rosnode kill "/multi_proxy_subMapFusion_robot_$i" >/dev/null 2>&1 || true; done
for ((t=0; t<30; t++)); do
  old_nodes=$(ros rosnode list 2>/dev/null || true)
  [[ $old_nodes != *'/multi_proxy_subMapFusion_robot_'* ]] && break
  sleep 1
done
[[ $old_nodes != *'/multi_proxy_subMapFusion_robot_'* ]] || { echo 'Old robot nodes did not stop' >&2; exit 1; }

modes=(isolated)
for mode in "${modes[@]}"; do
  for ((rep=1; rep<=REPEATS; rep++)); do
    run="${mode}_$(printf '%02d' "$rep")"
    current_out="$OUT/$mode/run_$rep"
    stage="$STAGE_ROOT/$run"
    container_stage="$CONTAINER_ROOT/$run"
    free_kb=$(df -Pk "$ROOT" | awk 'NR==2 {print $4}')
    (( free_kb >= MIN_FREE_GB * 1024 * 1024 )) || { echo "Not enough free disk before $run: ${free_kb} KiB" >&2; exit 1; }
    mkdir -p "$current_out" "$stage/log/bw_compare" "$stage/log/pcm_matrix" "$stage/pcd/robot_0" "$stage/pcd/robot_1" "$stage/pcd/robot_2"
    printf 'mode=%s\nrepeat=%s\nstart_utc=%s\nsettle_seconds=%s\n' "$mode" "$rep" "$(date -u -Is)" "$SETTLE_SECONDS" > "$current_out/metadata.txt"
    echo "Starting $run"
    bash "$NET" apply baseline
    network_active=1
    bash "$NET" status > "$current_out/tc_before.txt"
    for i in 0 1 2; do
      docker exec "${containers[$i]}" bash -c 'source /ws_code/devel/setup.bash; export ROS_MASTER_URI=http://172.0.0.2:11311; exec roslaunch multi_proxy part3_isolated.launch id:="$1" output:="$2"' _ "$i" "$container_stage" > "$current_out/robot_$i.console.log" 2>&1 &
      pids+=("$!")
    done
    ready=0
    for ((t=0; t<90; t++)); do
      services=$(ros rosservice list 2>/dev/null || true)
      nodes=$(ros rosnode list 2>/dev/null || true)
      if [[ $services == *'/robot_0/multi_proxy/save_traj'* && $services == *'/robot_1/multi_proxy/save_traj'* && $services == *'/robot_2/multi_proxy/save_traj'* && $nodes == *'/part3_robot_0'* && $nodes == *'/part3_robot_1'* && $nodes == *'/part3_robot_2'* ]]; then ready=1; break; fi
      for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || { echo 'Robot launch exited' >&2; exit 1; }; done
      sleep 1
    done
    (( ready )) || { echo 'Timed out waiting for robot services' >&2; exit 1; }
    docker exec multi-proxy0 bash -c 'source /ws_code/devel/setup.bash; export ROS_MASTER_URI=http://172.0.0.2:11311; exec rosbag record -O "$1" __name:=part3_markers /robot_0/loop_closure_constraints /robot_1/loop_closure_constraints /robot_2/loop_closure_constraints /robot_0/key_frame /robot_1/key_frame /robot_2/key_frame' _ "$container_stage/markers.bag" > "$current_out/markers_record.log" 2>&1 &
    recorder_pid=$!
    sleep 5
    kill -0 "$recorder_pid" 2>/dev/null || { echo 'Marker recorder exited' >&2; exit 1; }
    ros rosnode list > "$current_out/rosnodes.txt"
    ros rosparam dump /tmp/part3_params_"$SESSION".yaml
    docker cp "multi-proxy0:/tmp/part3_params_$SESSION.yaml" "$current_out/rosparams.yaml"
    printf 'play_start_utc=%s\nplay_start_epoch=%s\n' "$(date -u -Is)" "$(date +%s.%N)" >> "$current_out/metadata.txt"
    ros rosbag play --rate 1.0 "$bag" > "$current_out/rosbag_play.log" 2>&1
    printf 'play_end_utc=%s\nplay_end_epoch=%s\n' "$(date -u -Is)" "$(date +%s.%N)" >> "$current_out/metadata.txt"
    sleep "$SETTLE_SECONDS"
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || { echo 'Robot launch exited before saving' >&2; exit 1; }; done
    printf 'save_request_utc=%s\nsave_request_epoch=%s\n' "$(date -u -Is)" "$(date +%s.%N)" >> "$current_out/metadata.txt"
    for i in 0 1 2; do ros rosservice call "/robot_$i/multi_proxy/save_traj" 'file_path: ""' >> "$current_out/save_service.log"; done
    # Require non-empty trajectories and maps, no process holding the maps open,
    # and unchanged map sizes for 30 seconds; service itself returns immediately.
    done_save=0; stable=0; previous=''
    for ((t=0; t<SAVE_TIMEOUT; t+=5)); do
      sizes=''; files_ok=1
      for i in 0 1 2; do
        map="$stage/log/robot_${i}_pcd_wrt_global.pcd"
        traj="$stage/log/robot_${i}.tum"
        if [[ ! -s $map || ! -s $traj ]]; then files_ok=0; break; fi
        sizes+="$(stat -c %s "$map"),"
        docker exec "${containers[$i]}" python3 -c 'import glob,os,sys
target=sys.argv[1]
for fd in glob.glob("/proc/[0-9]*/fd/*"):
 try:
  if os.readlink(fd)==target: sys.exit(1)
 except OSError: pass
sys.exit(0)' "$container_stage/log/robot_${i}_pcd_wrt_global.pcd" || files_ok=0
      done
      if (( files_ok )) && [[ $sizes == "$previous" ]]; then ((stable+=5)); else stable=0; fi
      previous=$sizes
      if (( stable >= 30 )); then done_save=1; break; fi
      sleep 5
    done
    (( done_save )) || { echo 'Timed out waiting for saved maps; leaving working files in place' >&2; exit 1; }
    printf 'save_end_utc=%s\nsave_end_epoch=%s\n' "$(date -u -Is)" "$(date +%s.%N)" >> "$current_out/metadata.txt"
    bash "$NET" status > "$current_out/tc_after.txt"
    stop_nodes
    # ROS shutdown is asynchronous; wait until no robot still holds a run file.
    closed=0
    for ((t=0; t<60; t++)); do
      closed=1
      for c in "${containers[@]}"; do
        docker exec "$c" python3 -c 'import glob,os,sys
prefix=sys.argv[1]+"/"
for fd in glob.glob("/proc/[0-9]*/fd/*"):
 try:
  if os.readlink(fd).startswith(prefix): sys.exit(1)
 except OSError: pass
sys.exit(0)' "$container_stage" || closed=0
      done
      (( closed )) && break
      sleep 1
    done
    (( closed )) || { echo 'Run files still open after stopping nodes' >&2; exit 1; }
    bash "$NET" clear
    network_active=0
    cp -a "$stage/log" "$current_out/log"
    [[ -s $stage/markers.bag ]] || { echo 'Marker bag missing or empty' >&2; exit 1; }
    cp "$stage/markers.bag" "$current_out/markers.bag"
    if (( KEEP_WORKING_PCD )); then
      mv "$stage/pcd" "$current_out/working_pcd"
    else
      # This path is generated by this script, never the existing pcd directory.
      docker exec multi-proxy0 rm -rf -- "$container_stage/pcd"
    fi
    diff -qr "$stage/log" "$current_out/log" >/dev/null
    cmp "$stage/markers.bag" "$current_out/markers.bag"
    docker exec multi-proxy0 rm -rf -- "$container_stage"
    printf 'completed_utc=%s\n' "$(date -u -Is)" >> "$current_out/metadata.txt"
    echo complete > "$current_out/COMPLETE.txt"
    current_out=''
  done
done

echo "Results: $OUT"
