#!/usr/bin/env bash
# Apply netem only to traffic addressed to the other robot containers.
set -euo pipefail
action=${1:-}
mode=${2:-}
robots=(multi-proxy0 multi-proxy1 multi-proxy2)
ips=(172.0.0.2 172.0.0.3 172.0.0.4)
case "$action" in
  apply)
    case "$mode" in
      baseline) netem=(limit 10000) ;;
      delay) netem=(limit 10000 delay 100ms) ;;
      loss) netem=(limit 10000 loss 1%) ;;
      *) echo 'Usage: network.sh apply baseline|delay|loss' >&2; exit 2 ;;
    esac
    # Do not overwrite a pre-existing traffic control configuration.
    for robot in "${robots[@]}"; do
      qdisc=$(docker exec "$robot" tc qdisc show dev eth0)
      if [[ "$qdisc" != 'qdisc noqueue 0: root'* ]]; then
        echo "$robot: unexpected qdisc; refusing to overwrite: $qdisc" >&2; exit 1
      fi
    done
    created=()
    rollback() {
      for robot in "${created[@]}"; do docker exec "$robot" tc qdisc del dev eth0 root || true; done
    }
    trap rollback ERR INT TERM
    for i in 0 1 2; do
      robot=${robots[$i]}
      docker exec "$robot" tc qdisc add dev eth0 root handle 1: prio bands 3 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
      created+=("$robot")
      docker exec "$robot" tc qdisc add dev eth0 parent 1:3 handle 30: netem "${netem[@]}"
      for j in 0 1 2; do
        [[ $i == "$j" ]] && continue
        docker exec "$robot" tc filter add dev eth0 protocol ip parent 1: prio 3 u32 match ip dst "${ips[$j]}/32" flowid 1:3
      done
    done
    trap - ERR INT TERM
    ;;
  status)
    for robot in "${robots[@]}"; do
      echo "=== $robot ==="
      docker exec "$robot" tc -s qdisc show dev eth0
      docker exec "$robot" tc -s filter show dev eth0 parent 1: || true
    done
    ;;
  clear)
    for robot in "${robots[@]}"; do
      qdisc=$(docker exec "$robot" tc qdisc show dev eth0)
      if [[ "$qdisc" == *'qdisc prio 1: root'* ]]; then
        docker exec "$robot" tc qdisc del dev eth0 root
      elif [[ "$qdisc" != 'qdisc noqueue 0: root'* ]]; then
        echo "$robot: unexpected qdisc; refusing to delete: $qdisc" >&2; exit 1
      fi
    done
    ;;
  *) echo 'Usage: network.sh apply baseline|delay|loss | status | clear' >&2; exit 2 ;;
esac
