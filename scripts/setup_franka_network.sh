#!/usr/bin/env bash
# Configure an additional IPv4 address for a Franka FCI connection.
# Usage: ./scripts/setup_franka_network.sh
# This does NOT remove your existing IP address or default route.
set -euo pipefail

INTERFACE="${FRANKA_INTERFACE:-enp11s0}"
HOST_CIDR="${FRANKA_HOST_CIDR:-172.16.0.1/24}"
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"

if ! ip link show dev "$INTERFACE" >/dev/null 2>&1; then
  echo "Error: network interface '$INTERFACE' not found." >&2
  exit 1
fi

if [[ $(id -u) -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

ip link set dev "$INTERFACE" up
if ip -4 addr show dev "$INTERFACE" | grep -Fq "inet ${HOST_CIDR} "; then
  echo "Franka host address $HOST_CIDR is already configured on $INTERFACE."
else
  ip addr add "$HOST_CIDR" dev "$INTERFACE"
  echo "Added $HOST_CIDR to $INTERFACE (existing addresses preserved)."
fi

echo
echo "Route to Franka:"
ip route get "$ROBOT_IP"
echo
echo "Testing connectivity to $ROBOT_IP ..."
if ping -c 3 -W 1 "$ROBOT_IP"; then
  echo "Robot responded. You can run your libfranka program."
  echo "Franka Desk (if available on this interface): https://$ROBOT_IP"
else
  echo "WARNING: Robot did not respond. Check cabling and whether the robot"
  echo "is reachable on the same Ethernet segment. Adding an IP alone cannot"
  echo "make a robot accessible through a router that does not connect its network." >&2
  exit 2
fi
