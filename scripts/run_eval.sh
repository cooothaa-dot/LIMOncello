#!/bin/bash
set -e

BAG_PATH=$1
CONFIG_NAME=$2
OUTPUT_DIR=$3

if [ -z "$BAG_PATH" ] || [ -z "$CONFIG_NAME" ] || [ -z "$OUTPUT_DIR" ]; then
  echo "Usage: ./run_eval.sh <bag_path> <config_name> <output_dir>"
  echo ""
  echo "Examples:"
  echo "  ./run_eval.sh ~/data/rcampus r_campus ~/results/rcampus"
  echo "  ./run_eval.sh ~/data/city02  city     ~/results/city02"
  echo ""
  echo "Config names (tu LIMOncello/config/):"
  ls ~/ros2_ws_limo/src/LIMOncello/config/*.yaml 2>/dev/null \
    | xargs -I{} basename {} .yaml | sed 's/^/  /'
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_limo/install/setup.bash

mkdir -p "$OUTPUT_DIR"

echo "=== Running LIMOncello ==="
echo "  Bag:    $BAG_PATH"
echo "  Config: $CONFIG_NAME"
echo "  Output: $OUTPUT_DIR"
echo ""

ros2 launch limoncello limoncello.launch.py \
  config_name:="$CONFIG_NAME" \
  use_sim_time:=true \
  rviz:=false

echo "=== Done. Results in $OUTPUT_DIR ==="
