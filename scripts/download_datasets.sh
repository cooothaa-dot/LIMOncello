#!/bin/bash
# Huong dan tai datasets cho LIMOncello eval
# Chay tren Monster: bash scripts/download_datasets.sh

DATA_DIR=~/data/limoncello
mkdir -p "$DATA_DIR"
echo "Thu muc data: $DATA_DIR"
echo ""

echo "=== Datasets can tai ==="
echo ""
echo "1. R-Campus (~2GB) - Test nhanh, end-to-end drift"
echo "   Config: r_campus.yaml (lidar type: 4 = livox_interfaces)"
echo "   Link:   https://github.com/ZCao-lab/RESPLE -> Data section"
echo "   Luu:    $DATA_DIR/rcampus/"
echo ""
echo "2. MCD NTU Campus (~20-30GB) - Day/night sequences"
echo "   Config: mcd_viral_ntu.yaml"
echo "   Link:   https://mcdviral.github.io/ -> NTU subset"
echo "   Luu:    $DATA_DIR/mcd_ntu/"
echo ""
echo "3. GrandTour (~15-20GB) - Indoor/outdoor, stairwell"
echo "   Config: grandtour.yaml"
echo "   Link:   https://github.com/leggedrobotics/grand_tour"
echo "   Luu:    $DATA_DIR/grandtour/"
echo ""
echo "4. City02 tunnel (~5-10GB) - Showcase SGal(3)"
echo "   Config: city.yaml"
echo "   Link:   https://sites.google.com/view/mulran-pr/dataset"
echo "   Luu:    $DATA_DIR/city02/"
echo ""
echo "Sau khi tai xong, chay eval:"
echo "  bash scripts/run_eval.sh \$DATA_DIR/rcampus r_campus ~/results/rcampus"
