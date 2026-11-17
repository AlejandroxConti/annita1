#!/bin/bash
set -e
cd ~/vffe-rt1170
west build -b phyboard_atlas/mimxrt1176/cm4 -d build_m4 my_apps/uart_m4_bridge
OUT=my_apps/uart_integration/src/m4_image/m4_image_data.c
(cd build_m4/zephyr && xxd -i zephyr.bin) \
  | sed -E 's/^unsigned char [A-Za-z0-9_]+\[\]/unsigned char g_m4_image[]/; s/^unsigned int [A-Za-z0-9_]+ =/unsigned int g_m4_image_len =/' \
  > $OUT
tail -1 $OUT
west build -b phyboard_atlas/mimxrt1176/cm7 -d build_int my_apps/uart_integration
west flash -d build_int
echo "=== INTEGRACION OK ==="
