#!/usr/bin/env bash
XVISIO_SERIALS=($(rostopic list 2>/dev/null | grep -o '/xv_sdk/[^/]*' | cut -d'/' -f3 | grep -v -E '^(parameter_descriptions|parameter_updates|new_device)$' | sort -u))
echo $XVISIO_SERIALS
exit 0
XVISIO_SERIAL_ARRAY=(${XVISIO_SERIALS[@]})    # 直接展开为数组（注意：空格会拆分元素）
#echo $XVISIO_SERIAL_ARRAY
# 遍历数组
for item in $XVISIO_SERIAL_ARRAY; do
    echo "当前序列号: $item"
    gnome-terminal -- bash -c "bash single_fastumi_monitor_menu.sh"
done
