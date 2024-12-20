#!/bin/bash

# 初始化变量
pkg_path=""
output_base_path=""
execute_lat=0
execute_longi=0
execute_wm=0
execute_lat_lon_decider=0
execute_lat_lon_motion=0
execute_interaction_search=0
execute_lc=0
execute_sdmap=0

# 解析命令行参数
while (( "$#" )); do
  case "$1" in
    -s) pkg_path=$2; shift 2 ;;
    -o) output_base_path=$2; shift 2 ;;
    --lat) execute_lat=1; shift ;;
    --longi) execute_longi=1; shift ;;
    --wm) execute_wm=1; shift ;;
    --lat_lon_decider) execute_lat_lon_decider=1; shift ;;
    --lat_lon_motion) execute_lat_lon_motion=1;  shift ;;
    --interaction_search) execute_interaction_search=1;  shift ;;
    --sdmap) execute_sdmap=1; shift ;;
    --lc) execute_lc=1; shift ;;
    *) echo "Invalid option: $1" >&2; exit 1 ;;
  esac
done

# 检查是否提供了-s选项
if [ -z "$pkg_path" ]; then
    echo "Usage: $0 -s <xxx/dmppcl.00000.rsclbag> [-o <生成的html路径>] [--lat] [--longi] [--wm]"
    exit 1
fi

# 如果没有提供-o选项，设置输出路径为-s选项指定的目录
if [ -z "$output_base_path" ]; then
    output_base_path=$(dirname "$pkg_path")
fi

# 如果没有指定任何选项，则执行所有脚本
if [ $execute_lat -eq 0 ] && [ $execute_longi -eq 0 ] && [ $execute_wm -eq 0 ] \
     && [ $execute_lat_lon_decider -eq 0 ] && [ $execute_lat_lon_motion -eq 0 ] \
     && [ $execute_lc -eq 0 ] \
     && [ $execute_sdmap -eq 0 ] && [ $execute_interaction_search -eq 0 ] \
    ; then
    execute_lat=1
    execute_longi=1
    execute_wm=1
    execute_lat_lon_decider=1
    execute_lat_lon_motion=1
    execute_interaction_action=1
    execute_lc=1
    execute_sdmap=1
fi

# 获取-s后跟着的rsclbag文件名
file_name=$(basename "$pkg_path")

# 使用cut命令截取所需部分
desired_part=$(echo "$file_name" | cut -d'.' -f1)

# 根据选项执行相应的脚本
if [ $execute_lat -eq 1 ]; then
    python3.7 lat_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_lat_$desired_part.html"
fi

if [ $execute_longi -eq 1 ]; then
    python3.7 longi_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_longi_$desired_part.html"
fi

if [ $execute_wm -eq 1 ]; then
    python3.7 wm_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_wm_$desired_part.html"
fi

if [ $execute_lat_lon_decider -eq 1 ]; then
    python3.7 lat_lon_decider_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_lat_lon_decider_$desired_part.html"
fi

if [ $execute_lat_lon_motion -eq 1 ]; then
    python3.7 lat_lon_motion_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_lat_lon_motion_$desired_part.html"
fi

if [ $execute_interaction_search -eq 1 ]; then
    python3.7 interaction_search_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_interaction_search_$desired_part.html"
fi

if [ $execute_lc -eq 1 ]; then
    python3.7 lc_xviz.py -s "$pkg_path" -o "${output_base_path}/xviz_lc_$desired_part.html"
fi

if [ $execute_sdmap -eq 1 ]; then
    python3.7 lat_lon_motion_sdmap.py -s "$pkg_path" -o "${output_base_path}/xviz_sdmap_$desired_part.html"
fi
