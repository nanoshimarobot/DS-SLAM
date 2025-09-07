#!/bin/bash

# --- ROS環境のセットアップ ---
# ご自身の環境に合わせて変更してください (例: /opt/ros/noetic/setup.bash)
# source /path/to/your/ros/workspace/devel/setup.bash

# echo "ROS環境をセットアップしました。"

# --- 設定項目 ---

# データセットが格納されているベースディレクトリ
BASE_DATASET_DIR="/root/Dataset"

# 処理したいデータセット名のリスト (スペース区切りで列挙)
# 例: DATASET_NAMES=("rgbd_dataset_freiburg1_xyz" "rgbd_dataset_freiburg2_desk" "...")
DATASET_NAMES=(
#   "rgbd_dataset_freiburg3_sitting_static"
  "rgbd_dataset_freiburg3_walking_static"
#   "rgbd_dataset_freiburg3_sitting_xyz"
  # ここに他のデータセット名を追加
)

# 実行するlaunchファイルの情報
PACKAGE_NAME="ORB_SLAM2_PointMap_SegNetM"
LAUNCH_FILE="docker_demo.launch"
ASSOCIATE_PY_SCRIPT="data/associate.py"

# --- メインループ ---
echo "===================================================="
echo "Starting SLAM processing for multiple datasets..."
echo "Target Dataset size: ${#DATASET_NAMES[@]}"

# データセットのリストをループ処理
for dataset_name in "${DATASET_NAMES[@]}"
do
  # データセットのフルパスを生成
  full_dataset_path="${BASE_DATASET_DIR}/${dataset_name}"
  
  # association.txt のフルパスを生成
  association_file_path="${full_dataset_path}/association.txt"

  # データセットディレクトリの存在確認
  if [ ! -d "$full_dataset_path" ]; then
    echo "Warning: Directory not found: $full_dataset_path"
    echo "Skipping this dataset."
    continue # 次のループへ
  fi
  # --- association.txtの自動生成ロジック ---
  # もしassociation.txtが存在しない場合
  if [ ! -f "$association_file_path" ]; then
    echo "File not found: association.txt. Attempting to generate it..."
    
    rgb_txt_path="${full_dataset_path}/rgb.txt"
    depth_txt_path="${full_dataset_path}/depth.txt"

    # 生成に必要なrgb.txtとdepth.txtが存在するか確認
    if [ -f "$rgb_txt_path" ] && [ -f "$depth_txt_path" ]; then
      # pythonスクリプトを実行し、標準出力をassociation.txtにリダイレクトして保存
      python ${ASSOCIATE_PY_SCRIPT} ${rgb_txt_path} ${depth_txt_path} > ${association_file_path}
      echo "Successfully generated: ${association_file_path}"
    else
      echo "Error: Cannot generate association.txt because rgb.txt or depth.txt is missing in ${full_dataset_path}"
      echo "Skipping this dataset."
      continue # 次のループへ
    fi
  else
    echo "Found existing association.txt. Using it."
  fi

  echo "----------------------------------------------------"
  echo "Processing dataset: $dataset_name"
  echo "----------------------------------------------------"

  # roslaunchコマンドの実行
  # `:=` を使うことでlaunchファイル内の<arg>の値を上書きする
  roslaunch ${LAUNCH_FILE} \
    dataset:=${full_dataset_path} \
    associate:=${association_file_path}

  # SLAMプロセスが終了するまで待機
  echo "Dataset $dataset_name processing completed."
  sleep 2 # 次のlaunchまでの短い待機時間
done

echo "===================================================="
echo "All dataset processing completed."
echo "===================================================="