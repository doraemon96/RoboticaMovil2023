#!/bin/bash
# Robotica Movil 2023 - Trabajo Practico 3
#  This script downloads all necessary files to work with EuRoC calibration

EUROC_FOLDER_DESTINATION="EuRoC/cam_checkerboard"
EUROC_CALIBRATION_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_checkerboard"
EUROC_CALIBRATION_FILES=("checkerboard_7x6.yaml" "cam_checkerboard.zip")

# create data folder
if [ ! -d $EUROC_FOLDER_DESTINATION ]; then
  mkdir -p $EUROC_FOLDER_DESTINATION;
fi

# download calibration files
for file in ${EUROC_CALIBRATION_FILES[@]}; do
  FILE_DOWNLOAD_URL=$EUROC_CALIBRATION_URL/$file
  FILE_DOWNLOAD_PATH=$EUROC_FOLDER_DESTINATION/$file
  echo Downloading $FILE_DOWNLOAD_URL to $FILE_DOWNLOAD_PATH
  curl -# -C - -o $FILE_DOWNLOAD_PATH $FILE_DOWNLOAD_URL
done

# unzip zipped files
echo Unzipping zipped files \(silently\)
cd $EUROC_FOLDER_DESTINATION
unzip -n -q *.zip -x '__MACOSX/*'
