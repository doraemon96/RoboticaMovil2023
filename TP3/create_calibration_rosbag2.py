#!/env/bin/python3
# Robotica Movil 2023 - Trabajo Practico 3
#  This script creates the calibration Rosbag2 from the raw timestamped
#   data. The data should already be downloaded and uncompressed.
from pathlib import Path
from rosbags.serde import serialize_cdr
from rosbags.typesys.types import builtin_interfaces__msg__Time as Time
from rosbags.typesys.types import sensor_msgs__msg__CompressedImage as CompressedImage
from rosbags.typesys.types import std_msgs__msg__Header as Header
import argparse
import csv
import numpy as np
import rosbags.rosbag2 as r2


def write_images(bagwriter, image_folder, csv_path, topic='/camera', frameid='map'):

    timestamped_images = list()
    with open(csv_path, 'r', newline='') as csvfile:
        csvreader = csv.DictReader(csvfile)
        for row in csvreader:
            timestamped_images.append((image_folder / row['filename'], np.int64(row['#timestamp [ns]'])))

    conn = bagwriter.add_connection(topic, CompressedImage.__msgtype__)
    for path, timestamp in timestamped_images:
        message = CompressedImage(
                Header(
                    stamp=Time(
                        sec=int(timestamp // 10**9),
                        nanosec=int(timestamp % 10**9),
                    ),
                    frame_id=frameid,
                ),
                format='png',  # could also be 'png'
                data=np.fromfile(path, dtype=np.uint8),
            )
        bagwriter.write(
            conn,
            timestamp,
            serialize_cdr(message, message.__msgtype__),
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--folder', type=Path, default='EuRoC/cam_checkerboard',
        help='Top folder with EuRoC calibration data'
    )
    parser.add_argument(
        '-o', '--outfile', type=Path, default='EuRoC/cam_checkerboard/cam_checkerboard_rosbag2',
        help='Output rosbag2 DB3 file to create'
    )
    parser.add_argument(
        '--debug', action='store_true'
    )

    args = parser.parse_args()

    cam0_images = (args.folder / 'mav0/cam0/data/')
    cam0_csv = (args.folder / 'mav0/cam0/data.csv')
    cam1_images = (args.folder / 'mav0/cam1/data/')
    cam1_csv = (args.folder / 'mav0/cam1/data.csv')

    with r2.Writer(args.outfile) as bagwriter:
        write_images(bagwriter, cam0_images, cam0_csv, topic='/cam0/')

        write_images(bagwriter, cam1_images, cam1_csv, topic='/cam1/')

    print(f'Bag has been written to {args.outfile} successfully!')