#! /bin/bash


    echo "=================================="
    echo "=   press Ctrl + c to close      ="
    echo "=   press Ctrl + c to close      ="
    echo "=================================="

sudo chmod 777 /dev/ttyS0

#python3 security-cam-serial-fixedcam.py --video 0 & python3 key_control_py3.py
#python3 security-cam-rpi3B-serial-movecam.py --video 0 --graph face_graph --labels face_labels.txt & python3 key_control_py3.py
python3 security-cam-rpi3B-serial-movecam.py & python3 key_control_py3.py

read -p "press Enter to exit" a
