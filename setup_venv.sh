source /opt/ros/humble/setup.bash

VENV_NAME="gesture_commander_venv"

echo "Creating virtual environment named $VENV_NAME ... "
python3 -m venv $VENV_NAME

source ./$VENV_NAME/bin/activate
echo "Python path: $(which python3)"

echo "Installing wheels for Jetson ..."
wget https://nvidia.box.com/shared/static/zvultzsmd4iuheykxy17s4l2n91ylpl8.whl -O torch-2.3.0-cp310-cp310-linux_aarch64.whl
wget https://nvidia.box.com/shared/static/u0ziu01c0kyji4zz3gxam79181nebylf.whl -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl
pip install torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip install torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

echo "Installing ultralytics compatible with the previous torch versions ..."
pip3 install numpy==1.26.4 opencv-python==4.10.0.84 ultralytics

echo "Finished. Please update setup.cfg!"