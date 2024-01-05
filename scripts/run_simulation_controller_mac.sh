
IP=192.168.56.226
PORT=1234
ROBOT_NAME=e-puck
CONTROLLER=controller


export WEBOTS_HOME=/Applications/Webots.app
export WEBOTS_HOME_PATH=$WEBOTS_HOME/Contents
export DYLD_LIBRARY_PATH=${WEBOTS_HOME}/Contents/lib/controller
export WEBOTS_CONTROLLER_URL=tcp://${IP}:${PORT}/${ROBOT_NAME}

make clean
make
./${CONTROLLER}