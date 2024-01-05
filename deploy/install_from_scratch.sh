
# download docker convenience instalation script
curl -fsSL https://get.docker.com -o get-docker.sh

# install docker
sudo sh ./get-docker.sh --dry-run

# create docker group
sudo groupadd docker

# add user to docker group to allow permissions
sudo usermod -aG docker $USER

# activate changes to the group in the current shell
newgrp docker
