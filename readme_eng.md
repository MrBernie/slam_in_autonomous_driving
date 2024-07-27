# Docker installation

Install the docker engine: https://docs.docker.com/engine/install/ubuntu/#install-from-a-package.

Remember to add the user to Docker Group: https://docs.docker.com/engine/install/linux-postinstall/

Pull the docker image: 
```bash
docker pull zijiechenrobotic/sad-workspace:latest
```

Create a `docker-compose.yml` file, entering:
```
version: '3.4'
services:
  sad-workspace:
    image: zijiechenrobotic/sad-workspace:latest
    container_name: slam_workspace-1
    environment:
      - VNC_PW=abc123 # VNC password
      - VNC_GEOMETRY=1920x1080 # VNC resolution
      - VNC_DEPTH=24 # VNC color
    volumes:
      - ./workspace:/root/workspace # ./worskapce is your path to keep source code in the host computer
    ports:
      # noVNC port:
      - 46080:6080
      # standard VNC port:
      - 45901:5901
```
Remember to create the folder `workspace` to keep the source code.

Run:
```bash
docker-compose up -d
```

Now, you can visit the VNC by inputing `localhost:46080` in your browser.

Or, you can enter your container using terminal by:
```bash
docker exec -it slam_workspace-1 /bin/bash
```

# Installation: 

cd to the workspace folder. Clone the source code into the workspace folder.
```bash
git clone https://github.com/MrBernie/slam_in_autonomous_driving
```


