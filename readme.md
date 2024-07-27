# Docker Environment

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
docker compose up -d
```

Now, you can visit the VNC by inputing `localhost:46080`, and click `vnc.html` in your browser.

Or, you can enter your container using terminal by:
```bash
docker exec -it slam_workspace-1 /bin/bash
```

# SourceCode Dependencies Installation: 

Now enter the cotainer either using VNC or by shell.

cd to the workspace folder (in `\root\workspace`). Clone the source code into the workspace folder:
```bash
git clone https://github.com/MrBernie/slam_in_autonomous_driving_bernie
```
Now install the Pangolin thirdparty dependencies:
```bash
cd slam_in_autonomous_driving_bernie/thirdparty
unzip Pangolin.zip
cd Pangolin
mkdir build
cd build
cmake ..
make -j 8 #The number depends on your CPU core number
```

Now install the g2o thirdparty dependencies:
cd to the `/workspace` first.
```bash
cd slam_in_autonomous_driving_bernie/thirdparty
cd g2o
mkdir build
cd build
cmake ..
make -j 8
```



