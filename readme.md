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

# Source Code Dependencies Installation: 

Now enter the cotainer either using VNC or by terminal.

cd to the workspace folder (in `\root\workspace`). Clone the source code into the workspace folder:
```bash
git clone https://github.com/MrBernie/slam_in_autonomous_driving
```
Now install the Pangolin thirdparty dependencies:
```bash
cd slam_in_autonomous_driving/thirdparty
unzip Pangolin.zip
cd Pangolin
mkdir build
cd build
cmake ..
make -j
```

Now install the g2o thirdparty dependencies:
cd to the `/workspace` first.
```bash
cd slam_in_autonomous_driving/thirdparty
cd g2o
cmake .
make -j
```

# Source Code Compiling and Running:
cd to the `/workspace` first.
```bash
cd slam_in_autonomous_driving
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```
Now you can compile the project you need.
For example, compiling the `motion.cc` in chapter 2.

cd to the `/slam_in_autonomous_driving/build` and run
```bash
make motion
```

To run the project. (Some project must be run in the VNC due to graphic UI requirements)

cd to the `/slam_in_autonomous_driving` and run
```bash
./bin/motion
```

# MAC algorithm and IMU
Source code implemented in `./src/ch3/macins/`




