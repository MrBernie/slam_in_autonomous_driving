These are the file changed to implement point cloud (MAC algorithm) + imu navigation

```
./src/ch3/macins/**
./src/common/io_utils.cc
./src/common/io_utils.h
./src/common/mac.h
./src/ch3/CMakeLists.txt
./src/ch3/imu_integration.h
./src/ch3/run_imu_integration.cc
./src/ch3/static_imu_init.cc
```

Put the data file in `./data/ch3/macins/`.
Default name of the input IMU and MAC txt file is `macins.txt`. Output is `macins_out.txt`.