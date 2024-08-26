
// #include "ch3/eskf.hpp"
#include "ch3/macins/eskf_macins.hpp"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
// #include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "./data/ch3/macins/IMU_MAC_data_output2_24_8_13.txt", "data file path");
DEFINE_string(txt_path_out, "./data/ch3/macins/macins_out.txt", "output data file path");
// DEFINE_double(antenna_angle, 12.06, "RTK antenna angle）");
// DEFINE_double(antenna_pox_x, -0.17, "RTK antenna offset X");
// DEFINE_double(antenna_pox_y, -0.20, "RTK antenna offset Y");

// camera settings
DEFINE_double(camera_angle, 0.0, "camera angle");
DEFINE_double(camera_pox_x, 0.0, "camera offset X");
DEFINE_double(camera_pox_y, 0.0, "camera offset Y");
DEFINE_double(camera_pox_z, 0.0, "camera offset Z");

DEFINE_bool(imu_initialization, true, "IMU initialization or not");
DEFINE_int32(imu_init_time, 5, "IMU initialization time");
DEFINE_double(imu_dt, 0.04, "IMU time interval tolerance");

DEFINE_bool(with_ui, true, "use UI or not");
DEFINE_bool(with_odom, false, "use wheel odometry or not");

/**
 * Demonstrate the combination of point cloud (mac) + imu navigation
 */
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    // Initializer
    // sad::StaticIMUInit imu_init;  // use the default settings

    sad::StaticIMUInit::Options options;
    if (FLAGS_imu_initialization) {
        options.use_speed_for_static_checking_ = false;
        options.init_time_seconds_ = FLAGS_imu_init_time;   // time for static checking
    }
    sad::StaticIMUInit imu_init(options);    // for imu init

    sad::ESKFD_MACINS eskf;

    sad::TxtIO io(FLAGS_txt_path);
    // Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);


    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    std::ofstream fout(FLAGS_txt_path_out);
    bool imu_inited = false, mac_inited = false;

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// call back function settings
    // bool first_gnss_set = false;
    bool first_mac_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU handling
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// need IMU initialization
          if (!imu_inited) {
              // get the zero bias and set the ESKF
              sad::ESKFD_MACINS::Options options;
              // we use higher imu dt tolerance
              options.imu_dt_ = FLAGS_imu_dt;
              // estimate the noise using initializer
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }

          if (!mac_inited) {
              /// wait for the initialization of MAC
              return;
          }

          /// after we get the data from MAC, we can start predicting
          eskf.Predict(imu);

          /// prediction updates ESKF, we can get a nomimal state
          auto state = eskf.GetNominalState();
          if (ui) {
              ui->UpdateNavState(state);
          }

          /// save the result for map plotting
          save_result(fout, state);

          usleep(1e3);
      })
        .SetMACProcessFunc([&](const sad::MAC& mac) {
            /// PointCloud (MAC algorithm) handling function
            if (!imu_inited) {
                return;
            }
            //  convert TWC to TWB
            sad::MAC mac_convert = mac;
            // if(!eskf.MAC2World(mac_convert, FLAGS_camera_angle, Vec3d(FLAGS_camera_pox_x, FLAGS_camera_pox_y, FLAGS_camera_pox_z))) {
            //     return;
            // }
            //  set the origin
            if (!first_mac_set) {
                origin = mac_convert.pose_SE3.translation();
                first_mac_set = true;
            }
            mac_convert.pose_SE3.translation() -= origin;
            
            //  ESKF Update
            eskf.ObserveMAC(mac_convert);

            auto state = eskf.GetNominalState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);

            mac_inited = true;
        })
        // .SetOdomProcessFunc([&](const sad::Odom& odom) {
        //     /// Odom handling, in this Chapter, odometry is only used for IMU initialization
        //     imu_init.AddOdom(odom);
        //     if (FLAGS_with_odom && imu_inited && mac_inited) {
        //         eskf.ObserveWheelSpeed(odom);
        //     }
        // })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}