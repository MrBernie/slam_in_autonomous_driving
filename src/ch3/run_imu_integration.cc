//
// Created by xiang on 2021/11/5.
//

#include <glog/logging.h>
#include <iomanip>

#include "ch3/imu_integration.h"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

// DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_string(imu_txt_path, "./data/ch3/macins/IMU_data_output2_24_8_13.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(imu_initialization, true, "是否进行IMU初始化");
DEFINE_int32(imu_init_time, 5, "IMU初始化时间");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);
    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    sad::StaticIMUInit::Options options;
    if (FLAGS_imu_initialization) {
        options.use_speed_for_static_checking_ = false;
        options.init_time_seconds_ = FLAGS_imu_init_time;   // time for static checking
    }
    sad::StaticIMUInit imu_init(options);    // for imu init
    bool imu_inited = false;
    
    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    /// 记录结果
    // auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
    //                       const Vec3d& p, const Vec3d& acce, const Vec3d& gyro) {
    //     auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    //     auto save_quat = [](std::ofstream& fout, const Quatd& q) {
    //         fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    //     };

    //     fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
    //     save_vec3(fout, acce);
    //     save_vec3(fout, gyro);
    //     save_vec3(fout, p);
    //     save_quat(fout, R.unit_quaternion());
    //     save_vec3(fout, v);
    //     fout << std::endl;
    // };

    std::ofstream fout("./data/ch3/state.txt");
    // io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU Initialization
          if (!imu_init.InitSuccess() && FLAGS_imu_initialization) {
            imu_init.AddIMU(imu);
            return;
          }

          /// need IMU initialization
          if (!imu_inited && FLAGS_imu_initialization) {
              // estimate the noise using initializer
              //   options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              //   options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              //   eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              init_bg = imu_init.GetInitBg();
              init_ba = imu_init.GetInitBa();

              imu_integ = sad::IMUIntegration(gravity, init_bg, init_ba);

              imu_inited = true;
              return;
          }
          
          imu_integ.AddIMU(imu);
        //   save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP(), imu.acce_-imu_init.GetInitBa(), imu.gyro_-imu_init.GetInitBg());
        save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go();

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}