#include <dirent.h>
#include <sys/types.h>

#include <condition_variable>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fstream"
#include "glog/logging.h"
#include "glog_sink.h"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/common/time.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "jarvis_brige.h"
#include "mutex"
#include "ostream"
#include "slip_detection/slip_detect.h"
#include "time.h"
#include "unistd.h"
//
Eigen::MatrixXd Matrixmult(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  Eigen::MatrixXd resutl = Eigen::MatrixXd::Zero(a.rows(), b.cols());
  for (int i = 0; i < a.rows(); i++) {
    for (int j = 0; j < b.cols(); j++) {
      resutl(i, j) = a.row(i) * b.col(j);
    }
  }
  return resutl;
};
Eigen::MatrixXd MatrixmultTemp(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  Eigen::MatrixXd resutl = Eigen::MatrixXd::Zero(a.rows(), b.cols());
  for (int i = 0; i < a.rows(); i++) {
    for (int j = 0; j < b.cols(); j++) {
      double r = 0;
      for (int k = 0; k < a.cols(); k++) {
        r += a(i, k) * b(k, j);
      }
      resutl(i, j) = r;
    }
  }
  return resutl;
};
//
int main(int argc, char *argv[]) {
  int n  = std::stoi(argv[1]);
  int m  = std::stoi(argv[2]);
  Eigen::MatrixXd test = Eigen::MatrixXd::Random(n, m);
  for (int i = 0; i < 1000; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd test1 = test * test.transpose();
    std::cout << "eigen cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::high_resolution_clock::now() - start)
                     .count()
              << std::endl;

    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd test2 = Matrixmult(test, test.transpose());
    // for (int i = 0; i < test1.rows(); i++) {
    //   for (int j = 0; j < test1.cols(); j++) {
    //     assert(test1(i, j) == test2(i, j));
    //   }
    // }
    std::cout << "my eigen cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::high_resolution_clock::now() - start)
                     .count()
              << std::endl;

    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd test3 = MatrixmultTemp(test, test.transpose());
    std::cout << "my a cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::high_resolution_clock::now() - start)
                     .count()
              << std::endl;

    // Eigen::MatrixXd test1 = test * test.transpose();
//     Eigen::MatrixXd test2 = test * test.transpose();
//     Eigen::MatrixXd test4 = test * test.transpose();
//     Eigen::MatrixXd test5 = test * test.transpose();
// Eigen::MatrixXd a= test1+test2+test2+test4;

  }

  return 0;
}