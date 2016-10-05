/**
 *  @brief Robot 2D trajectory optimization
 *
 *  @author Atsushi Sakai
 *
 **/

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "math.h"
#include "csvparser.h"
#include "matplotlibcpp.h"
#include <vector>

using namespace std;

namespace plt = matplotlibcpp;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct OdometryConstraint{
  OdometryConstraint(double odo_x, double odo_y, double odo_yaw)
    :odo_x_(odo_x), odo_y_(odo_y), odo_yaw_(odo_yaw) {}


  template <typename T>
    bool operator()(const T* const odometry, T* residual) const {
      // residual[0]=T(y_)-
      residual[0]=T(0.0);
      return true;
    }

  static ceres::CostFunction* Create(
      const double x,
      const double y,
      const double yaw
      ){
    return (new ceres::AutoDiffCostFunction<OdometryConstraint,3,3>(
          new OdometryConstraint(x,y,yaw)));
  }

  private:
    //Observations for a sample
    const double odo_x_;
    const double odo_y_;
    const double odo_yaw_;
};

int main(int argc, char** argv){
  cout<<"Start similation"<<endl;
  google::InitGoogleLogging(argv[0]);

  //data read
  CSVParser csvparser("data.csv");

  //true trajectory
  vector<double> tx;
  vector<double> ty;
  vector<double> tyaw;

  for(int i=0;i<csvparser.ncol_;i++){
    tx.push_back(csvparser.data_[i][1]);
    ty.push_back(csvparser.data_[i][2]);
    tyaw.push_back(csvparser.data_[i][3]);
  }

  //parameter
  vector<double> x;
  vector<double> y;
  vector<double> yaw;

  for(int i=0;i<csvparser.ncol_;i++){
    x.push_back(csvparser.data_[i][4]);
    y.push_back(csvparser.data_[i][5]);
    yaw.push_back(csvparser.data_[i][6]);
  }

  //parameter
  vector<double> zx;
  vector<double> zy;

  for(int i=0;i<csvparser.ncol_;i++){
    zx.push_back(csvparser.data_[i][7]);
    zy.push_back(csvparser.data_[i][8]);
  }

  //init param
  vector<double> ix;
  vector<double> iy;
  vector<double> iyaw;
  ix=x;
  iy=y;
  iyaw=yaw;

  ceres::Problem problem;

  // for(int i=0;i<csvparser.ncol_;i++){
    // problem.AddResidualBlock(
        // OdometryConstraint::Create,
        // NULL,//損失関数
        // //パラメータ
        // )
  // }
  
  plt::named_plot("Truth",tx,ty, "-b");
  plt::named_plot("init",ix,iy, "-g");
  plt::named_plot("Estmated",x, y, "--r");
  plt::named_plot("GPS",zx, zy, "xk");
  plt::legend();
  plt::axis("equal");
  plt::grid(true);
  plt::show();

  return 0;
}

