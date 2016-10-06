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

struct GPSConstraint{
  GPSConstraint(double x, double y)
    :x_(x), y_(y) {}

  template <typename T>
    bool operator()(
        const T* const x,
        const T* const y,
        T* residual) const {
      residual[0]=x[0]-T(x_);
      residual[1]=y[0]-T(y_);
      return true;
    }

  static ceres::CostFunction* Create(
      const double gx,
      const double gy
      ){
    return (new ceres::AutoDiffCostFunction<GPSConstraint,2,1,1>(
          new GPSConstraint(gx,gy)));
  }

  private:
    const double x_;
    const double y_;
};

struct OdometryConstraint{
  OdometryConstraint(double v, double omega)
    :v_(v), omega_(omega) {}


  template <typename T>
    bool operator()(
        const T* const cx,
        const T* const cy,
        const T* const cyaw,
        const T* const nx,
        const T* const ny,
        const T* const nyaw,
        T* residual) const {
      residual[0]=cx[0]-(nx[0]+v_*0.1*cos(nyaw[0]));
      residual[1]=cy[0]-(ny[0]+v_*0.1*sin(nyaw[0]));
      residual[2]=cyaw[0]-(nyaw[0]+0.1*omega_);
      return true;
    }

  static ceres::CostFunction* Create(
      const double v,
      const double omega
      ){
    return (new ceres::AutoDiffCostFunction<OdometryConstraint,3,1,1,1,1,1,1>(
          new OdometryConstraint(v,omega)));
  }

  private:
    //Observations for a sample
    const double v_;
    const double omega_;
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

  //input
  vector<double> v;
  vector<double> omega;

  for(int i=0;i<csvparser.ncol_;i++){
    v.push_back(csvparser.data_[i][9]);
    omega.push_back(csvparser.data_[i][10]);
  }

  //init param
  vector<double> ix;
  vector<double> iy;
  vector<double> iyaw;
  ix=x;
  iy=y;
  iyaw=yaw;

  //====Optimization=====
  ceres::Problem problem;

  for(int i=0;i<csvparser.ncol_-1;i++){
    // odometry constraint
    problem.AddResidualBlock(
        OdometryConstraint::Create(v[i],omega[i]),
        NULL,//損失関数
        // currentState,//パラメータ
        // nextState//パラメータ
        &(x[i]),
        &(y[i]),
        &(yaw[i]),
        &(x[i+1]),
        &(y[i+1]),
        &(yaw[i+1])
        );

    //gps constraint
    if(fabs(zx[i])>=0.001){
      // cout<<zx[i]<<","<<zy[i]<<endl;

      problem.AddResidualBlock(
        GPSConstraint::Create(zx[i],zy[i]),
        NULL,
        &x[i],
        &y[i]
        );
    }
  }
  //最適化の実行
  Solver::Options options;//最適化のオプション設定用構造体
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;//最適化の結果を標準出力に表示する。
  Solver::Summary summary;//最適化の結果を格納するよう構造体
  Solve(options,&problem,&summary);//最適化の実行

  for(int i=0;i<x.size();i++){
    cout<<x[i]<<","<<y[i]<<","<<yaw[i]<<endl;
  }

  plt::named_plot("Truth",tx,ty, "-b");
  plt::named_plot("init",ix,iy, "-g");
  plt::named_plot("Estmated",x, y, "-r");
  plt::named_plot("GPS",zx, zy, "xk");
  plt::legend();
  plt::axis("equal");
  plt::grid(true);
  plt::show();

  return 0;
}

