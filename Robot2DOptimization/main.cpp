/**
 *  @brief Robot 2D trajectory optimization
 *
 *  @author Atsushi Sakai
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
  GPSConstraint(double x, double y, double n)
    :x_(x), y_(y), n_(n) {}

  template <typename T>
    bool operator()(
        const T* const x,
        const T* const y,
        T* residual) const {
      residual[0]=(x[0]-T(x_))/n_;
      residual[1]=(y[0]-T(y_))/n_;
      return true;
    }

  static ceres::CostFunction* Create(
      const double gx,
      const double gy,
      const double gn
      ){
    return (new ceres::AutoDiffCostFunction<GPSConstraint,2,1,1>(
          new GPSConstraint(gx,gy,gn)));
  }

  private:
    const double x_;//gps position x
    const double y_;//gps position y
    const double n_;//gps xy accuracy
};

struct OdometryConstraint{
  OdometryConstraint(double dl, double dtheta, double dl_n, double dtheta_n)
    :dl_(dl), dtheta_(dtheta), dl_n_(dl_n), dtheta_n_(dtheta_n) {}

  template <typename T>
    bool operator()(
        const T* const cx,
        const T* const cy,
        const T* const cyaw,
        const T* const nx,
        const T* const ny,
        const T* const nyaw,
        T* residual) const {
      residual[0]=(nx[0]-(cx[0]+dl_*cos(cyaw[0])))/dl_n_;
      residual[1]=(ny[0]-(cy[0]+dl_*sin(cyaw[0])))/dl_n_;
      residual[2]=(nyaw[0]-(cyaw[0]+dtheta_))/dtheta_n_;
      return true;
    }

  static ceres::CostFunction* Create(
      const double dl,
      const double dtheta,
      const double dl_n,
      const double dtheta_n
      ){
    return (new ceres::AutoDiffCostFunction<OdometryConstraint,3,1,1,1,1,1,1>(
          new OdometryConstraint(dl,dtheta,dl_n, dtheta_n)));
  }

  private:
    const double dl_;//move distance
    const double dtheta_;//change angle
    const double dl_n_;// move distance accuracy
    const double dtheta_n_;// change angle accuracy
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
  //parameter
  vector<double> x;
  vector<double> y;
  vector<double> yaw;
  //observation
  vector<double> zx;
  vector<double> zy;
  vector<double> zn;
  //input
  vector<double> dl;
  vector<double> dtheta;
  vector<double> dl_n;
  vector<double> dtheta_n;

  for(int i=0;i<csvparser.ncol_;i++){
    tx.push_back(csvparser.data_[i][1]);
    ty.push_back(csvparser.data_[i][2]);
    tyaw.push_back(csvparser.data_[i][3]);
    x.push_back(csvparser.data_[i][4]);
    y.push_back(csvparser.data_[i][5]);
    yaw.push_back(csvparser.data_[i][6]);
    zx.push_back(csvparser.data_[i][7]);
    zy.push_back(csvparser.data_[i][8]);
    dl.push_back(csvparser.data_[i][9]);
    dtheta.push_back(csvparser.data_[i][10]);
    zn.push_back(csvparser.data_[i][11]);
    dl_n.push_back(csvparser.data_[i][12]);
    dtheta_n.push_back(csvparser.data_[i][13]);
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
        OdometryConstraint::Create(dl[i],dtheta[i],dl_n[i],dtheta_n[i]),
        NULL,
        &(x[i]),
        &(y[i]),
        &(yaw[i]),
        &(x[i+1]),
        &(y[i+1]),
        &(yaw[i+1])
        );

    //gps constraint
    if(fabs(zn[i])>=0.001){
      problem.AddResidualBlock(
        GPSConstraint::Create(zx[i],zy[i],zn[i]),
        NULL,
        &x[i],
        &y[i]
        );
    }
  }

  //Optimization
  Solver::Options options;
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;
  Solver::Summary summary;
  Solve(options,&problem,&summary);

  plt::named_plot("Truth",tx,ty, "-b");
  plt::named_plot("init",ix,iy, "-g");
  plt::named_plot("Estmated",x, y, "-r");
  plt::named_plot("GPS", zx, zy, "xk");
  plt::legend();
  plt::axis("equal");
  plt::grid(true);
  plt::show();

  return 0;
}

