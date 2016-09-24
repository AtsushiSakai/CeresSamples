/**
 *  @brief Powell Optimimization Sample
 *
 *  @author Atsushi Sakai
 *
 **/

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "math.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

/**
 *  @brief コスト関数
 **/
struct F1{
  template <typename T>
  bool operator()(
      const T* const x1,  
      const T* const x2,
      T* residual) const{
    residual[0]=x1+10.0*x2;
    return true;
    }
};

struct F2{
  template <typename T>
  bool operator()(
      const T* const x3,  
      const T* const x4,
      T* residual) const{
    residual[0]=T(sqrt(5.0))*(x3-x4);
    return true;
    }
};

struct F3{
  template <typename T>
  bool operator()(
      const T* const x2,  
      const T* const x3,
      T* residual) const{
    residual[0]=(x2-2.0*x3)*(x2-2.0*x3);
    return true;
    }
};

struct F4{
  template <typename T>
  bool operator()(
      const T* const x1,  
      const T* const x4,
      T* residual) const{
    residual[0]=T(sqrt(10.0))*(x1-x4)*(x1-x4);
    return true;
    }
};





int main(int argc, char** argv){
  // google::InitGoogleLogging(argv[0]);

  // //最適化問題を解く変数と初期値の設定
  // double initial_x=15.0;
  // double x=initial_x;

  // //最適化問題を解く用のオブジェクトの生成
  // Problem problem;

  // //コスト関数の設定
  // //AutoDiffCostFunctionを使うことで、自動的にヤコビ行列を設定できる
  // CostFunction* cost_function=new AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);

  // //最適化問題に残差項と変数を設定
  // problem.AddResidualBlock(cost_function,NULL,&x);

  // //最適化の実行
  // Solver::Options options;//最適化のオプション設定用構造体
  // options.linear_solver_type=ceres::DENSE_QR;
  // options.minimizer_progress_to_stdout=true;//最適化の結果を標準出力に表示する。
  // Solver::Summary summary;//最適化の結果を格納するよう構造体
  // Solve(options,&problem,&summary);//最適化の実行

  // //結果の表示
  // std::cout<<summary.BriefReport()<<std::endl;
  // std::cout<<"x:"<<initial_x<<"->"<<x<<std::endl;

  return 0;
}


