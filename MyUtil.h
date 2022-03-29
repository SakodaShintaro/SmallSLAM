#ifndef MY_UTIL_H_
#define MY_UTIL_H_

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979  // 円周率
#endif

#ifndef NULL
#define NULL 0  // 基本的には、C++11のnullptrを使う
#endif

#define DEG2RAD(x) ((x)*M_PI / 180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180 / M_PI)  // ラジアンから度

typedef unsigned char uchar;

//////////

class MyUtil {
 public:
  MyUtil(void) {}

  ~MyUtil(void) {}

  ///////////

  static int add(int a1, int a2);
  static double add(double a1, double a2);
  static double addR(double a1, double a2);

  static Eigen::Matrix3d svdInverse(const Eigen::Matrix3d &A);
  static void calEigen2D(double (*mat)[2], double *vals, double *vec1, double *vec2);
};

#endif
