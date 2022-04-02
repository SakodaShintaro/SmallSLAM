#ifndef MAP_DRAWER_H_
#define MAP_DRAWER_H_

#include <stdio.h>

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "Scan2D.h"

class MapDrawer {
 private:
  FILE *gp;     // gnuplotへのパイプ
  double xmin;  // 描画範囲[m]
  double xmax;
  double ymin;
  double ymax;
  double aspectR;  // xy比

 public:
  MapDrawer() : gp(nullptr), xmin(-10), xmax(10), ymin(-10), ymax(10), aspectR(-1.0) {}

  ~MapDrawer() { finishGnuplot(); }

  void initGnuplot() {
    // gp = popen("gnuplot", "w");  // パイプオープン.Linux
    gp = fopen("result.txt", "w");
  }

  void finishGnuplot() {
    if (gp != nullptr) pclose(gp);
  }

  void setAspectRatio(double a) {
    aspectR = a;
    fprintf(gp, "set size ratio %lf\n", aspectR);
  }

  void setRange(double R) {  // 描画範囲をR四方にする
    xmin = ymin = -R;
    xmax = ymax = R;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xR, double yR) {  // 描画範囲を±xR、±yRにする
    xmin = -xR;
    xmax = xR;
    ymin = -yR;
    ymax = yR;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xm, double xM, double ym, double yM) {  // 描画範囲を全部指定
    xmin = xm;
    xmax = xM;
    ymin = ym;
    ymax = yM;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void drawMapGp(const PointCloudMap &pcmap);
  void drawScanGp(const Scan2D &scan);
  void drawTrajectoryGp(const std::vector<Pose2D> &poses);
  void drawGp(const std::vector<LPoint2D> &lps, const std::vector<Pose2D> &poses, bool flush = true);
};

#endif
