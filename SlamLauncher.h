#ifndef SLAM_LAUNCHER_H_
#define SLAM_LAUNCHER_H_

#include <unistd.h>

#include <vector>

#include "FrameworkCustomizer.h"
#include "MapDrawer.h"
#include "PointCloudMapBS.h"
#include "SensorDataReader.h"
#include "SlamFrontEnd.h"

class SlamLauncher {
 private:
  int startN;         // 開始スキャン番号
  int drawSkip;       // 描画間隔
  bool odometryOnly;  // オドメトリによる地図構築か
  Pose2D ipose;       // オドメトリ地図構築の補助データ。初期位置の角度を0にする

  Pose2D lidarOffset;  // レーザスキャナとロボットの相対位置

  SensorDataReader sreader;  // ファイルからのセンサデータ読み込み
  PointCloudMap *pcmap;      // 点群地図
  MapDrawer mdrawer;         // gnuplotによる描画

  SlamFrontEnd sfront;          // SLAMフロントエンド
  FrameworkCustomizer fcustom;  // フレームワークの改造

 public:
  SlamLauncher() : startN(0), drawSkip(10), odometryOnly(false) {}

  void setStartN(int n) { startN = n; }

  void setOdometryOnly(bool p) { odometryOnly = p; }

  void run();
  void showScans();
  void mapByOdometry(Scan2D *scan);
  bool setFilename(const std::string &filename);
  void skipData(int num);
  void customizeFramework();
};

#endif
