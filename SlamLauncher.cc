#include "SlamLauncher.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/timer.hpp>

// #include "ScanPointResampler.h"

using namespace std;  // C++標準ライブラリの名前空間を使う

void SlamLauncher::run() {
  mdrawer.setAspectRatio(-0.9);  // x軸とy軸の比（負にすると中身が一定）

  size_t cnt = 0;                    // 処理の論理時刻
  if (startN > 0) skipData(startN);  // startNまでデータを読み飛ばす

  double totalTime = 0, totalTimeDraw = 0, totalTimeRead = 0;
  Scan2D scan;
  bool eof = sreader.loadScan(cnt, scan);  // ファイルからスキャンを1個読み込む
  boost::timer tim;
  while (!eof) {
    if (odometryOnly) {  // オドメトリによる地図構築（SLAMより優先）
      if (cnt == 0) {
        ipose = scan.pose;
        ipose.calRmat();
      }
      mapByOdometry(&scan);
    } else {
      sfront.process(scan);  // SLAMによる地図構築
    }

    double t1 = 1000 * tim.elapsed();

    if (cnt % drawSkip == 0) {  // drawSkipおきに結果を描画
      mdrawer.drawMapGp(*pcmap);
    }
    double t2 = 1000 * tim.elapsed();

    ++cnt;                              // 論理時刻更新
    eof = sreader.loadScan(cnt, scan);  // 次のスキャンを読み込む

    double t3 = 1000 * tim.elapsed();
    totalTime = t3;              // 全体処理時間
    totalTimeDraw += (t2 - t1);  // 描画時間の合計
    totalTimeRead += (t3 - t2);  // ロード時間の合計

    printf("---- SlamLauncher: cnt=%lu ends ----\n", cnt);
  }
  sreader.closeScanFile();

  printf("Elapsed time: mapping=%g, drawing=%g, reading=%g\n", (totalTime - totalTimeDraw - totalTimeRead),
         totalTimeDraw, totalTimeRead);
  printf("SlamLauncher finished.\n");
}

// 開始からnum個のスキャンまで読み飛ばす
void SlamLauncher::skipData(int num) {
  Scan2D scan;
  bool eof = sreader.loadScan(0, scan);
  for (int i = 0; !eof && i < num; i++) {  // num個空読みする
    eof = sreader.loadScan(0, scan);
  }
}

///////// オドメトリのよる地図構築 //////////

void SlamLauncher::mapByOdometry(Scan2D *scan) {
  //  Pose2D &pose = scan->pose;               // スキャン取得時のオドメトリ位置
  Pose2D pose;
  Pose2D::calRelativePose(scan->pose, ipose, pose);
  vector<LPoint2D> &lps = scan->lps;  // スキャン点群
  vector<LPoint2D> glps;              // 地図座標系での点群
  for (size_t j = 0; j < lps.size(); j++) {
    LPoint2D &lp = lps[j];
    LPoint2D glp;
    pose.globalPoint(lp, glp);  // センサ座標系から地図座標系に変換
    glps.emplace_back(glp);
  }

  // 点群地図pcmapにデータを格納
  pcmap->addPose(pose);
  pcmap->addPoints(glps);
  pcmap->makeGlobalMap();

  printf("Odom pose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);
}

////////// スキャン描画 ////////

void SlamLauncher::showScans() {
  mdrawer.setRange(6);           // 描画範囲。スキャンが6m四方の場合
  mdrawer.setAspectRatio(-0.9);  // x軸とy軸の比（負にすると中身が一定）

  // ScanPointResampler spres;

  size_t cnt = 0;                    // 処理の論理時刻
  if (startN > 0) skipData(startN);  // startNまでデータを読み飛ばす

  Scan2D scan;
  bool eof = sreader.loadScan(cnt, scan);
  while (!eof) {
    //    spres.resamplePoints(&scan);         // コメントアウトはずせば、スキャン点間隔を均一にする。

    // 描画間隔をあける
    usleep(100000);  // Linuxではusleep

    mdrawer.drawScanGp(scan);  // スキャン描画

    printf("---- scan num=%lu ----\n", cnt);
    eof = sreader.loadScan(cnt, scan);
    ++cnt;
  }
  sreader.closeScanFile();
  printf("SlamLauncher finished.\n");
}

//////// スキャン読み込み /////////

bool SlamLauncher::setFilename(const std::string &filename) {
  bool flag = sreader.openScanFile(filename);  // ファイルをオープン

  return (flag);
}

void SlamLauncher::customizeFramework() {
  fcustom.setSlamFrontEnd(&sfront);
  fcustom.makeFramework();
  fcustom.customizeI();

  pcmap = fcustom.getPointCloudMap();  // customizeの後にやること
}
