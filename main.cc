#include <iostream>

#include "SlamLauncher.h"

int main() {
  std::cout << "Small SLAM" << std::endl;
  const bool scanCheck = false;                               // スキャン表示のみか
  const bool odometryOnly = false;                            // オドメトリによる地図構築か
  const std::string filename = "../../dataset/corridor.lsc";  // データファイル名
  int startN = 0;                                             // 開始スキャン番号

  printf("SlamLauncher: startN=%d, scanCheck=%d, odometryOnly=%d\n", startN, scanCheck, odometryOnly);
  std::cout << "filename = " << filename << std::endl;

  // ファイルを開く
  SlamLauncher sl;
  bool flag = sl.setFilename(filename);
  if (!flag) {
    return 1;
  }

  sl.setStartN(startN);  // 開始スキャン番号の設定

  // 処理本体
  if (scanCheck) {
    sl.showScans();
  } else {  // スキャン表示以外はSlamLauncher内で場合分け
    sl.setOdometryOnly(odometryOnly);
    sl.customizeFramework();
    sl.run();
  }
}
