#ifndef SENSOR_DATA_READER_H_
#define SENSOR_DATA_READER_H_

#include <fstream>
#include <iostream>
#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"

class SensorDataReader {
 public:
  SensorDataReader() : angleOffset(180) {}

  bool openScanFile(const std::string& filepath) {
    inFile.open(filepath);
    if (!inFile.is_open()) {
      std::cerr << "Error: cannot open file " << filepath << std::endl;
      return false;
    }
    return true;
  }

  void closeScanFile() { inFile.close(); }

  void setAngleOffset(int o) { angleOffset = o; }

  bool loadScan(size_t cnt, Scan2D &scan);
  bool loadLaserScan(size_t cnt, Scan2D &scan);

 private:
  int angleOffset;       // レーザスキャナとロボットの向きのオフセット
  std::ifstream inFile;  // データファイル
};

#endif
