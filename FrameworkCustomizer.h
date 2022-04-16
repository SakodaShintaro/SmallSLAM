#ifndef FRAMEWORK_CUSTOMIZER_H_
#define FRAMEWORK_CUSTOMIZER_H_

#include <vector>

#include "CostFunction.h"
#include "CostFunctionED.h"
#include "CostFunctionPD.h"
#include "DataAssociator.h"
#include "DataAssociatorGT.h"
#include "DataAssociatorLS.h"
#include "LoopDetector.h"
#include "LoopDetectorSS.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "PointCloudMapBS.h"
#include "PointCloudMapGT.h"
#include "PointCloudMapLP.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"
#include "PoseOptimizer.h"
#include "PoseOptimizerSD.h"
#include "PoseOptimizerSL.h"
#include "RefScanMaker.h"
#include "RefScanMakerBS.h"
#include "RefScanMakerLM.h"
#include "ScanMatcher2D.h"
#include "ScanPointAnalyser.h"
#include "ScanPointResampler.h"
#include "SlamFrontEnd.h"

class FrameworkCustomizer {
  // フレームワーク改造用の部品
  RefScanMakerBS rsmBS;
  RefScanMakerLM rsmLM;
  DataAssociatorLS dassLS;
  DataAssociatorGT dassGT;
  CostFunctionED cfuncED;
  CostFunctionPD cfuncPD;
  PoseOptimizerSD poptSD;
  PoseOptimizerSL poptSL;
  PointCloudMapBS pcmapBS;
  PointCloudMapGT pcmapGT;
  PointCloudMapLP pcmapLP;
  PointCloudMap *pcmap;  // SlamLauncherで参照するためメンバ変数にする
  LoopDetector lpdDM;    // ダミー。何もしない
  LoopDetectorSS lpdSS;
  ScanPointResampler spres;
  ScanPointAnalyser spana;

  PoseEstimatorICP poest;
  PoseFuser pfu;
  ScanMatcher2D smat;
  SlamFrontEnd *sfront;

 public:
  FrameworkCustomizer() : pcmap(nullptr) {}

  ~FrameworkCustomizer() {}

  void setSlamFrontEnd(SlamFrontEnd *f) { sfront = f; }

  PointCloudMap *getPointCloudMap() { return (pcmap); }

  void makeFramework();
  void customizeA();
  void customizeB();
  void customizeC();
  void customizeD();
  void customizeE();
  void customizeF();
  void customizeG();
  void customizeH();
  void customizeI();
};

#endif
