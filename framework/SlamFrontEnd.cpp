/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file SlamFrontEnd.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SlamFrontEnd.h"
#include "debug.h" 

using namespace std;

/////////

// 初期化
void SlamFrontEnd::init() {
  smat->reset();
  smat->setPointCloudMap(pcmap);
  sback.setPointCloudMap(pcmap);
}

///////////

// 現在スキャンscanを処理する。
void SlamFrontEnd::process(Scan2D &scan, PerformanceMonitor &pmon) {
  if (cnt == 0) 
    init();                                       // 開始時に初期化

  // スキャンマッチング
  pmon.resumeTimer("matchScan");
  smat->matchScan(scan);
  pmon.stopTimer("matchScan");
  
  Pose2D curPose = pcmap->getLastPose();          // これはスキャンマッチングで推定した現在のロボット位置
  
  // ポーズグラフにオドメトリアークを追加
  pmon.resumeTimer("makeOdometryArc");
  if (cnt == 0) {                                 // 最初はノードを置くだけ。
    pg->addNode(curPose);
  }
  else {                                          // 次からはノードを追加して、オドメトリアークを張る
    Eigen::Matrix3d &cov = smat->getCovariance();
    makeOdometryArc(curPose, cov);
  }
  pmon.stopTimer("makeOdometryArc");

  pmon.resumeTimer("makeGlobalMap");
  if (cnt%keyframeSkip==0) {                             // キーフレームのときだけ行う
    if (cnt == 0)
      pcmap->setNthre(1);                                // cnt=0のときは地図が小さいのでサンプリング多くする
    else
      pcmap->setNthre(5);
    pcmap->makeGlobalMap();                              // 点群地図の全体地図を生成
  }
  pmon.stopTimer("makeGlobalMap");
  
  // ループ閉じ込み
  pmon.resumeTimer("detectLoop");
  if (cnt > keyframeSkip && cnt%keyframeSkip==0) {       // キーフレームのときだけ行う
    bool flag = lpd->detectLoop(&scan, curPose, cnt);    // ループ検出を起動
    if (flag) {
      sback.adjustPoses();                               // ループが見つかったらポーズ調整
      sback.remakeMaps();                                // 地図やポーズグラフの修正
    }
  }
  pmon.stopTimer("detectLoop");
  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "pcmap.size={}", pcmap->globalMap.size());   // 確認用
  //countLoopArcs();            // 確認用
  //  pmon.writeToFile();
  ++cnt;
}

////////////

// オドメトリアークの生成
bool SlamFrontEnd::makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &fusedCov) {
  if (pg->nodes.size() == 0)                             // 念のためのチェック
    return(false);
  PoseNode *lastNode = pg->nodes.back();                 // 直前ノード
  PoseNode *curNode = pg->addNode(curPose);              // ポーズグラフに現在ノードを追加

  // 直前ノードと現在ノードの間にオドメトリアークを張る
  Pose2D &lastPose = lastNode->pose;
  Pose2D relPose;
  Pose2D::calRelativePose(curPose, lastPose, relPose);   // 現在位置と直前位置の相対位置（移動量）の計算

  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "sfront: lastPose:  tx={}, ty={}, th={}", lastPose.tx, lastPose.ty, lastPose.th);

  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(lastPose, fusedCov, cov, true);     // 移動量の共分散に変換
  PoseArc *arc = pg->makeArc(lastNode->nid, curNode->nid, relPose, cov);     // アークの生成
  pg->addArc(arc);                                                           // ポーズグラフにアークを追加

  return(true);
}

////////////

// ループアーク数を数える。確認用
void SlamFrontEnd::countLoopArcs() {
  vector<PoseArc*> &parcs = pg->arcs;       // 全ポーズアーク
  int an=0;                                 // ループアーク数
  for (size_t i=0; i<parcs.size(); i++) {
    PoseArc *a = parcs[i];
    PoseNode *src = a->src;
    PoseNode *dst = a->dst;
    if (src->nid != dst->nid-1)             // オドメトリアークは始点と終点が連番になっている
      ++an;                                 // オドメトリアークでなければループアーク
  }
  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "loopArcs.size={}", an);         // 確認用
}
