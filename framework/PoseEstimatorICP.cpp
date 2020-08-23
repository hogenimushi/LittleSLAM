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
 * @file PoseEstimatorICP.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include <boost/timer.hpp>
#include "PoseEstimatorICP.h"
#include "debug.h"

using namespace std;

//////////////

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimatorICP::estimatePose(Pose2D &initPose, Pose2D &estPose){
  boost::timer tim;

  double evmin = HUGE_VAL;             // コスト最小値。初期値は大きく
  double evthre = 0.000001;            // コスト変化閾値。変化量がこれ以下なら繰り返し終了
//  double evthre = 0.00000001;            // コスト変化閾値。変化量がこれ以下なら繰り返し終了
  popt->setEvthre(evthre);
  popt->setEvlimit(0.2);               // evlimitは外れ値の閾値[m]

  double ev = 0;                       // コスト
  double evold = evmin;                // 1つ前の値。収束判定のために使う。
  Pose2D pose = initPose;
  Pose2D poseMin = initPose;
  for (int i=0; abs(evold-ev) > evthre && i<100; i++) {           // i<100は振動対策
    if (i > 0)
      evold = ev;
    double mratio = dass->findCorrespondence(curScan, pose);      // データ対応づけ
    Pose2D newPose;
    popt->setPoints(dass->lps);                  // 対応結果を渡す
    ev = popt->optimizePose(pose, newPose);                       // その対応づけにおいてロボット位置の最適化
    pose = newPose;

    if (ev < evmin) {                                             // コスト最小結果を保存
      poseMin = newPose;
      evmin = ev;
    }

//    printf("dass.curLps.size=%lu, dass.refLps.size=%lu\n", dass->curLps.size(), dass->refLps.size());
//    printf("mratio=%g\n", mratio);
//    printf("i=%d: ev=%g, evold=%g\n", i, ev, evold);
  }

  pnrate = popt->getPnrate();
  usedNum = dass->lps.size();

  estPose = poseMin;


  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "finalError={}, pnrate={}", evmin, pnrate);
  SPDLOG_LOGGER_DEBUG(logger, "estPose:  tx={}, ty={}, th={}", pose.tx, pose.ty, pose.th);      // 確認用

  double t1 = 1000*tim.elapsed();
  SPDLOG_LOGGER_DEBUG(logger, "PoseEstimatorICP: t1={}", t1);                 // 処理時間

  if (evmin < HUGE_VAL)
    totalError += evmin;                                   // 誤差合計
  totalTime += t1;                                         // 処理時間合計
  SPDLOG_LOGGER_DEBUG(logger, "totalError={}, totalTime={}", totalError, totalTime);    // 確認用

  return(evmin);
}
