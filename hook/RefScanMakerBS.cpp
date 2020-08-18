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
 * @file RefScanMakerBS.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "RefScanMakerBS.h"

using namespace std;

const Scan2D *RefScanMakerBS::makeRefScan() {
  vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &refLps = refScan.lps;         // 参照スキャンの点群のコンテナ
  refLps.clear();

  Pose2D lastPose = pcmap->getLastPose();         // 点群地図に保存した最後の推定位置
  Eigen::Matrix2d R = lastPose.Rmat;

  Eigen::Vector2d trans = lastPose.trans;
  //  double tx = lastPose.trans(0);
  //  double ty = lastPose.trans(1);

  // 点群地図に保存した最後のスキャンを参照スキャンにする
  const vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &lps = pcmap->lastScan.lps;
  for (size_t i=0; i<lps.size(); i++) {
    const LPoint2D &mp = lps[i];                  // 参照スキャンの点

    // スキャンはロボット座標系なので、地図座標系に変換
    LPoint2D rp;
    //    rp.pos(0) = R(0,0)*mp.pos(0) + R(0,1)*mp.pos(1) + tx;      // 点の位置
    //    rp.pos(1) = R(1,0)*mp.pos(0) + R(1,1)*mp.pos(1) + ty;
    //    rp.norm(0)= R(0,0)*mp.norm(0)+ R(0,1)*mp.norm(1);        // 法線ベクトル
    //    rp.norm(1)= R(1,0)*mp.norm(0)+ R(1,1)*mp.norm(1);
    rp.pos = R*mp.pos + trans;
    rp.norm = R*mp.norm;
    refLps.emplace_back(rp);
  }

  return(&refScan);
}
