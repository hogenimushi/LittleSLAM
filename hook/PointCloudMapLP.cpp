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
 * @file PointCloudMapLP.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PointCloudMapLP.h"
#include "NNGridTable.h"
#include "debug.h"

using namespace std;

double PointCloudMapLP::atdThre = 10;

///////////

// 格子テーブルを用いて、部分地図の代表点を得る
vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> Submap::subsamplePoints(int nthre) {
  NNGridTable nntab;                     // 格子テーブル
  for (size_t i=0; i<mps.size(); i++) {
    LPoint2D &lp = mps[i];
    nntab.addPoint(&lp);                 // 全点を登録
  }

  vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> sps;
  nntab.makeCellPoints(nthre, sps);      // nthre個以上のセルの代表点をspsに入れる

  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "mps.size={}, sps.size={}", mps.size(), sps.size());

  return(sps);
}

/////////

// ロボット位置の追加
void PointCloudMapLP::addPose(const Pose2D &p) {
  // 累積走行距離(atd)の計算
  if (poses.size() > 0) {
    Pose2D pp = poses.back();
    atd += (p.trans - pp.trans).norm();
  }
  else {
    atd += p.trans.norm();
  }

  poses.emplace_back(p);
}

// スキャン点の追加
void PointCloudMapLP::addPoints(const vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &lps) {
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  if (atd - curSubmap.atdS >= atdThre ) {          // 累積走行距離が閾値を超えたら新しい部分地図に変える
    size_t size = poses.size();
    curSubmap.cntE = size-1;                       // 部分地図の最後のスキャン番号
    curSubmap.mps = curSubmap.subsamplePoints(nthre); // 終了した部分地図は代表点のみにする（軽量化）

    Submap submap(atd, size);                      // 新しい部分地図
    submap.addPoints(lps);                         // スキャン点群の登録
    submaps.emplace_back(submap);                  // 部分地図を追加
  }
  else {
    curSubmap.addPoints(lps);                      // 現在の部分地図に点群を追加
  }
}

// 全体地図の生成。局所地図もここでいっしょに作った方が速い
void PointCloudMapLP::makeGlobalMap(){
  globalMap.clear();                               // 初期化
  localMap.clear();
  // 現在以外のすでに確定した部分地図から点を集める
  for (size_t i=0; i<submaps.size()-1; i++) {
    Submap &submap = submaps[i];                   // 部分地図
    vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &mps = submap.mps;            // 部分地図の点群。代表点だけになっている
    for (size_t j=0; j<mps.size(); j++) {
      globalMap.emplace_back(mps[j]);              // 全体地図には全点入れる
    }
    if (i == submaps.size()-2) {                   // 局所地図には最後の部分地図だけ入れる
      for (size_t j=0; j<mps.size(); j++) {
        localMap.emplace_back(mps[j]);
      }
    }
  }

  // 現在の部分地図の代表点を全体地図と局所地図に入れる
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> sps = curSubmap.subsamplePoints(nthre);  // 代表点を得る
  for (size_t i=0; i<sps.size(); i++) {
    globalMap.emplace_back(sps[i]);
    localMap.emplace_back(sps[i]);
  }

  // 以下は確認用
  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "curSubmap.atd={}, atd={}, sps.size={}", curSubmap.atdS, atd, sps.size());
  SPDLOG_LOGGER_DEBUG(logger, "submaps.size={}, globalMap.size={}", submaps.size(), globalMap.size());
}

// 局所地図の生成
void PointCloudMapLP::makeLocalMap(){
  localMap.clear();                                // 初期化
  if (submaps.size() >= 2) {
    Submap &submap = submaps[submaps.size()-2];    // 直前の部分地図だけ使う
    vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &mps = submap.mps;            // 部分地図の点群。代表点だけになっている
    for (size_t i=0; i<mps.size(); i++) {
      localMap.emplace_back(mps[i]);
    }
  }

  // 現在の部分地図の代表点を局所地図に入れる
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> sps = curSubmap.subsamplePoints(nthre);  // 代表点を得る
  for (size_t i=0; i<sps.size(); i++) {
    localMap.emplace_back(sps[i]);
  }
  auto logger = spdlog::get("slamlogger");
  SPDLOG_LOGGER_DEBUG(logger, "localMap.size={}", localMap.size());   // 確認用
}

//////////

// ポーズ調整後のロボット軌跡newPoseを用いて、地図を再構築する
void PointCloudMapLP::remakeMaps(const vector<Pose2D,Eigen::aligned_allocator<Pose2D>> &newPoses){
  // 各部分地図内の点の位置を修正する
  for (size_t i=0; i<submaps.size(); i++) {
    Submap &submap = submaps[i];
    vector<LPoint2D,Eigen::aligned_allocator<LPoint2D>> &mps = submap.mps;                // 部分地図の点群。現在地図以外は代表点になっている
    for (size_t j=0; j<mps.size(); j++) {
      LPoint2D &mp = mps[j];
      size_t idx = mp.sid;                             // 点のスキャン番号
      if (idx >= poses.size()) {                       // 不正なスキャン番号（あったらバグ）
        continue;
      }

      const Pose2D &oldPose = poses[idx];              // mpに対応する古いロボット位置
      const Pose2D &newPose = newPoses[idx];           // mpに対応する新しいロボット位置
      const Eigen::Matrix2d R1 = oldPose.Rmat;
      const Eigen::Matrix2d R2 = newPose.Rmat;
      LPoint2D lp1 = oldPose.relativePoint(mp);        // oldPoseでmpをセンサ座標系に変換
      LPoint2D lp2 = newPose.globalPoint(lp1);         // newPoseでポーズ調整後の地図座標系に変換
      mp.pos = lp2.pos;
      // 法線ベクトルもoldPoseでセンサ座標系に変換
      // 法線ベクトルもnewPoseでポーズ調整後の地図座標系に変換
      Eigen::Vector2d nnorm = R2*R1.transpose()*mp.norm;
      
      mp.setNormal(nnorm);
    }
  }

  makeGlobalMap();                                     // 部分地図から全体地図と局所地図を生成

  for (size_t i=0; i<poses.size(); i++) {              // posesをポーズ調整後の値に更新
    poses[i] = newPoses[i];
  }
  lastPose = newPoses.back();
}
