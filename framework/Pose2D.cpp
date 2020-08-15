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
 * @file Pose2D.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "Pose2D.h"

// Moddification for Eigen is required

// グローバル座標系での点pを、自分（Pose2D）の局所座標系に変換
LPoint2D Pose2D::relativePoint(const LPoint2D &p) const {
  //  double dx = p.x - tx;
  //  double dy = p.y - ty;
  //  double dx = p.pos(0) - trans(0);
  //  double dy = p.pos(1) - trans(1);
  Eigen::Vector2d d = p.pos-trans;

  //  double x = dx*Rmat[0][0] + dy*Rmat[1][0];  // 回転の逆行列
  //  double y = dx*Rmat[0][1] + dy*Rmat[1][1];
  //  double x = dx*Rmat(0,0) + dy*Rmat(1,0);  // 回転の逆行列
  //  double y = dx*Rmat(0,1) + dy*Rmat(1,1);
  Eigen::Vector2d res = Rmat.transpose()*d;
  
  return LPoint2D(p.sid, res);
}

////////

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換
LPoint2D Pose2D::globalPoint(const LPoint2D &p) const {
  Eigen::Vector2d res = Rmat*p.pos + trans;
  return LPoint2D(p.sid, res);
}

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換してpoに入れる
void Pose2D::globalPoint(const LPoint2D &pi, LPoint2D &po) const {
  po.pos = Rmat*pi.pos+trans;
}

///////

// 基準座標系bposeから見た現座標系nposeの相対位置relPoseを求める（Inverse compounding operator）
void Pose2D::calRelativePose(const Pose2D &npose, const Pose2D &bpose, Pose2D &relPose) {
  const Eigen::Matrix2d R0 = bpose.Rmat;           // 基準座標系
  const Eigen::Matrix2d R1 = npose.Rmat;           // 現座標系
  Eigen::Matrix2d       R2 = relPose.Rmat;         // 相対位置

  // 並進
  Eigen::Vector2d d = npose.trans-bpose.trans;
  relPose.trans = R0.transpose()*d;

  // 回転
  double th = npose.th - bpose.th;
  if (th < -180)
    th += 360;
  else if (th >= 180)
    th -= 360;
  relPose.th = th;

  relPose.calRmat();
}

// 基準座標系bposeから相対位置relPoseだけ進んだ、座標系nposeを求める（Compounding operator）
void Pose2D::calGlobalPose(const Pose2D &relPose, const Pose2D &bpose, Pose2D &npose) {
  const Eigen::Matrix2d R0 = bpose.Rmat;           // 基準座標系
  const Eigen::Matrix2d R1 = relPose.Rmat;         // 相対位置
  Eigen::Matrix2d       R2 = npose.Rmat;           // 新座標系

  // 並進
  npose.trans = R0*relPose.trans + bpose.trans;

  // 角度
  double th = bpose.th + relPose.th;
  if (th < -180)
    th += 360;
  else if (th >= 180)
    th -= 360;
  npose.th = th;

  npose.calRmat();
}
