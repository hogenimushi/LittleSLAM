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
 * @file CostFunctionPD.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "CostFunctionPD.h"

using namespace std;

// 垂直距離によるコスト関数
double CostFunctionPD::calValue(double tx, double ty, double th) {
  double a = DEG2RAD(th);

  double error=0;
  int pn=0;
  int nn=0;
  for (size_t i=0; i<curLps.size(); i++) {
    const LPoint2D *clp = curLps[i];             // 現在スキャンの点
    const LPoint2D *rlp = refLps[i];             // clpに対応する参照スキャンの点

    if (rlp->type != LINE)                       // 直線上の点でなければ使わない
      continue;
    Eigen::Vector2d cp = clp->pos;
    Eigen::Matrix2d mat;
    mat << cos(a), -sin(a), sin(a), cos(a);
    Eigen::Vector2d p  = mat*cp + Eigen::Vector2d(tx,ty);      // clpを参照スキャンの座標系に変換
    double pdis = (p - rlp->pos).dot(rlp->norm);        // 垂直距離
    
    double er = pdis*pdis;
    if (er <= evlimit*evlimit)
      ++pn;                                      // 誤差が小さい点の数

    error += er;                                 // 各点の誤差を累積
    ++nn;
  }

  error = (nn>0)? error/nn : HUGE_VAL;           // 有効点数が0なら、値はHUGE_VAL
  pnrate = 1.0*pn/nn;                            // 誤差が小さい点の比率

//  printf("CostFunctionPD: error=%g, pnrate=%g, evlimit=%g\n", error, pnrate, evlimit);     // 確認用

  error *= 100;                                  // 評価値が小さくなりすぎないよう100かける。

  return(error);
}
