﻿/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file LPoint2D.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef LPOINT2D_H_
#define LPOINT2D_H_
#include "eigensetting.h"
#include <Eigen/Core>

////////////////////////

struct Vector2D
{
  double x,y;
};

////////////////////////

enum ptype {UNKNOWN=0, LINE=1, CORNER=2, ISOLATE=3};    // 点のタイプ：未知、直線、コーナ、孤立

struct LPoint2D
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d pos;
  Eigen::Vector2d norm;
  int sid;                 // フレーム番号（スキャン番号）
  //  double x;                // 位置x
  //  double y;                // 位置y
  //  double nx;               // 法線ベクトル
  //  double ny;               // 法線ベクトル
  double atd;              // 累積走行距離(accumulated travel distance)
  ptype type;              // 点のタイプ

  LPoint2D() : sid(-1) {
    pos << 0,0;
    init();
  }

  LPoint2D(int id, double _x, double _y) {
    pos << _x,_y;
    init();
    sid = id;
  }

  LPoint2D(int id, Eigen::Vector2d _pos) {
    pos = _pos;
    init();
    sid = id;
  }

//////////

  void init() {
    sid = -1;
    atd =  0;
    type = UNKNOWN;
    norm << 0,0;
  }

  void setData(int id, double _x, double _y) {
    init();
    sid = id;
    pos << _x,_y;
  }

  void setData(int id, Eigen::Vector2d _pos) {
    init();
    sid = id;
    pos = _pos;
  }

  void setXY(double _x, double _y) {
    pos << _x,_y;
  }

  void setXY(Eigen::Vector2d _pos){
    pos = _pos;
  }
  // rangeとangleからxyを求める(右手系)
  void calXY(double range, double angle) {
    double a = DEG2RAD(angle);
    //x = range*cos(a);
    //y = range*sin(a);
    pos << range*cos(a), range*sin(a);
  }

  // rangeとangleからxyを求める(左手系）
  void calXYi(double range, double angle) {
    double a = DEG2RAD(angle);
    //x = range*cos(a);
    //y = -range*sin(a);
    pos << range*cos(a), -range*sin(a);
  }

  void setSid(int i) {
    sid = i;
  }

  void setAtd(double t) {
    atd = t;
  }

  void setType(ptype t) {
    type = t;
  }

  void setNormal(double x, double y) {
    norm << x,y;
  }
  void setNormal(Eigen::Vector2d n){
    norm = n;
  }

  
};

#endif
