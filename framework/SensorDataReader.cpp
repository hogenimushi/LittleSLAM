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
 * @file SensorDataReader.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SensorDataReader.h"

using namespace std;

// ファイルからスキャンを1個読む
bool SensorDataReader::loadScan(size_t cnt, Scan2D &scan) {
  bool isScan=false;
  while (!(scanres==Eof) && !isScan) {     // スキャンを読むまで続ける
    isScan = loadLaserScan(cnt, scan);
  }

  if (isScan) 
    return(false);                       // まだファイルが続くという意味
  else
    return(true);                        // ファイルが終わったという意味
}

//////////////

// ファイルから項目1個を読む。読んだ項目がスキャンならtrueを返す。
bool SensorDataReader::loadLaserScan(size_t cnt, Scan2D &scan) {
  char type[256];                           // ファイル内の項目ラベル
  fscanf(inFile, "%s", type);
  if (std::strcmp(type,"LASERSCAN")==0) {             // スキャンの場合
    scan.setSid(cnt);

    int sid, sec, nsec;
    //    inFile >> sid >> sec >> nsec;        // これらは使わない
    fscanf(inFile,"%d %d %d",&sid, &sec, &nsec);
    
    vector<LPoint2D> lps;
    int pnum;                            // スキャン点数
    fscanf(inFile,"%d",& pnum);
    lps.reserve(pnum);
    for (int i=0; i<pnum; i++) {
      float angle, range;
      fscanf(inFile,"%f %f",&angle,&range);
      //inFile >> angle >> range;          // スキャン点の方位と距離
      angle += angleOffset;              // レーザスキャナの方向オフセットを考慮
      if (range <= Scan2D::MIN_SCAN_RANGE || range >= Scan2D::MAX_SCAN_RANGE) {
//      if (range <= Scan2D::MIN_SCAN_RANGE || range >= 3.5) {         // わざと退化を起こしやすく
        continue;
      }

      LPoint2D lp;
      lp.setSid(cnt);                    // スキャン番号はcnt（通し番号）にする
      lp.calXY(range, angle);            // angle,rangeから点の位置xyを計算
      lps.emplace_back(lp);
    }
    scan.setLps(lps);

    // スキャンに対応するオドメトリ情報
    Pose2D &pose = scan.pose;
    double tx, ty;
    fscanf(inFile,"%lf %lf",&tx, &ty);
    scan.pose.trans << tx,ty;
    double th;
    scanres = fscanf(inFile,"%lf",&th);
    pose.setAngle(RAD2DEG(th));          // オドメトリ角度はラジアンなので度にする
    pose.calRmat();

    return(true);
  }
  else {                                 // スキャン以外の場合

    while(scanres!=Eof){
      scanres=fgetc(inFile);
      if((char)scanres=='\n') break;
      if(scanres==Eof) break;
    }
    //    string line;
    //    getline(inFile, line);               // 読み飛ばす

    return(false);
  }
}
