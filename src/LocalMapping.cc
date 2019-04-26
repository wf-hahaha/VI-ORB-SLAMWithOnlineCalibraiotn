/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include <unistd.h>
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>
#include "IMU/configparam.h"
#include "Converter.h"
#define PI (3.1415926535897932346f)

namespace ORB_SLAM2
{
  //syl***add for Tbc change
    cv::Mat Tbc_true;
    cv::Mat Rbc_true;
    cv::Mat pbc_true;
    cv::Mat Rcb_true;
    cv::Mat pcb_true;
  //syl***add  end
//syl***add for online calibration
//含VI外餐标定的单目视觉惯性联合初始化
bool LocalMapping::TryInitVIOWithoutPreCalibration(void)
{
  
  if(mpMap->KeyFramesInMap()<=mnLocalWindowSize)
        return false;
  
  static bool fopened = false;
  static ofstream fgw,fscale,fbiasa,ftime1,ftime2,ftime3,fbiasg,fpcb,feul,finf1,finf2,finf3;
  static bool YPRFinish=false,PcbScaleFinish=false;
  static bool firstInit=true;
  float thr1=0.1;  //用于判断VI旋转角是否收敛
  //float thr1=0.5;
  float thr2=0.01; //用于判断VI偏移角是否收敛
  //float thr2=0.3;
  float thr3=0.02; //用于判断尺度因子scale是否收敛  
  //float thr3=0.4;
  int Nwin=10;    //用于判断旋转角,偏移角和尺度因子是否收敛的窗口
  int Nwin2=40;  //添加窗口的大小
  
  if(!fopened)
    {
        // Need to modify this to correct path
        string tmpfilepath = ConfigParam::getTmpFilePath();
        fgw.open(tmpfilepath+"gw.txt");
        fscale.open(tmpfilepath+"scale.txt");
        fbiasa.open(tmpfilepath+"biasa.txt");
	ftime1.open(tmpfilepath+"time1.txt");
	ftime2.open(tmpfilepath+"time2.txt");
	ftime3.open(tmpfilepath+"time3.txt");
        fbiasg.open(tmpfilepath+"biasg.txt");
	fpcb.open(tmpfilepath+"pcb.txt");
	feul.open(tmpfilepath+"eul.txt");	
	finf1.open(tmpfilepath+"finf1.txt");
	finf2.open(tmpfilepath+"finf2.txt");
	finf3.open(tmpfilepath+"finf3.txt");
        if(fgw.is_open() && fscale.is_open() && fbiasa.is_open()
	  && ftime1.is_open()&& ftime2.is_open()&& ftime3.is_open()&& fbiasg.is_open()
	  &&fpcb.is_open()&& feul.is_open()
	  &&finf1.is_open()&&finf2.is_open()&& finf3.is_open())
            fopened = true;
        else
        {
            cerr<<"file open error in TryInitVIO"<<endl;
            fopened = false;
        }
        fgw<<std::fixed<<std::setprecision(6);
        fscale<<std::fixed<<std::setprecision(6);
        fbiasa<<std::fixed<<std::setprecision(6);
        ftime1<<std::fixed<<std::setprecision(6);
	ftime2<<std::fixed<<std::setprecision(6);
	ftime3<<std::fixed<<std::setprecision(6);
	fbiasg<<std::fixed<<std::setprecision(6);
        fpcb<<std::fixed<<std::setprecision(6);
	feul<<std::fixed<<std::setprecision(6);
	finf1<<std::fixed<<std::setprecision(6);
	finf2<<std::fixed<<std::setprecision(6);
	finf3<<std::fixed<<std::setprecision(6);	
    }
    
    // Use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();

    
    //syl***add for init fised sliding window
    
 /*   if(N>Nwin2)
    {
      vScaleGravityKF.erase(vScaleGravityKF.begin(), vScaleGravityKF.begin()+N-Nwin2);
    }
    
    N = vScaleGravityKF.size();
    cout<<"N=:"<<N<<endl;*/
    
    //待标定和初始化的参数
    static Vector3d bgest(0,0,0);
    
    
    
    //VI旋转角标定
  //  if(YPRFinish==false)
    if(firstInit==true)
    {
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      YPRFinish=VIRotationCalibrationWithInfoWin(vScaleGravityKF,Nwin,thr1,feul,finf1,Nwin2);
   //   YPRFinish=VIRotationCalibration(vScaleGravityKF,Nwin,thr1,feul,finf1);
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      ftime1<<t<<endl;
      firstInit=false;
    }
    
    
    //bg初始化
    bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF);
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(bgest);
    }
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();
    } 
    
    {
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      YPRFinish=VIRotationCalibrationWithInfoWin(vScaleGravityKF,Nwin,thr1,feul,finf1,Nwin2);
   //   YPRFinish=VIRotationCalibration(vScaleGravityKF,Nwin,thr1,feul,finf1);
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      ftime1<<t<<endl;
      cout<<"11111111"<<endl;
    }
    
    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
     
    
    
    static double sstar;
    static cv::Mat gwstar;
    static cv::Mat pcb_c;
    
    static double s_;
    static cv::Mat gwafter_;
    static cv::Mat dbiasa_;
    
    cv::Mat Tbc = ConfigParam::GetMatTbc();
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
    cv::Mat Rcb = Rbc.t();
    
    //VI偏移量标定和初始化    
 //   if(PcbScaleFinish==false)
    {
      //假设ba=0,求解scale，gw,pcb
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      ScaleGwPcbApproximationWithInfoWin(vScaleGravityKF,Rcb,sstar,gwstar,pcb_c,finf2,Nwin2);
    //  ScaleGwPcbApproximation(vScaleGravityKF,Rcb,sstar,gwstar,pcb_c,finf2);
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      ftime2<<t<<endl;
      cout<<"222222222"<<endl;
    
      //引入重力大小的约束，求解scale,gw,ba,pcb_refined
      t1 = std::chrono::steady_clock::now();
      PcbScaleFinish=ScaleGwBaPcbRefineWithInfoWin(vScaleGravityKF,gwstar,Rcb,s_,gwafter_,dbiasa_,Nwin,thr2,thr3,fpcb,finf3,Nwin2);
     // PcbScaleFinish=ScaleGwBaPcbRefine(vScaleGravityKF,gwstar,Rcb,s_,gwafter_,dbiasa_,Nwin,thr2,thr3,fpcb,finf3);
      t2 = std::chrono::steady_clock::now();
      t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      ftime3<<t<<endl; 
      cout<<"333333333"<<endl;
    }
    
        // Debug log
    {

        cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;

        fgw<<mpCurrentKeyFrame->mTimeStamp<<"\t"
           <<gwafter_.at<float>(0)<<"\t"<<gwafter_.at<float>(1)<<"\t"<<gwafter_.at<float>(2)<<"\t"<<endl;
        fscale<<mpCurrentKeyFrame->mTimeStamp<<"\t"
              <<s_<<"\t"<<sstar<<"\t"<<endl;
        fbiasa<<mpCurrentKeyFrame->mTimeStamp<<"\t"
              <<dbiasa_.at<float>(0)<<"\t"<<dbiasa_.at<float>(1)<<"\t"<<dbiasa_.at<float>(2)<<"\t"<<endl;
     //   fcondnum<<w2.at<float>(1)/w2.at<float>(5)<<endl;
        //        ftime<<mpCurrentKeyFrame->mTimeStamp<<"\t"
        //             <<(t3-t0)/cv::getTickFrequency()*1000<<"\t"<<endl;
        fbiasg<<mpCurrentKeyFrame->mTimeStamp<<"\t"
              <<bgest(0)<<"\t"<<bgest(1)<<"\t"<<bgest(2)<<"\t"<<endl;
    }

    bool bVIOInited = false;
    if(mbFirstTry)
    {
        mbFirstTry = false;
        mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }
    if(YPRFinish==true&&PcbScaleFinish==true/*&&mpCurrentKeyFrame->mTimeStamp - mnStartTime >= 15.0*/)
    {
        bVIOInited = true;
	SetMapUpdateFlagInTracking(true);
	//cout<<"set map updated in tracking!"<<endl;
    }
    
       if(!bVIOInited)
    {
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->SetNavStateBiasGyr(Vector3d::Zero());
            pKF->SetNavStateBiasAcc(Vector3d::Zero());
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
        }
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }
    else//计算各个关键帧的速度
    {
              // Set NavState , scale and bias for all KeyFrames
        // Scale
        double scale = s_;
        mnVINSInitScale = s_;
        // gravity vector in world frame
        mGravityVec = gwafter_;
        cv::Mat Tbc = ConfigParam::GetMatTbc();
        cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
        cv::Mat pbc = Tbc.rowRange(0,3).col(3);
        cv::Mat Rcb = Rbc.t();
        cv::Mat pcb = -Rcb*pbc;
	Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
        Vector3d gweig = Converter::toVector3d(gwafter_);

        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            // Position and rotation of visual SLAM
            cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
            cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
            // Set position and rotation of navstate
            cv::Mat wPb = scale*wPc + Rwc*pcb;
            pKF->SetNavStatePos(Converter::toVector3d(wPb));
            pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcb));
            // Update bias of Gyr & Acc
            pKF->SetNavStateBiasGyr(bgest);
            pKF->SetNavStateBiasAcc(dbiasa_eig);
            // Set delta_bias to zero. (only updated during optimization)
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
            // Step 4.
            // compute velocity
            if(pKF != vScaleGravityKF.back())
            {
                KeyFrame* pKFnext = pKF->GetNextKeyFrame();
                // IMU pre-int between pKF ~ pKFnext
                const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                // Time from this(pKF) to next(pKFnext)
                double dt = imupreint.getDeltaTime();                                       // deltaTime
                cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
                cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
                cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);           // wPc next
                cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);    // Rwc next

                cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*dbiasa_) + 0.5*gwafter_*dt*dt );
                Eigen::Vector3d veleig = Converter::toVector3d(vel);
                pKF->SetNavStateVel(veleig);
            }
            else
            {
                // If this is the last KeyFrame, no 'next' KeyFrame exists
                KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
                const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
                double dt = imupreint_prev_cur.getDeltaTime();
                Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                //
                Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*dbiasa_eig );
                pKF->SetNavStateVel(veleig);
            }
        }
        
        // Re-compute IMU pre-integration at last.
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }
    return bVIOInited;
     
}

bool LocalMapping::VIRotationCalibrationWithInfoWin(vector<KeyFrame*> vScaleGravityKF,int Nwin,float thr,ofstream &feul,ofstream &finf1,int Nwin2)
{
  int N = vScaleGravityKF.size();
  cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
  cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);

  
  cv::Mat Qlbi = cv::Mat::zeros(4,4,CV_32F);
  cv::Mat Qrci = cv::Mat::zeros(4,4,CV_32F);  
  cv::Mat Qi = cv::Mat::zeros(4,4,CV_32F);
//  cv::Mat Q=cv::Mat::zeros(4*(N-1),4,CV_32F);
  static Eigen::Matrix3d Rbc_cEigen;
  static cv::Mat Rbc_c=cv::Mat::eye(3,3,CV_32F); 
  static cv::Mat qbc_c=cv::Mat::zeros(4,1,CV_32F);
  //static bool init=1;
  
  //for informative window
   cv::Mat Q=cv::Mat::zeros(4*Nwin2,4,CV_32F);
   cv::Mat Qfinal=cv::Mat::zeros(4*(N-1),4,CV_32F);   //固定用于计算的Q的维度，避免计算量随时间线性增长
   float *INFO=new float[Nwin2];
   for(int i=0;i<Nwin2;i++)
   {
     INFO[i]=0;
   }
   // float INFO[20]={0};//存储与Q对应的info，按info从大到小排序
  //end  
  
  
  float Sb;
  float xb;
  float yb;
  float zb;
  float Sc;
  float xc;
  float yc;
  float zc;
  static Eigen::Vector3d eulc;
  static Eigen::Vector3d eult;
  
  for(int i=0;i<N-1;i++)
    {
      KeyFrame* pKF1 = vScaleGravityKF[i];
      KeyFrame* pKF2 = vScaleGravityKF[i+1];
      cv::Mat Twc1 = pKF1->GetPoseInverse();
      cv::Mat Twc2 = pKF2->GetPoseInverse();
      cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
      cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
      
      cv::Mat dRc=Rc1.inv()*Rc2;
   //   cout<<"N=:"<<N<<endl;           
   //   cout<<"Rc2"<<Rc2<<endl;
   //   cout<<"Rc1"<<Rc1<<endl;
   //   cout<<"dRc"<<dRc<<endl;
          
      Eigen::Matrix3d dRcEigen=Converter::toMatrix3d(dRc);
   //   cout<<"eulrc=:"<<eulrc<<endl;
      
      Eigen::Quaterniond qc = Eigen::Quaterniond ( dRcEigen );
      Sc=qc.w();
      xc=qc.x();
      yc=qc.y();
      zc=qc.z();
      Eigen::Vector3d VcEigen(xc,yc,zc);
      cv::Mat Vc=Converter::toCvMat(VcEigen);
 //     cout<<"RC1=:"<<Rc1<<endl<<"RC2=:"<<Rc2<<endl<<"dRC=:"<<dRc<<endl;  
 //     cout<<"qc=:"<<qc.coeffs()<<endl;
 //     cout<<"Sc=:"<<Sc<<endl;
 //     cout<<"Vc=:"<<Vc<<endl;
            
      cv::Mat dRb = (Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaR()));//!!!!这里只是相邻关键帧之间IMU的相对旋转角，如果间隔较多的关键帧需要进行修改
      
      Eigen::Matrix3d dRbEigen=Converter::toMatrix3d(dRb);
      Eigen::Quaterniond qb = Eigen::Quaterniond ( dRbEigen );//四元数存储的顺序是[w x y z]
      Sb=qb.w();
      xb=qb.x();
      yb=qb.y();
      zb=qb.z();
      Eigen::Vector3d VbEigen(xb,yb,zb);
      cv::Mat Vb=Converter::toCvMat(VbEigen);   
      
      cv::Mat Ql11=Sb*I3+Converter::toCvMat(Sophus::SO3::hat(VbEigen));
      cv::Mat Ql12=Vb;
      cv::Mat Ql21=-Vb.t();
      cv::Mat Ql22=Sb*I1;
      
      Ql11.copyTo(Qlbi.rowRange(0,3).colRange(0,3));
      Ql12.copyTo(Qlbi.rowRange(0,3).col(3));
      Ql21.copyTo(Qlbi.row(3).colRange(0,3));
      Ql22.copyTo(Qlbi.row(3).col(3));
            
      cv::Mat Qr11=Sc*I3-Converter::toCvMat(Sophus::SO3::hat(VcEigen));
      cv::Mat Qr12=Vc;
      cv::Mat Qr21=-Vc.t();
      cv::Mat Qr22=Sc*I1;
      
      Qr11.copyTo(Qrci.rowRange(0,3).colRange(0,3));
      Qr12.copyTo(Qrci.rowRange(0,3).col(3));
      Qr21.copyTo(Qrci.row(3).colRange(0,3));
      Qr22.copyTo(Qrci.row(3).col(3));   
      
      Qi=Qlbi-Qrci;
      
      //for informative window
      {
	cv::Mat Ji=-Qi;
  //    cout<<"Ji"<<Ji<<endl;
        Eigen::Matrix<double,4,4> IMiEigen=Converter::toMatrix4d(Ji.t()*Ji);
  //    cout<<"IMiEigen"<<IMiEigen<<endl;
        float info=IMiEigen.trace();
  //    cout<<"info"<<info<<endl;
      if(i==0)  //第一个数直接放在第一个位置
      {
	  INFO[0]=info;
	  Qi.copyTo(Q.rowRange(4*(0-0)+0,4*(0-0)+4).colRange(0,4));
	//  cout<<"i"<<i<<endl;
      }
      if(i==1)  //第二个数仅通过与第一个数比较判断放置位置
      {  
	if(info<INFO[0]) 
	{
	  INFO[1]=INFO[0];
	  (Q.rowRange(4*(0-0)+0,4*(0-0)+4).colRange(0,4)).copyTo(Q.rowRange(4*(1-0)+0,4*(1-0)+4).colRange(0,4));
	  INFO[0]=info;
	  Qi.copyTo(Q.rowRange(4*(0-0)+0,4*(0-0)+4).colRange(0,4));
	  
	}
	else 
	{
	  INFO[1]=info;
	  Qi.copyTo(Q.rowRange(4*(1-0)+0,4*(1-0)+4).colRange(0,4));
	}
	//cout<<"i"<<i<<endl;
      } 
      if(i<Nwin2&&i>1)  //如果没有达到窗口限制，则将当前数按照info放入正确位置
      {
	 if(info<=INFO[0])  //判断是否小于最小值
	  {
	    for(int n=i;n>0;n--)
	    {
	      INFO[n]=INFO[n-1]; //将INFO值依次右移
	      (Q.rowRange(4*(n-1-0)+0,4*(n-1-0)+4).colRange(0,4)).copyTo(Q.rowRange(4*(n-0)+0,4*(n-0)+4).colRange(0,4));
	    }
	    INFO[0]=info;
	    Qi.copyTo(Q.rowRange(4*(0-0)+0,4*(0-0)+4).colRange(0,4));
	  }
	  
	  else if(info>=INFO[i-1]) //判断是否大于最大值
	  {
	    INFO[i]=info;
	    Qi.copyTo(Q.rowRange(4*(i-0)+0,4*(i-0)+4).colRange(0,4));
	  }
	
	 else 
	 {
	   for(int index=0;index<i-1;index++)
	   {  
	     if(info>=INFO[index]&&info<INFO[index+1])//查找插入的位置index+1
	    {
	      for(int n=i;n>index+1;n--)
	      {
	        INFO[n]=INFO[n-1]; //将INFO值依次下移
	        (Q.rowRange(4*(n-1-0)+0,4*(n-1-0)+4).colRange(0,4)).copyTo(Q.rowRange(4*(n-0)+0,4*(n-0)+4).colRange(0,4));
	      }
	      INFO[index+1]=info;
	      Qi.copyTo(Q.rowRange(4*(index+1-0)+0,4*(index+1-0)+4).colRange(0,4));
	      break;
	    }
	   }
	 }//else end 
	// cout<<"i"<<i<<endl;
	}//if end
	
	if(i>=Nwin2&&info>INFO[0])//只对当前info大于最小值的情况进行操作
	{
	//  cout<<"i=:"<<i<<endl;
	    if(info>INFO[Nwin2-1]) //如果当前info大于最大值
	    {
	      for(int n=0;n<Nwin2-1;n++)
	      {
		INFO[n]=INFO[n+1];
		(Q.rowRange(4*(n+1-0)+0,4*(n+1-0)+4).colRange(0,4)).copyTo(Q.rowRange(4*(n-0)+0,4*(n-0)+4).colRange(0,4));
	      }
	      INFO[Nwin2-1]=info;
	      Qi.copyTo(Q.rowRange(4*(Nwin2-1-0)+0,4*(Nwin2-1-0)+4).colRange(0,4));
	    }
	    
	    else for(int index=0;index<Nwin2-1;index++)//如果当前info并非大于最大值，则寻找插入位置
	   {  
	     if(info>INFO[index]&&info<=INFO[index+1])//查找插入的位置index
	    {
	      for(int n=0;n<index;n++)
	      {
	        INFO[n]=INFO[n+1];
		Qi.copyTo(Q.rowRange(4*(n+1-0)+0,4*(n+1-0)+4).colRange(0,4));
	      }
	        INFO[index]=info;
		Qi.copyTo(Q.rowRange(4*(index-0)+0,4*(index-0)+4).colRange(0,4));
	        break;
	    }
	   }//else end
	}//if end
      }
   //   cout<<"11"<<endl;
    //  Qi.copyTo(Q.rowRange(4*(i-0)+0,4*(i-0)+4).colRange(0,4));
  //    cout<<"IMi"<<IMi<<endl;  //各个相邻两帧之间估计的协方差矩阵，等于信息矩阵的逆，。
   //   double inf=cv::determinant(IMi)<<endl;
  //    cout<<"infi:"<<cv::determinant(IMi)<<endl;
  //    cout<<"information count "<<cv::determinant(IMi)<<endl;
   //   finf1<<cv::determinant(IMi)<<endl;//值越小，包含的信息量越大
      
  //    cout<<"Q=:"<<Q<<endl;
    }
    for(int i=0;i<Nwin2;i++)
    {
        cout<<INFO[i]<<endl;
    }
    delete[] INFO;
 //   cout<<"1!"<<endl;
       cv::Mat J;
       if(N<=Nwin2)
      {
	(Q.rowRange(4*(0-0)+0,4*(N-2-0)+4).colRange(0,4)).copyTo(Qfinal.rowRange(4*(0-0)+0,4*(N-2-0)+4).colRange(0,4));
	J=-Qfinal;
	//cout<<"Qfinal=:"<<endl<<Qfinal<<endl;
      }
      else
      {
	J=-Q;
      }
   //   cout<<"J"<<J<<endl;
      cv::Mat IM=(J.t()*J);
   //   cout<<"IM"<<IM<<endl;
   //   cout<<"information count "<<cv::determinant(IM)<<endl;
      Eigen::Matrix<double,4,4> IMEigen=Converter::toMatrix4d(IM);
      finf1<<IMEigen.trace()<<endl;  
      cout<<"12"<<endl;
  //    fQ<<Q<<endl;
  //    fq<<mpCurrentKeyFrame->mTimeStamp-mnStartTime<<"\t"
  //      <<xb<<"\t"<<yb<<"\t"<<zb<<"\t"<<Sb<<"\t"
//	<<xc<<"\t"<<yc<<"\t"<<zc<<"\t"<<Sc<<endl;	
      //用svd解Q*qbc=0;
      //Q=u1*w1*vt1
      cv::Mat w1,u1,vt1;
    //  cv::SVDecomp(Q,w1,u1,vt1,cv::SVD::FULL_UV);
      if(N<=Nwin2)
      {
	cv::SVDecomp(Qfinal,w1,u1,vt1,cv::SVD::MODIFY_A);
      }
      else
      {
	cv::SVDecomp(Q,w1,u1,vt1,cv::SVD::MODIFY_A);
      }
      cout<<"13"<<endl;
   //   cout<<"u1=:"<<u1<<endl;
   //   cout<<"w1=:"<<w1<<endl;      
   //   cout<<"vt1=:"<<vt1<<endl;     

      vt1=vt1.t();
  //    cout<<"vt1=:"<<vt1<<endl;
      qbc_c=vt1.col(3);     //[x,y,z,w]
      
      Eigen::Quaterniond qbc;
      qbc.x()=qbc_c.at<float>(0,0);
      qbc.y()=qbc_c.at<float>(1,0);
      qbc.z()=qbc_c.at<float>(2,0);
      qbc.w()=qbc_c.at<float>(3,0);
      Rbc_cEigen=Eigen::Matrix3d(qbc);
      
      Rbc_c=Converter::toCvMat(Rbc_cEigen);
   //   cout<<"qcb=:"<<qcb.coeffs()<<endl;
      
  //    cout<<"Rbc_c=:"<<endl<<Rbc_c<<endl;
      
       eulc= (Rbc_cEigen.eulerAngles(2,1,0))*180/PI;  
       cout<<"eulc=:"<<endl<<eulc<<endl;
       
       eult=(Converter::toMatrix3d(Rbc_true)).eulerAngles(2,1,0)*180/PI;
       cout<<"eult=:"<<endl<<eult<<endl;
//       cout<<"Rcb_cEigen=:"<<endl<<Rcb_cEigen<<endl;  
    // Extrinsics

 //   cout<<"Rcb before change=:"<<Rcb<<endl;
    
    ConfigParam::SetRbc(Rbc_cEigen);
    feul<<eulc(0,0)<<"\t"<<eulc(1,0)<<"\t"<<eulc(2,0)<<endl;
    bool YPRFinish=false;
    YPRFinish=CheckVIRotationConvergence(Nwin,thr);
    
    return YPRFinish;
    
  
}
bool LocalMapping::ScaleGwPcbApproximationWithInfoWin(vector<KeyFrame*> vScaleGravityKF,cv::Mat Rcb,double &sstar,cv::Mat &gwstar,cv::Mat pcb_c,ofstream &finf2,int Nwin2)
{
    int N = vScaleGravityKF.size();
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);
    
     // Solve A*x=B for x=[s,gw,pcb] 7x1 vector
 //   cv::Mat A = cv::Mat::zeros(3*(N-2),7,CV_32F);
 //   cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    cv::Mat A = cv::Mat::zeros(3*Nwin2,7,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*Nwin2,1,CV_32F);
    cv::Mat Ai= cv::Mat::zeros(3,7,CV_32F);
    cv::Mat Bi= cv::Mat::zeros(3,1,CV_32F);
    cv::Mat Afinal=cv::Mat::zeros(3*(N-2),7,CV_32F);
    cv::Mat Bfinal=cv::Mat::zeros(3*(N-2),1,CV_32F);
    float *INFO2=new float[Nwin2];
    for(int i=0;i<Nwin2;i++)
   {
     INFO2[i]=0;
   }
    //float INFO2[20]={0};//存储与Q对应的info，按info从大到小排序

    // Step 2.
    // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp) cerr<<"dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;
        if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 - (pc3-pc2)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
	cv::Mat phi = (Rc3-Rc2)*dt12-(Rc2-Rc1)*dt23;
        cv::Mat gamma =Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
    //    lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
    //    beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
//	phi.copyTo(A.rowRange(3*i+0,3*i+3).colRange(4,7));
    //    gamma.copyTo(B.rowRange(3*i+0,3*i+3));
	
	{
	lambda.copyTo(Ai.rowRange(0,3).col(0));
        beta.copyTo(Ai.rowRange(0,3).colRange(1,4));
	phi.copyTo(Ai.rowRange(0,3).colRange(4,7));
        gamma.copyTo(Bi.rowRange(0,3));
	cv::Mat Ji2=-Ai;
	//cv::Mat IMi=(Ji2.t()*Ji2).inv();
	//cout<<"IMi=:"<<endl<<IMi<<endl;
	Eigen::Matrix<double,7,7> IMiEigen2=Converter::toMatrix7d(Ji2.t()*Ji2);
	float info2=IMiEigen2.trace();
	//float info2=IMiEigen2.trace();
	//cout<<"info2=:"<<info2<<endl;
	if(i==0)  //第一个数直接放在第一个位置
      {
	  INFO2[0]=info2;
	  
	  
	  Ai.copyTo(A.rowRange(3*0+0,3*0+3).colRange(0,7));
	//  cout<<"A=:"<<A<<endl;
	//  cout<<"Bi=:"<<Bi<<endl;
	//  cout<<"B=:"<<B<<endl;
	  Bi.copyTo(B.rowRange(3*0+0,3*0+3));
	//  cout<<"B=:"<<B<<endl;
//	  cout<<"i"<<i<<endl;
      }
      if(i==1)  //第二个数仅通过与第一个数比较判断放置位置
      {  
	if(info2<INFO2[0]) 
	{
	  INFO2[1]=INFO2[0];
	  (A.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,7)).copyTo(A.rowRange(3*(1-0)+0,3*(1-0)+3).colRange(0,7));
	  (B.rowRange(3*(0-0)+0,3*(0-0)+3)).copyTo(B.rowRange(3*(1-0)+0,3*(1-0)+3));
	  INFO2[0]=info2;
	  Ai.copyTo(A.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,7));
	  Bi.copyTo(B.rowRange(3*(0-0)+0,3*(0-0)+3));
	  
	}
	else 
	{
	  INFO2[1]=info2;
	  Ai.copyTo(A.rowRange(3*(1-0)+0,3*(1-0)+3).colRange(0,7));
	  Bi.copyTo(B.rowRange(3*(1-0)+0,3*(1-0)+3));
	}
//	cout<<"i"<<i<<endl;
      } 
      if(i<Nwin2&&i>1)  //如果没有达到窗口限制，则将当前数按照info放入正确位置
      {
	 if(info2<=INFO2[0])  //判断是否小于最小值
	  {
	//    cout<<"21>"<<endl;
	    for(int n=i;n>0;n--)
	    {
	      INFO2[n]=INFO2[n-1]; //将INFO值依次右移
	      (A.rowRange(3*(n-1-0)+0,3*(n-1-0)+3).colRange(0,7)).copyTo(A.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,7));
	      (B.rowRange(3*(n-1-0)+0,3*(n-1-0)+3)).copyTo(B.rowRange(3*(n-0)+0,3*(n-0)+3));
	    }
	    INFO2[0]=info2;
	    Ai.copyTo(A.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,7));
	    Bi.copyTo(B.rowRange(3*(0-0)+0,3*(0-0)+3));

	  }
	  
	  else if(info2>=INFO2[i-1]) //判断是否大于最大值
	  {
	//    cout<<"21<"<<endl;
	    INFO2[i]=info2;
	    Ai.copyTo(A.rowRange(3*(i-0)+0,3*(i-0)+3).colRange(0,7));
	    Bi.copyTo(B.rowRange(3*(i-0)+0,3*(i-0)+3));

	  }
	
	 else 
	 {
	//   cout<<"21><"<<endl;
	   for(int index=0;index<i-1;index++)
	   {  
	     if(info2>=INFO2[index]&&info2<INFO2[index+1])//查找插入的位置index+1
	    {
	      for(int n=i;n>index+1;n--)
	      {
	        INFO2[n]=INFO2[n-1]; //将INFO值依次下移
	        (A.rowRange(3*(n-1-0)+0,3*(n-1-0)+3).colRange(0,7)).copyTo(A.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,7));
		(B.rowRange(3*(n-1-0)+0,3*(n-1-0)+3)).copyTo(B.rowRange(3*(n-0)+0,3*(n-0)+3));
	      }
	      INFO2[index+1]=info2;
	      Ai.copyTo(A.rowRange(3*(index+1-0)+0,3*(index+1-0)+3).colRange(0,7));
	      Bi.copyTo(B.rowRange(3*(index+1-0)+0,3*(index+1-0)+3));
	      break;
	    }
	   }
	 }//else end 
//	 cout<<"i"<<i<<endl;
	}//if end
	
	if(i>=Nwin2&&info2>INFO2[0])//只对当前info大于最小值的情况进行操作
	{
	 // cout<<"i=:"<<i<<endl;

	    if(info2>INFO2[Nwin2-1]) //如果当前info大于最大值
	    {
	//      cout<<"22<"<<endl;
	      for(int n=0;n<Nwin2-1;n++)
	      {
		INFO2[n]=INFO2[n+1];
		(A.rowRange(3*(n+1-0)+0,3*(n+1-0)+3).colRange(0,7)).copyTo(A.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,7));
		(B.rowRange(3*(n+1-0)+0,3*(n+1-0)+3)).copyTo(B.rowRange(3*(n-0)+0,3*(n-0)+3));
	      }
	      INFO2[Nwin2-1]=info2;
	//      cout<<"hi"<<endl;
	      Ai.copyTo(A.rowRange(3*(Nwin2-1-0)+0,3*(Nwin2-1-0)+3).colRange(0,7));
	      Bi.copyTo(B.rowRange(3*(Nwin2-1-0)+0,3*(Nwin2-1-0)+3));
	    }
	    
	    else for(int index=0;index<Nwin2-1;index++)//如果当前info并非大于最大值，则寻找插入位置
	   {  
	//     cout<<"22><"<<endl;
	     if(info2>INFO2[index]&&info2<=INFO2[index+1])//查找插入的位置index
	    {
	      for(int n=0;n<index;n++)
	      {
	        INFO2[n]=INFO2[n+1];
		Ai.copyTo(A.rowRange(3*(n+1-0)+0,3*(n+1-0)+3).colRange(0,7));
		Bi.copyTo(B.rowRange(3*(n+1-0)+0,3*(n+1-0)+3));
	      }
	        INFO2[index]=info2;
		Ai.copyTo(A.rowRange(3*(index-0)+0,3*(index-0)+3).colRange(0,7));
		Bi.copyTo(B.rowRange(3*(index-0)+0,3*(index-0)+3));
	        break;
	    }
	   }//else end
	}//if end
	}
	
 //  cout<<"21"<<endl;
    }
    
    for(int i=0;i<Nwin2;i++)
    {
       cout<<INFO2[i]<<endl;
    }
    delete [] INFO2;
 //   cout<<"2!"<<endl;
    cv::Mat J2=-A;
    if(N<=Nwin2+1)
      {
	(A.rowRange(3*(0-0)+0,3*(N-3-0)+3).colRange(0,7)).copyTo(Afinal.rowRange(3*(0-0)+0,3*(N-3-0)+3).colRange(0,7));
	(B.rowRange(3*(0-0)+0,3*(N-3-0)+3)).copyTo(Bfinal.rowRange(3*(0-0)+0,3*(N-3-0)+3));
	cv::Mat J2=-Afinal;
//	cout<<"Afinal=:"<<endl<<Afinal<<endl;
//	cout<<"Bfinal=:"<<endl<<Bfinal<<endl;
      }
    cv::Mat IM2=(J2.t()*J2);
    Eigen::Matrix<double,7,7> IMEigen2=Converter::toMatrix7d(IM2);
    finf2<<IMEigen2.trace()<<endl;
 //   cout<<"2222222"<<endl;
    
    cv::Mat w,u,vt;
    // Note w is 7x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    if(N<=Nwin2+1)
    {
      cv::SVDecomp(Afinal,w,u,vt,cv::SVD::MODIFY_A);
    }
    else
    {
      cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
    }
 //   cout<<"2222223"<<endl;
    // Debug log
    //cout<<"u:"<<endl<<u<<endl;
    //cout<<"vt:"<<endl<<vt<<endl;
    //cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv=cv::Mat::eye(7,7,CV_32F);
    for(int i=0;i<7;i++)
    {
        if(fabs(w.at<float>(i))<1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
        }

        winv.at<float>(i,i) = 1./w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x;
  if(N<=Nwin2+1)
    {
       x= vt.t()*winv*u.t()*Bfinal;
    }
    else
    {
       x = vt.t()*winv*u.t()*B;
    }

    // x=[s,gw] 4x1 vector
    sstar = x.at<float>(0);    // scale should be positive
    gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
    pcb_c = x.rowRange(4,7);    
  //  cout<<"pcb_c=:"<<pcb_c<<endl;
  //  cout<<"pcb_t=:"<<pcb_true<<endl;
  //  fpcb<<pcb_cafter.at<float>(0,0)<<"\t"<<pcb_cafter.at<float>(1,0)<<"\t"<<pcb_cafter.at<float>(2,0)<<endl;
  //  <<pcb_true.at<float>(0,0)<<"\t"<<pcb_true.at<float>(1,0)<<"\t"<<pcb_true.at<float>(2,0)<<endl;
    //cout<<"scale sstar: "<<sstar<<endl;
    //cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;
    // Test log
    if(w.type()!=I3.type() || u.type()!=I3.type() || vt.type()!=I3.type())
        cerr<<"different mat type, I3,w,u,vt: "<<I3.type()<<","<<w.type()<<","<<u.type()<<","<<vt.type()<<endl;
    
    return true;
    
}
bool LocalMapping::ScaleGwBaPcbRefineWithInfoWin(vector<KeyFrame*> vScaleGravityKF,cv::Mat gwstar,cv::Mat Rcb,double &s_,cv::Mat &gwafter_,cv::Mat &dbiasa_,int Nwin,float thr2,float thr3,ofstream &fpcb,ofstream &finf3,int Nwin3)
{
    int N = vScaleGravityKF.size();
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);
 
    // Use gravity magnitude 9.8 as constraint
    // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    // Debug log
    //cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    // Debug log
    //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI*ConfigParam::GetG();//9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba,pcb] (1+2+3)x1 vector
//    cv::Mat C = cv::Mat::zeros(3*(N-2),9,CV_32F);
//    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);
    
    cv::Mat C = cv::Mat::zeros(3*Nwin3,9,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*Nwin3,1,CV_32F);
    cv::Mat Ci= cv::Mat::zeros(3,9,CV_32F);
    cv::Mat Di= cv::Mat::zeros(3,1,CV_32F);
    cv::Mat Cfinal=cv::Mat::zeros(3*(N-2),9,CV_32F);
    cv::Mat Dfinal=cv::Mat::zeros(3*(N-2),1,CV_32F);
    float *INFO3=new float[Nwin3];
    //float INFO3[20]={0};
   for(int i=0;i<Nwin3;i++)
   {
     INFO3[i]=0;
   }
    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
	cv::Mat gama = (Rc2-Rc1)*dt23-(Rc3-Rc2)*dt12;
       // cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
       //              - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        cv::Mat psi = Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23);
//	lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
//        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
//        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
//	gama.copyTo(C.rowRange(3*i+0,3*i+3).colRange(6,9));
 //       psi.copyTo(D.rowRange(3*i+0,3*i+3));
	
	{
		
	lambda.copyTo(Ci.rowRange(0,3).col(0));
        phi.colRange(0,2).copyTo(Ci.rowRange(0,3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(Ci.rowRange(0,3).colRange(3,6));
	gama.copyTo(Ci.rowRange(0,3).colRange(6,9));
        psi.copyTo(Di.rowRange(0,3));
	
	cv::Mat Ji3=-Ci;
//	cv::Mat IMi=(Ji3.t()*Ji3).inv();
//	cout<<"IMi=:"<<endl<<IMi<<endl;
	Eigen::Matrix<double,9,9> IMiEigen3=Converter::toMatrix9d(Ji3.t()*Ji3);
	//cout<<"IMiEigen3=:"<<endl<<IMiEigen3<<endl;
	float info3=IMiEigen3.trace();
	//float info3=cv::determinant(IMi);
//	cout<<"info3=:"<<info3<<endl;
	
	if(i==0)  //第一个数直接放在第一个位置
      {
	  INFO3[0]=info3;  
	  Ci.copyTo(C.rowRange(3*0+0,3*0+3).colRange(0,9));
	  Di.copyTo(D.rowRange(3*0+0,3*0+3));
//	  cout<<"i"<<i<<endl;
      }
      if(i==1)  //第二个数仅通过与第一个数比较判断放置位置
      {  
	if(info3<INFO3[0]) 
	{
	  INFO3[1]=INFO3[0];
	  (C.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,9)).copyTo(C.rowRange(3*(1-0)+0,3*(1-0)+3).colRange(0,9));
	  (D.rowRange(3*(0-0)+0,3*(0-0)+3)).copyTo(D.rowRange(3*(1-0)+0,3*(1-0)+3));
	  INFO3[0]=info3;
	  Ci.copyTo(C.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,9));
	  Di.copyTo(D.rowRange(3*(0-0)+0,3*(0-0)+3));
	  
	}
	else 
	{
	  INFO3[1]=info3;
	  Ci.copyTo(C.rowRange(3*(1-0)+0,3*(1-0)+3).colRange(0,9));
	  Di.copyTo(D.rowRange(3*(1-0)+0,3*(1-0)+3));
	}
//	cout<<"i"<<i<<endl;
      } 
      if(i<Nwin3&&i>1)  //如果没有达到窗口限制，则将当前数按照info放入正确位置
      {
	 if(info3<=INFO3[0])  //判断是否小于最小值
	  {
	    for(int n=i;n>0;n--)
	    {
	      INFO3[n]=INFO3[n-1]; //将INFO值依次右移
	      (C.rowRange(3*(n-1-0)+0,3*(n-1-0)+3).colRange(0,9)).copyTo(C.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,9));
	      (D.rowRange(3*(n-1-0)+0,3*(n-1-0)+3)).copyTo(D.rowRange(3*(n-0)+0,3*(n-0)+3));
	    }
	    INFO3[0]=info3;
	    Ci.copyTo(C.rowRange(3*(0-0)+0,3*(0-0)+3).colRange(0,9));
	    Di.copyTo(D.rowRange(3*(0-0)+0,3*(0-0)+3));
	  }
	  
	  else if(info3>=INFO3[i-1]) //判断是否大于最大值
	  {
	    INFO3[i]=info3;
	    Ci.copyTo(C.rowRange(3*(i-0)+0,3*(i-0)+3).colRange(0,9));
	    Di.copyTo(D.rowRange(3*(i-0)+0,3*(i-0)+3));
	  }
	
	 else 
	 {
	   for(int index=0;index<i-1;index++)
	   {  
	     if(info3>=INFO3[index]&&info3<INFO3[index+1])//查找插入的位置index+1
	    {
	      for(int n=i;n>index+1;n--)
	      {
	        INFO3[n]=INFO3[n-1]; //将INFO值依次下移
	        (C.rowRange(3*(n-1-0)+0,3*(n-1-0)+3).colRange(0,9)).copyTo(C.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,9));
		(D.rowRange(3*(n-1-0)+0,3*(n-1-0)+3)).copyTo(D.rowRange(3*(n-0)+0,3*(n-0)+3));
	      }
	      INFO3[index+1]=info3;
	      Ci.copyTo(C.rowRange(3*(index+1-0)+0,3*(index+1-0)+3).colRange(0,9));
	      Di.copyTo(D.rowRange(3*(index+1-0)+0,3*(index+1-0)+3));
	      break;
	    }
	   }
	 }//else end 
//	 cout<<"i"<<i<<endl;
	}//if end
	
	if(i>=Nwin3&&info3>INFO3[0])//只对当前info大于最小值的情况进行操作
	{
	 // cout<<"i=:"<<i<<endl;
	    if(info3>INFO3[Nwin3-1]) //如果当前info大于最大值
	    {
	      for(int n=0;n<Nwin3-1;n++)
	      {
		INFO3[n]=INFO3[n+1];
		(C.rowRange(3*(n+1-0)+0,3*(n+1-0)+3).colRange(0,9)).copyTo(C.rowRange(3*(n-0)+0,3*(n-0)+3).colRange(0,9));
		(D.rowRange(3*(n+1-0)+0,3*(n+1-0)+3)).copyTo(D.rowRange(3*(n-0)+0,3*(n-0)+3));
	      }
	      INFO3[Nwin3-1]=info3;
	      Ci.copyTo(C.rowRange(3*(Nwin3-1-0)+0,3*(Nwin3-1-0)+3).colRange(0,9));
	      Di.copyTo(D.rowRange(3*(Nwin3-1-0)+0,3*(Nwin3-1-0)+3));
	    }
	    
	    else for(int index=0;index<Nwin3-1;index++)//如果当前info并非小于最小值，则寻找插入位置
	   {  
	     if(info3>INFO3[index]&&info3<=INFO3[index+1])//查找插入的位置index
	    {
	      for(int n=0;n<index;n++)
	      {
	        INFO3[n]=INFO3[n+1];
		Ci.copyTo(C.rowRange(3*(n+1-0)+0,3*(n+1-0)+3).colRange(0,9));
		Di.copyTo(D.rowRange(3*(n+1-0)+0,3*(n+1-0)+3));
	      }
	        INFO3[index]=info3;
		Ci.copyTo(C.rowRange(3*(index-0)+0,3*(index-0)+3).colRange(0,9));
		Di.copyTo(D.rowRange(3*(index-0)+0,3*(index-0)+3));
	        break;
	    }
	   }//else end
	}//if end*/
	}

 //   cout<<"31"<<endl;
        // Debug log
        //cout<<"iter "<<i<<endl;
    }
    	for(int i=0;i<Nwin3;i++)
	{
	  cout<<INFO3[i]<<endl;
	}
    delete [] INFO3;
  //  cout<<"3!"<<endl;
    cv::Mat J3=-C;
    if(N<=Nwin3+1)
      {
	(C.rowRange(3*(0-0)+0,3*(N-3-0)+3).colRange(0,9)).copyTo(Cfinal.rowRange(3*(0-0)+0,3*(N-3-0)+3).colRange(0,9));
	(D.rowRange(3*(0-0)+0,3*(N-3-0)+3)).copyTo(Dfinal.rowRange(3*(0-0)+0,3*(N-3-0)+3));
	cv::Mat J3=-Cfinal;
//	cout<<"Afinal=:"<<endl<<Afinal<<endl;
//	cout<<"Bfinal=:"<<endl<<Bfinal<<endl;
      }
    cv::Mat IM3=(J3.t()*J3);
    Eigen::Matrix<double,9,9> IMEigen3=Converter::toMatrix9d(IM3);
    finf3<<IMEigen3.trace()<<endl;
  //  cout<<"32"<<endl;
    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2,u2,vt2;
     if(N<=Nwin3+1)
    {
      cv::SVDecomp(Cfinal,w2,u2,vt2,cv::SVD::MODIFY_A);
    }
    else
    {
      cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    }
    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(9,9,CV_32F);
    for(int i=0;i<9;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }

        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
 //   cout<<"33"<<endl;
    // Then y = vt'*winv*u'*D
    cv::Mat y;
    if(N<=Nwin3+1)
    {
      y = vt2.t()*w2inv*u2.t()*Dfinal;
    }
    else
    {
      y = vt2.t()*w2inv*u2.t()*D;
    }

    s_ = y.at<float>(0);
    cv::Mat dthetaxy = y.rowRange(1,3);
    dbiasa_ = y.rowRange(3,6);
    Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
    cv::Mat pcb_cafter = y.rowRange(6,9);
    cout<<"pcb_cafter=:"<<pcb_cafter<<endl;
    cout<<"pcb_true=:"<<pcb_true<<endl;
    ConfigParam::SetPcb(pcb_cafter);       //update pcb
    fpcb<<pcb_cafter.at<float>(0,0)<<"\t"<<pcb_cafter.at<float>(1,0)<<"\t"<<pcb_cafter.at<float>(2,0)<<endl;
    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);
    gwafter_=Rwi_*GI;
    
    //check convergence for pcb and scale
    bool PcbFinish=false;
    bool ScaleFinish=false;
    PcbFinish=CheckPcbConvergence(Nwin,thr2);
    ScaleFinish=CheckScaleConvergence(Nwin,thr3);
    return (PcbFinish&&ScaleFinish);
  //  return PcbFinish;
    
    
}

bool LocalMapping::VIRotationCalibration(vector<KeyFrame*> vScaleGravityKF,int Nwin,float thr,ofstream &feul,ofstream &finf1)
{
  int N = vScaleGravityKF.size();
  cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
  cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);

  
  cv::Mat Qlbi = cv::Mat::zeros(4,4,CV_32F);
  cv::Mat Qrci = cv::Mat::zeros(4,4,CV_32F);  
  cv::Mat Qi = cv::Mat::zeros(4,4,CV_32F);
  cv::Mat Q=cv::Mat::zeros(4*(N-1),4,CV_32F);
  static Eigen::Matrix3d Rbc_cEigen;
  static cv::Mat Rbc_c=cv::Mat::eye(3,3,CV_32F); 
  static cv::Mat qbc_c=cv::Mat::zeros(4,1,CV_32F);
  //static bool init=1;
  
  
  float Sb;
  float xb;
  float yb;
  float zb;
  float Sc;
  float xc;
  float yc;
  float zc;
  static Eigen::Vector3d eulc;
  static Eigen::Vector3d eult;
  
  for(int i=0;i<N-1;i++)
    {
      KeyFrame* pKF1 = vScaleGravityKF[i];
      KeyFrame* pKF2 = vScaleGravityKF[i+1];
      cv::Mat Twc1 = pKF1->GetPoseInverse();
      cv::Mat Twc2 = pKF2->GetPoseInverse();
      cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
      cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
      cv::Mat dRc=Rc1.inv()*Rc2;
   //   cout<<"N=:"<<N<<endl;           
   //   cout<<"Rc2"<<Rc2<<endl;
   //   cout<<"Rc1"<<Rc1<<endl;
   //   cout<<"dRc"<<dRc<<endl;
          
      Eigen::Matrix3d dRcEigen=Converter::toMatrix3d(dRc);
   //   cout<<"eulrc=:"<<eulrc<<endl;
      cv::Mat pc1=Twc1.rowRange(0,3).col(3);
   //   cout<<"pc1"<<endl<<pc1<<endl;
      cv::Mat pc2=Twc2.rowRange(0,3).col(3);
   //   cout<<"pc2"<<endl<<pc2<<endl;
      cv::Mat dpc=pc2-pc1;
      cout<<"dpc"<<endl<<dpc<<endl;
      
      
      Eigen::Quaterniond qc = Eigen::Quaterniond ( dRcEigen );
      Sc=qc.w();
      xc=qc.x();
      yc=qc.y();
      zc=qc.z();
      Eigen::Vector3d VcEigen(xc,yc,zc);
      cv::Mat Vc=Converter::toCvMat(VcEigen);
 //     cout<<"RC1=:"<<Rc1<<endl<<"RC2=:"<<Rc2<<endl<<"dRC=:"<<dRc<<endl;  
 //     cout<<"qc=:"<<qc.coeffs()<<endl;
 //     cout<<"Sc=:"<<Sc<<endl;
 //     cout<<"Vc=:"<<Vc<<endl;
            
      cv::Mat dRb = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaR());//!!!!这里只是相邻关键帧之间IMU的相对旋转角，如果间隔较多的关键帧需要进行修改
      
      cv::Mat dpb = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
      cv::Mat sdpc=dpb-(Rc2-Rc1)*pcb_true;
      cout<<"sdpc"<<endl<<dpb<<endl;
      cout<<"s"<<endl<<sdpc.at<float>(0)/dpc.at<float>(0)<<"\t"<<sdpc.at<float>(1)/dpc.at<float>(1)<<"\t"<<sdpc.at<float>(2)/dpc.at<float>(2)<<endl;
      
      
      
      
      Eigen::Matrix3d dRbEigen=Converter::toMatrix3d(dRb);
      Eigen::Quaterniond qb = Eigen::Quaterniond ( dRbEigen );//四元数存储的顺序是[w x y z]
      Sb=qb.w();
      xb=qb.x();
      yb=qb.y();
      zb=qb.z();
      Eigen::Vector3d VbEigen(xb,yb,zb);
      cv::Mat Vb=Converter::toCvMat(VbEigen);   
      
      cv::Mat Ql11=Sb*I3+Converter::toCvMat(Sophus::SO3::hat(VbEigen));
      cv::Mat Ql12=Vb;
      cv::Mat Ql21=-Vb.t();
      cv::Mat Ql22=Sb*I1;
      
      Ql11.copyTo(Qlbi.rowRange(0,3).colRange(0,3));
      Ql12.copyTo(Qlbi.rowRange(0,3).col(3));
      Ql21.copyTo(Qlbi.row(3).colRange(0,3));
      Ql22.copyTo(Qlbi.row(3).col(3));
            
      cv::Mat Qr11=Sc*I3-Converter::toCvMat(Sophus::SO3::hat(VcEigen));
      cv::Mat Qr12=Vc;
      cv::Mat Qr21=-Vc.t();
      cv::Mat Qr22=Sc*I1;
      
      Qr11.copyTo(Qrci.rowRange(0,3).colRange(0,3));
      Qr12.copyTo(Qrci.rowRange(0,3).col(3));
      Qr21.copyTo(Qrci.row(3).colRange(0,3));
      Qr22.copyTo(Qrci.row(3).col(3));   
      
      Qi=Qlbi-Qrci;
      
      Qi.copyTo(Q.rowRange(4*(i-0)+0,4*(i-0)+4).colRange(0,4));
  //    cout<<"Q=:"<<Q<<endl;
    }
 
      cv::Mat J=-Q;
   //   cout<<"J"<<J<<endl;
      cv::Mat IM=J.t()*J;
      Eigen::Matrix<double,4,4> IMEigen=Converter::toMatrix4d(IM);
   //   cout<<"IM"<<IM<<endl;
   //   cout<<"information count "<<cv::determinant(IM)<<endl;
      finf1<<IMEigen.trace()<<endl;  
  //    cout<<"12"<<endl;
      //用svd解Q*qbc=0;
      //Q=u1*w1*vt1
      cv::Mat w1,u1,vt1;
      cv::SVDecomp(Q,w1,u1,vt1,cv::SVD::MODIFY_A);
   //   cout<<"13"<<endl;   
      vt1=vt1.t();
      qbc_c=vt1.col(3);     //[x,y,z,w]
      
      Eigen::Quaterniond qbc;
      qbc.x()=qbc_c.at<float>(0,0);
      qbc.y()=qbc_c.at<float>(1,0);
      qbc.z()=qbc_c.at<float>(2,0);
      qbc.w()=qbc_c.at<float>(3,0);
      Rbc_cEigen=Eigen::Matrix3d(qbc);
      
      Rbc_c=Converter::toCvMat(Rbc_cEigen);
   //   cout<<"qcb=:"<<qcb.coeffs()<<endl;
      
  //    cout<<"Rbc_c=:"<<endl<<Rbc_c<<endl;
      
       eulc= (Rbc_cEigen.eulerAngles(2,1,0))*180/PI;  
       cout<<"eulc=:"<<endl<<eulc<<endl;
       
       eult=(Converter::toMatrix3d(Rbc_true)).eulerAngles(2,1,0)*180/PI;
       cout<<"eult=:"<<endl<<eult<<endl;
//       cout<<"Rcb_cEigen=:"<<endl<<Rcb_cEigen<<endl;  
    // Extrinsics   
    ConfigParam::SetRbc(Rbc_cEigen);
    feul<<eulc(0,0)<<"\t"<<eulc(1,0)<<"\t"<<eulc(2,0)<<endl;
    bool YPRFinish=false;
    YPRFinish=CheckVIRotationConvergence(Nwin,thr);
    
    return YPRFinish;
    
  
}
bool LocalMapping::ScaleGwPcbApproximation(vector<KeyFrame*> vScaleGravityKF,cv::Mat Rcb,double &sstar,cv::Mat &gwstar,cv::Mat pcb_c,ofstream &finf2)
{
    int N = vScaleGravityKF.size();
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);
    
     // Solve A*x=B for x=[s,gw,pcb] 7x1 vector
    cv::Mat A = cv::Mat::zeros(3*(N-2),7,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);

    // Step 2.
    // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp) cerr<<"dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;
        if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 - (pc3-pc2)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
	cv::Mat phi = (Rc3-Rc2)*dt12-(Rc2-Rc1)*dt23;
        cv::Mat gamma =Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
        lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
	phi.copyTo(A.rowRange(3*i+0,3*i+3).colRange(4,7));
        gamma.copyTo(B.rowRange(3*i+0,3*i+3));
	
 //  cout<<"21"<<endl;
    }

 //   cout<<"2!"<<endl;
    cv::Mat J2=-A;
    cv::Mat IM2=J2.t()*J2;
    Eigen::Matrix<double,7,7> IMEigen2=Converter::toMatrix7d(IM2);
    finf2<<IMEigen2.trace()<<endl;
 //   cout<<"2222222"<<endl;
    
    cv::Mat w,u,vt;
    // Note w is 7x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
 //   cout<<"2222223"<<endl;
    // Debug log
    //cout<<"u:"<<endl<<u<<endl;
    //cout<<"vt:"<<endl<<vt<<endl;
    //cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv=cv::Mat::eye(7,7,CV_32F);
    for(int i=0;i<7;i++)
    {
        if(fabs(w.at<float>(i))<1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
        }

        winv.at<float>(i,i) = 1./w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x;
    x = vt.t()*winv*u.t()*B;

    // x=[s,gw] 4x1 vector
    sstar = x.at<float>(0);    // scale should be positive
    gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
    pcb_c = x.rowRange(4,7);    
  //  cout<<"pcb_c=:"<<pcb_c<<endl;
  //  cout<<"pcb_t=:"<<pcb_true<<endl;
  //  fpcb<<pcb_cafter.at<float>(0,0)<<"\t"<<pcb_cafter.at<float>(1,0)<<"\t"<<pcb_cafter.at<float>(2,0)<<endl;
  //  <<pcb_true.at<float>(0,0)<<"\t"<<pcb_true.at<float>(1,0)<<"\t"<<pcb_true.at<float>(2,0)<<endl;
    //cout<<"scale sstar: "<<sstar<<endl;
    //cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;
    // Test log
    if(w.type()!=I3.type() || u.type()!=I3.type() || vt.type()!=I3.type())
        cerr<<"different mat type, I3,w,u,vt: "<<I3.type()<<","<<w.type()<<","<<u.type()<<","<<vt.type()<<endl;
    
    return true;
    
}
bool LocalMapping::ScaleGwBaPcbRefine(vector<KeyFrame*> vScaleGravityKF,cv::Mat gwstar,cv::Mat Rcb,double &s_,cv::Mat &gwafter_,cv::Mat &dbiasa_,int Nwin,float thr2,float thr3,ofstream &fpcb,ofstream &finf3)
{
    int N = vScaleGravityKF.size();
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat I1 = cv::Mat::eye(1,1,CV_32F);
 
    // Use gravity magnitude 9.8 as constraint
    // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    // Debug log
    //cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    // Debug log
    //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI*ConfigParam::GetG();//9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba,pcb] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3*(N-2),9,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
	cv::Mat gama = (Rc2-Rc1)*dt23-(Rc3-Rc2)*dt12;
       // cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
       //              - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        cv::Mat psi = Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23);
	lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
	gama.copyTo(C.rowRange(3*i+0,3*i+3).colRange(6,9));
        psi.copyTo(D.rowRange(3*i+0,3*i+3));
	
 //   cout<<"31"<<endl;
        // Debug log
        //cout<<"iter "<<i<<endl;
    }
  //  cout<<"3!"<<endl;
    cv::Mat J3=-C;
    cv::Mat IM3=(J3.t()*J3);
    Eigen::Matrix<double,9,9> IMEigen3=Converter::toMatrix9d(IM3);
    finf3<<IMEigen3.trace()<<endl;
  //  cout<<"32"<<endl;
    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2,u2,vt2;
    cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(9,9,CV_32F);
    for(int i=0;i<9;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }

        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
 //   cout<<"33"<<endl;
    // Then y = vt'*winv*u'*D
    cv::Mat y;
    y = vt2.t()*w2inv*u2.t()*D;

    s_ = y.at<float>(0);
    cv::Mat dthetaxy = y.rowRange(1,3);
    dbiasa_ = y.rowRange(3,6);
    Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
    cv::Mat pcb_cafter = y.rowRange(6,9);
    cout<<"pcb_cafter=:"<<pcb_cafter<<endl;
    cout<<"pcb_true=:"<<pcb_true<<endl;
    ConfigParam::SetPcb(pcb_cafter);       //update pcb
    fpcb<<pcb_cafter.at<float>(0,0)<<"\t"<<pcb_cafter.at<float>(1,0)<<"\t"<<pcb_cafter.at<float>(2,0)<<endl;
    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);
    gwafter_=Rwi_*GI;
    
    //check convergence for pcb and scale
    bool PcbFinish=false;
    bool ScaleFinish=false;
    PcbFinish=CheckPcbConvergence(Nwin,thr2);
    ScaleFinish=CheckScaleConvergence(Nwin,thr3);
    return (PcbFinish&&ScaleFinish);
  //  return PcbFinish;
    
    
}

bool LocalMapping::CheckVIRotationConvergence(/*ofstream &feul,*/int Nwin,float thr)
{
    float thetay=0,thetap=0,thetar=0;
    float sumy=0,sump=0,sumr=0;
    float yaw=0,pitch=0,roll=0;
    float avey=0,avep=0,aver=0;
    ifstream fRandP;
    int num=0;
    bool YPRFinish=false;

    string tmpfilepath = ConfigParam::getTmpFilePath();
    fRandP.open(tmpfilepath+"eul.txt");
    while(1)
    {      
      if(fRandP.eof()!=0) break;
      fRandP>>yaw>>pitch>>roll;
      num++;           
    }
    fRandP.close();
    num=num-1;
  //  cout<<"num=:"<<num<<endl;
    if(num>=Nwin)
    {
    fRandP.open(tmpfilepath+"eul.txt");
    for(int n=0;n<num;n++)
    {
      
      fRandP>>yaw>>pitch>>roll;
      if(n>=num-Nwin)
      {
      sumy=sumy+yaw;
      sump=sump+pitch;
      sumr=sumr+roll;
 //     cout<<"ypr=:"<<"\t"<<yaw<<"\t"<<pitch<<"\t"<<roll<<endl;
      }
    }
    fRandP.close();
    avey=sumy/Nwin;
    avep=sump/Nwin;
    aver=sumr/Nwin;
  // cout<<"ave"<<"\t"<<avey<<"\t"<<avep<<"\t"<<aver<<endl;
    fRandP.open(tmpfilepath+"eul.txt");
    for(int n=0;n<num;n++)
    {
      fRandP>>yaw>>pitch>>roll;
      if(n>=num-Nwin)
      {
      thetay=thetay+(yaw-avey)*(yaw-avey);
      thetap=thetap+(pitch-avep)*(pitch-avep);
      thetar=thetar+(roll-aver)*(roll-aver);
      }
    }
    fRandP.close();
    thetay=sqrt(thetay/Nwin);
    thetap=sqrt(thetap/Nwin);
    thetar=sqrt(thetar/Nwin);
    cout<<"theta"<<"\t"<<thetay<<"\t"<<thetap<<"\t"<<thetar<<endl;
    if(thetay<thr&&thetap<thr&&thetar<thr)
    {
      YPRFinish=true;
      cout<<"Rbc calibration finished!"<<endl;
    }
    }
    return YPRFinish;
}
bool LocalMapping::CheckPcbConvergence(/*ofstream &fpcb,*/int Nwin,float thr)
{
  float thetapx=0,thetapy=0,thetapz=0;
  float sumpx=0,sumpy=0,sumpz=0;
  float px=0,py=0,pz=0;
  float avepx=0,avepy=0,avepz=0;
  ifstream fp;
  int nump=0;
  bool PcbFinish=false;
  
  string tmpfilepath = ConfigParam::getTmpFilePath();
  fp.open(tmpfilepath+"pcb.txt");
  
  while(1)
    {      
      if(fp.eof()!=0) break;
      fp>>px>>py>>pz;
      nump++;           
    }
    fp.close();
    nump=nump-1;
  //  cout<<num<<endl;
    if(nump>=Nwin)
    {
    fp.open(tmpfilepath+"pcb.txt");
    for(int n=0;n<nump;n++)
    {
      
      fp>>px>>py>>pz;
      if(n>=nump-Nwin)
      {
      sumpx=sumpx+px;
      sumpy=sumpy+py;
      sumpz=sumpz+pz;
  //    cout<<"pcb=:"<<"\t"<<px<<"\t"<<py<<"\t"<<pz<<endl;
      }
    }
    fp.close();
    avepx=sumpx/Nwin;
    avepy=sumpy/Nwin;
    avepz=sumpz/Nwin;
  // cout<<"ave"<<"\t"<<avepx<<"\t"<<avepy<<"\t"<<avepz<<endl;
    fp.open(tmpfilepath+"pcb.txt");
    for(int n=0;n<nump;n++)
    {
      fp>>px>>py>>pz;
      if(n>=nump-Nwin)
      {
      thetapx=thetapx+(px-avepx)*(px-avepx);
      thetapy=thetapy+(py-avepy)*(py-avepy);
      thetapz=thetapz+(pz-avepz)*(pz-avepz);
      }
    }
    fp.close();
    thetapx=sqrt(thetapx/Nwin);
    thetapy=sqrt(thetapy/Nwin);
    thetapz=sqrt(thetapz/Nwin);
    cout<<"theta"<<"\t"<<thetapx<<"\t"<<thetapy<<"\t"<<thetapz<<endl;
    
    if(thetapx<thr&&thetapy<thr&&thetapz<thr)
    {
      PcbFinish=true;
      cout<<"pcb calibration finished!"<<endl;
    }
    }
  
  
  return PcbFinish;
}

bool LocalMapping::CheckScaleConvergence(/*ofstream &fscale,*/int Nwin,float thr)
{
  float theta=0;
  float sum=0;
  float time=0,s=0,sstar=0;
  float ave=0;
  ifstream fscale;
  int nums=0;
  bool ScaleFinish=false;
  
  string tmpfilepath = ConfigParam::getTmpFilePath();
  fscale.open(tmpfilepath+"scale.txt");
  
  while(1)
    {      
      if(fscale.eof()!=0) break;
      fscale>>time>>s>>sstar;
      nums++;           
    }
    fscale.close();
    nums=nums-1;
  //  cout<<num<<endl;
    if(nums>=Nwin)
    {
    fscale.open(tmpfilepath+"scale.txt");
    for(int n=0;n<nums;n++)
    {
      
      fscale>>time>>s>>sstar;
      if(n>=nums-Nwin)
      {
      sum=sum+s;
  //    cout<<"pcb=:"<<"\t"<<px<<"\t"<<py<<"\t"<<pz<<endl;
      }
    }
    fscale.close();
    ave=sum/Nwin;
  // cout<<"ave"<<"\t"<<avepx<<"\t"<<avepy<<"\t"<<avepz<<endl;
    fscale.open(tmpfilepath+"scale.txt");
    for(int n=0;n<nums;n++)
    {
      fscale>>time>>s>>sstar;
      if(n>=nums-Nwin)
      {
      theta=theta+(s-ave)*(s-ave);
      }
    }
    fscale.close();
    theta=sqrt(theta/Nwin);
    cout<<"theta"<<"\t"<<theta<<endl;
    
    if(theta<thr)
    {
      ScaleFinish=true;
      cout<<"Initialization finished!"<<endl;
    }
    }
 
  return ScaleFinish;
}


//syl***add
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

KeyFrame* LocalMapping::GetMapUpdateKF()
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    return mpMapUpdateKF;
}

bool LocalMapping::GetMapUpdateFlagForTracking()
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    return mbMapUpdateFlagForTracking;
}

void LocalMapping::SetMapUpdateFlagInTracking(bool bflag)
{
    unique_lock<mutex> lock(mMutexMapUpdateFlag);
    mbMapUpdateFlagForTracking = bflag;
    if(bflag)
    {
        mpMapUpdateKF = mpCurrentKeyFrame;
    }
}

bool LocalMapping::GetVINSInited(void)
{
    unique_lock<mutex> lock(mMutexVINSInitFlag);
    return mbVINSInited;
}

void LocalMapping::SetVINSInited(bool flag)
{
    unique_lock<mutex> lock(mMutexVINSInitFlag);
    mbVINSInited = flag;
}

bool LocalMapping::GetFirstVINSInited(void)
{
    unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
    return mbFirstVINSInited;
}

void LocalMapping::SetFirstVINSInited(bool flag)
{
    unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
    mbFirstVINSInited = flag;
}

cv::Mat LocalMapping::GetGravityVec()
{
    return mGravityVec.clone();
}

bool LocalMapping::TryInitVIO(void)
{
    if(mpMap->KeyFramesInMap()<=mnLocalWindowSize)
        return false;

    static bool fopened = false;
    static ofstream fgw,fscale,fbiasa,fcondnum,ftime,fbiasg;
    if(!fopened)
    {
        // Need to modify this to correct path
        string tmpfilepath = ConfigParam::getTmpFilePath();
        fgw.open(tmpfilepath+"gw.txt");
        fscale.open(tmpfilepath+"scale.txt");
        fbiasa.open(tmpfilepath+"biasa.txt");
        fcondnum.open(tmpfilepath+"condnum.txt");
        ftime.open(tmpfilepath+"computetime.txt");
        fbiasg.open(tmpfilepath+"biasg.txt");
        if(fgw.is_open() && fscale.is_open() && fbiasa.is_open() &&
                fcondnum.is_open() && ftime.is_open() && fbiasg.is_open())
            fopened = true;
        else
        {
            cerr<<"file open error in TryInitVIO"<<endl;
            fopened = false;
        }
        fgw<<std::fixed<<std::setprecision(6);
        fscale<<std::fixed<<std::setprecision(6);
        fbiasa<<std::fixed<<std::setprecision(6);
        fcondnum<<std::fixed<<std::setprecision(6);
        ftime<<std::fixed<<std::setprecision(6);
        fbiasg<<std::fixed<<std::setprecision(6);
    }

    // Extrinsics
    cv::Mat Tbc = ConfigParam::GetMatTbc();
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
    cv::Mat pbc = Tbc.rowRange(0,3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb*pbc;

    // Use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();

    // Step 1.
    // Try to compute initial gyro bias, using optimization with Gauss-Newton
    Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF);

    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(bgest);
    }
    for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();
    }

    // Solve A*x=B for x=[s,gw] 4x1 vector
    cv::Mat A = cv::Mat::zeros(3*(N-2),4,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);

    // Step 2.
    // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp) cerr<<"dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;
        if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
        cv::Mat gamma = (Rc3-Rc2)*pcb*dt12 + (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
        lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
        gamma.copyTo(B.rowRange(3*i+0,3*i+3));
        // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx

        // Debug log
        //cout<<"iter "<<i<<endl;
    }
    // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w,u,vt;
    // Note w is 4x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u:"<<endl<<u<<endl;
    //cout<<"vt:"<<endl<<vt<<endl;
    //cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<4;i++)
    {
        if(fabs(w.at<float>(i))<1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
        }

        winv.at<float>(i,i) = 1./w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x = vt.t()*winv*u.t()*B;

    // x=[s,gw] 4x1 vector
    double sstar = x.at<float>(0);    // scale should be positive
    cv::Mat gwstar = x.rowRange(1,4);   // gravity should be about ~9.8

    // Debug log
    //cout<<"scale sstar: "<<sstar<<endl;
    //cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;

    // Test log
    if(w.type()!=I3.type() || u.type()!=I3.type() || vt.type()!=I3.type())
        cerr<<"different mat type, I3,w,u,vt: "<<I3.type()<<","<<w.type()<<","<<u.type()<<","<<vt.type()<<endl;

    // Step 3.
    // Use gravity magnitude 9.8 as constraint
    // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    // Debug log
    //cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    // Debug log
    //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI*ConfigParam::GetG();//9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3*(N-2),6,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

    for(int i=0; i<N-2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i+1];
        KeyFrame* pKF3 = vScaleGravityKF[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
        cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
                     - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        psi.copyTo(D.rowRange(3*i+0,3*i+3));

        // Debug log
        //cout<<"iter "<<i<<endl;
    }

    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2,u2,vt2;
    // Note w2 is 6x1 vector by SVDecomp()
    // C is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u2:"<<endl<<u2<<endl;
    //cout<<"vt2:"<<endl<<vt2<<endl;
    //cout<<"w2:"<<endl<<w2<<endl;

    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
    for(int i=0;i<6;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }

        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*D
    cv::Mat y = vt2.t()*w2inv*u2.t()*D;

    double s_ = y.at<float>(0);
    cv::Mat dthetaxy = y.rowRange(1,3);
    cv::Mat dbiasa_ = y.rowRange(3,6);
    Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);

    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);


    // Debug log
    {
        cv::Mat gwbefore = Rwi*GI;
        cv::Mat gwafter = Rwi_*GI;
        cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;

        fgw<<mpCurrentKeyFrame->mTimeStamp<<" "
           <<gwafter.at<float>(0)<<" "<<gwafter.at<float>(1)<<" "<<gwafter.at<float>(2)<<" "
           <<gwbefore.at<float>(0)<<" "<<gwbefore.at<float>(1)<<" "<<gwbefore.at<float>(2)<<" "
           <<endl;
        fscale<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<s_<<" "<<sstar<<" "<<endl;
        fbiasa<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<dbiasa_.at<float>(0)<<" "<<dbiasa_.at<float>(1)<<" "<<dbiasa_.at<float>(2)<<" "<<endl;
        fcondnum<<mpCurrentKeyFrame->mTimeStamp<<" "
                <<w2.at<float>(0)<<" "<<w2.at<float>(1)<<" "<<w2.at<float>(2)<<" "<<w2.at<float>(3)<<" "
                <<w2.at<float>(4)<<" "<<w2.at<float>(5)<<" "<<endl;
        //        ftime<<mpCurrentKeyFrame->mTimeStamp<<" "
        //             <<(t3-t0)/cv::getTickFrequency()*1000<<" "<<endl;
        fbiasg<<mpCurrentKeyFrame->mTimeStamp<<" "
              <<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
    }


    // ********************************
    // Todo:
    // Add some logic or strategy to confirm init status
    bool bVIOInited = false;
    if(mbFirstTry)
    {
        mbFirstTry = false;
        mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }
    if(mpCurrentKeyFrame->mTimeStamp - mnStartTime >= 15.0)
    {
        bVIOInited = true;
    }

    // When failed. Or when you're debugging.
    // Reset biasg to zero, and re-compute imu-preintegrator.
    if(!bVIOInited)
    {
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->SetNavStateBiasGyr(Vector3d::Zero());
            pKF->SetNavStateBiasAcc(Vector3d::Zero());
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
        }
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }
    else
    {
        // Set NavState , scale and bias for all KeyFrames
        // Scale
        double scale = s_;
        mnVINSInitScale = s_;
        // gravity vector in world frame
        cv::Mat gw = Rwi_*GI;
        mGravityVec = gw;
        Vector3d gweig = Converter::toVector3d(gw);

        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            // Position and rotation of visual SLAM
            cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
            cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
            // Set position and rotation of navstate
            cv::Mat wPb = scale*wPc + Rwc*pcb;
            pKF->SetNavStatePos(Converter::toVector3d(wPb));
            pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcb));
            // Update bias of Gyr & Acc
            pKF->SetNavStateBiasGyr(bgest);
            pKF->SetNavStateBiasAcc(dbiasa_eig);
            // Set delta_bias to zero. (only updated during optimization)
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
            // Step 4.
            // compute velocity
            if(pKF != vScaleGravityKF.back())
            {
                KeyFrame* pKFnext = pKF->GetNextKeyFrame();
                // IMU pre-int between pKF ~ pKFnext
                const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                // Time from this(pKF) to next(pKFnext)
                double dt = imupreint.getDeltaTime();                                       // deltaTime
                cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
                cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
                cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);           // wPc next
                cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);    // Rwc next

                cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*dbiasa_) + 0.5*gw*dt*dt );
                Eigen::Vector3d veleig = Converter::toVector3d(vel);
                pKF->SetNavStateVel(veleig);
            }
            else
            {
                // If this is the last KeyFrame, no 'next' KeyFrame exists
                KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
                const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
                double dt = imupreint_prev_cur.getDeltaTime();
                Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                //
                Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*dbiasa_eig );
                pKF->SetNavStateVel(veleig);
            }
        }

        // Re-compute IMU pre-integration at last.
        for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }

    return bVIOInited;
}

//rocky for stereo vio
bool LocalMapping::TryInitStereoVIO(void)
{
    if (mpMap->KeyFramesInMap() <= mnLocalWindowSize) //NOTE  mnLocalWindowSize
        return false;

    static bool fopened = false;
    static ofstream fgw, fscale, fbiasa, fcondnum, ftime, fbiasg;
    if (!fopened)
    {
        // Need to modify this to correct path
        string tmpfilepath = ConfigParam::getTmpFilePath();
        fgw.open(tmpfilepath + "gw.txt");
        fscale.open(tmpfilepath + "scale.txt");
        fbiasa.open(tmpfilepath + "biasa.txt");
        fcondnum.open(tmpfilepath + "condnum.txt");
        ftime.open(tmpfilepath + "computetime.txt");
        fbiasg.open(tmpfilepath + "biasg.txt");
        if (fgw.is_open() && fscale.is_open() && fbiasa.is_open() &&
                fcondnum.is_open() && ftime.is_open() && fbiasg.is_open())
            fopened = true;
        else
        {
            cerr << "file open error in TryInitStereoVIO" << endl;
            fopened = false;
        }
        fgw << std::fixed << std::setprecision(6);
        fscale << std::fixed << std::setprecision(6);
        fbiasa << std::fixed << std::setprecision(6);
        fcondnum << std::fixed << std::setprecision(6);
        ftime << std::fixed << std::setprecision(6);
        fbiasg << std::fixed << std::setprecision(6);
    }

    // Extrinsics
    cv::Mat Tbc = ConfigParam::GetMatTbc();
    cv::Mat Rbc = Tbc.rowRange(0, 3).colRange(0, 3);
    cv::Mat pbc = Tbc.rowRange(0, 3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb * pbc;

    // Use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();

    // Step 1.
    // Try to compute initial gyro bias, using optimization with Gauss-Newton
    Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF);

    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
    for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->SetNavStateBiasGyr(bgest);
    }
    for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
    {
        KeyFrame* pKF = *vit;
        pKF->ComputePreInt();
    }

    // Solve A*x=B for x=[s,gw] 4x1 vector
    //cv::Mat A = cv::Mat::zeros(3*(N-2),4,CV_32F);
    cv::Mat A = cv::Mat::zeros(3 * (N - 2), 3, CV_32F); //rcoky for stereo vio

    cv::Mat B = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_32F);

    // Step 2.
    // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    for (int i = 0; i < N - 2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i + 1];
        KeyFrame* pKF3 = vScaleGravityKF[i + 2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if (dt12 != pKF2->mTimeStamp - pKF1->mTimeStamp) cerr << "dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp" << endl;
        if (dt23 != pKF3->mTimeStamp - pKF2->mTimeStamp) cerr << "dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp" << endl;

        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0, 3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0, 3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0, 3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0, 3).colRange(0, 3);
        cv::Mat Rc2 = Twc2.rowRange(0, 3).colRange(0, 3);
        cv::Mat Rc3 = Twc3.rowRange(0, 3).colRange(0, 3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        //
        // cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        // cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
        // cv::Mat gamma = (Rc3-Rc2)*pcb*dt12 + (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
        // lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        // beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
        // gamma.copyTo(B.rowRange(3*i+0,3*i+3));


        // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx

        // Debug log
        //cout<<"iter "<<i<<endl;

        //rocky for stereo vio
        //
        cv::Mat lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
        cv::Mat beta = 0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
        cv::Mat gamma = (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 - Rc1 * Rcb * dv12 * dt12 * dt23;
        gamma = gamma - lambda;

        beta.copyTo(A.rowRange(3 * i + 0, 3 * i + 3));
        gamma.copyTo(B.rowRange(3 * i + 0, 3 * i + 3));

    }
    // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w, u, vt;
    // Note w is 4x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u:"<<endl<<u<<endl;
    //cout<<"vt:"<<endl<<vt<<endl;
    //cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv = cv::Mat::eye(3, 3, CV_32F);
    for (int i = 0; i < 3; i++)
    {
        if (fabs(w.at<float>(i)) < 1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr << "w(i) < 1e-10, w=" << endl << w << endl;
        }

        winv.at<float>(i, i) = 1. / w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x = vt.t() * winv * u.t() * B;
   // cout << "in initiailize stereo vio step2, x.size(should be 1x3): " << x.size() << endl;

    // x=[s,gw] 4x1 vector
    const double sstar = 1;    // scale should be positive
    cv::Mat gwstar = x.rowRange(0, 3);  // gravity should be about ~9.8

    // Debug log
    //cout<<"scale sstar: "<<sstar<<endl;
    //cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;

    // Test log
    if (w.type() != I3.type() || u.type() != I3.type() || vt.type() != I3.type())
        cerr << "different mat type, I3,w,u,vt: " << I3.type() << "," << w.type() << "," << u.type() << "," << vt.type() << endl;

    // Step 3.
    // Use gravity magnitude 9.8 as constraint
    // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
    cv::Mat gI = cv::Mat::zeros(3, 1, CV_32F);
    gI.at<float>(2) = 1;
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar / cv::norm(gwstar);
    // Debug log
    //cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn / normgIxgwn;
    double theta = std::atan2(normgIxgwn, gI.dot(gwn));
    // Debug log
    //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig * theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI * ConfigParam::GetG(); //9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3 * (N - 2), 5, CV_32F);
    cv::Mat D = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);

    for (int i = 0; i < N - 2; i++)
    {
        KeyFrame* pKF1 = vScaleGravityKF[i];
        KeyFrame* pKF2 = vScaleGravityKF[i + 1];
        KeyFrame* pKF3 = vScaleGravityKF[i + 2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(pKF3->GetIMUPreInt().getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = pKF1->GetPoseInverse();
        cv::Mat Twc2 = pKF2->GetPoseInverse();
        cv::Mat Twc3 = pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0, 3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0, 3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0, 3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0, 3).colRange(0, 3);
        cv::Mat Rc2 = Twc2.rowRange(0, 3).colRange(0, 3);
        cv::Mat Rc3 = Twc3.rowRange(0, 3).colRange(0, 3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        // 
        // 
        // cv::Mat lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
        // cv::Mat phi = - 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * Rwi * SkewSymmetricMatrix(GI); // note: this has a '-', different to paper
        // cv::Mat zeta = Rc2 * Rcb * Jpba23 * dt12 + Rc1 * Rcb * Jvba12 * dt12 * dt23 - Rc1 * Rcb * Jpba12 * dt23;
        // cv::Mat psi = (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 - (Rc2 - Rc3) * pcb * dt12
        //               - Rc2 * Rcb * dp23 * dt12 - Rc1 * Rcb * dv12 * dt23 * dt12 - 0.5 * Rwi * GI * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23); // note:  - paper
        // lambda.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).col(0));
        // phi.colRange(0, 2).copyTo(C.rowRange(3 * i + 0, 3 * i + 3).colRange(1, 3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        // zeta.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).colRange(3, 6));
        // psi.copyTo(D.rowRange(3 * i + 0, 3 * i + 3));


        //rocky for stereo vio
        cv::Mat lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
        cv::Mat phi = - 0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * Rwi * SkewSymmetricMatrix(GI); // note: this has a '-', different to paper
        cv::Mat zeta = Rc2 * Rcb * Jpba23 * dt12 + Rc1 * Rcb * Jvba12 * dt12 * dt23 - Rc1 * Rcb * Jpba12 * dt23;
        cv::Mat psi = (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 - (Rc2 - Rc3) * pcb * dt12
                      - Rc2 * Rcb * dp23 * dt12 - Rc1 * Rcb * dv12 * dt23 * dt12 - 0.5 * Rwi * GI * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23); // note:  - paper
        psi = psi - lambda;
        // lambda.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).col(0));
        phi.colRange(0, 2).copyTo(C.rowRange(3 * i + 0, 3 * i + 3).colRange(0, 2)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).colRange(2, 5));
        psi.copyTo(D.rowRange(3 * i + 0, 3 * i + 3));



        // Debug log
        //cout<<"iter "<<i<<endl;
    }

    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2, u2, vt2;
    // Note w2 is 6x1 vector by SVDecomp()
    // C is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(C, w2, u2, vt2, cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u2:"<<endl<<u2<<endl;
    //cout<<"vt2:"<<endl<<vt2<<endl;
    //cout<<"w2:"<<endl<<w2<<endl;

    // Compute winv
    cv::Mat w2inv = cv::Mat::eye(5, 5, CV_32F);
    for (int i = 0; i < 5; i++)
    {
        if (fabs(w2.at<float>(i)) < 1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr << "w2(i) < 1e-10, w=" << endl << w2 << endl;
        }

        w2inv.at<float>(i, i) = 1. / w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*D
    cv::Mat y = vt2.t() * w2inv * u2.t() * D;

    const double s_ = 1;
    cv::Mat dthetaxy = y.rowRange(0, 2);
    cv::Mat dbiasa_ = y.rowRange(2, 5);
    Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);

    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3, 1, CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0, 2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig * Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_);


    // Debug log
    {
        cv::Mat gwbefore = Rwi * GI;
        cv::Mat gwafter = Rwi_ * GI;
        cout << "Time: " << mpCurrentKeyFrame->mTimeStamp - mnStartTime << ", sstar: " << sstar << ", s: " << s_ << endl;

        fgw << mpCurrentKeyFrame->mTimeStamp << " "
            << gwafter.at<float>(0) << " " << gwafter.at<float>(1) << " " << gwafter.at<float>(2) << " "
            << gwbefore.at<float>(0) << " " << gwbefore.at<float>(1) << " " << gwbefore.at<float>(2) << " "
            << endl;
        fscale << mpCurrentKeyFrame->mTimeStamp << " "
               << s_ << " " << sstar << " " << endl;
        fbiasa << mpCurrentKeyFrame->mTimeStamp << " "
               << dbiasa_.at<float>(0) << " " << dbiasa_.at<float>(1) << " " << dbiasa_.at<float>(2) << " " << endl;
        fcondnum << mpCurrentKeyFrame->mTimeStamp << " "
                 << w2.at<float>(0) << " " << w2.at<float>(1) << " " << w2.at<float>(2) << " " << w2.at<float>(3) << " "
                 << w2.at<float>(4) << " " << w2.at<float>(5) << " " << endl;
        //        ftime<<mpCurrentKeyFrame->mTimeStamp<<" "
        //             <<(t3-t0)/cv::getTickFrequency()*1000<<" "<<endl;
        fbiasg << mpCurrentKeyFrame->mTimeStamp << " "
               << bgest(0) << " " << bgest(1) << " " << bgest(2) << " " << endl;
    }


    // ********************************
    // Todo:
    // Add some logic or strategy to confirm init status
    bool bVIOInited = false;
    if (mbFirstTry)
    {
        mbFirstTry = false;
        mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }
    if (mpCurrentKeyFrame->mTimeStamp - mnStartTime >= 15.0)//ADJU   adjust this time for stereo init
    {
        bVIOInited = true;
    }

    // When failed. Or when you're debugging.
    // Reset biasg to zero, and re-compute imu-preintegrator.
    if (!bVIOInited)
    {
        for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->SetNavStateBiasGyr(Vector3d::Zero());
            pKF->SetNavStateBiasAcc(Vector3d::Zero());
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
        }
        for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }
    else
    {
        // Set NavState , scale and bias for all KeyFrames
        // Scale
        double scale = s_;
        mnVINSInitScale = s_;
        // gravity vector in world frame
        cv::Mat gw = Rwi_ * GI;
        mGravityVec = gw;
        Vector3d gweig = Converter::toVector3d(gw);

        for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
        {
            KeyFrame* pKF = *vit;
            // Position and rotation of visual SLAM
            cv::Mat wPc = pKF->GetPoseInverse().rowRange(0, 3).col(3);                  // wPc
            cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0, 3).colRange(0, 3);          // Rwc
            // Set position and rotation of navstate
            cv::Mat wPb = scale * wPc + Rwc * pcb;
            pKF->SetNavStatePos(Converter::toVector3d(wPb));
            pKF->SetNavStateRot(Converter::toMatrix3d(Rwc * Rcb));
            // Update bias of Gyr & Acc
            pKF->SetNavStateBiasGyr(bgest);
            pKF->SetNavStateBiasAcc(dbiasa_eig);
            // Set delta_bias to zero. (only updated during optimization)
            pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
            pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
            // Step 4.
            // compute velocity
            if (pKF != vScaleGravityKF.back())
            {
                KeyFrame* pKFnext = pKF->GetNextKeyFrame();
                // IMU pre-int between pKF ~ pKFnext
                const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                // Time from this(pKF) to next(pKFnext)
                double dt = imupreint.getDeltaTime();                                       // deltaTime
                cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
                cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
                cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0, 3).col(3);          // wPc next
                cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0, 3).colRange(0, 3);  // Rwc next

                cv::Mat vel = - 1. / dt * ( scale * (wPc - wPcnext) + (Rwc - Rwcnext) * pcb + Rwc * Rcb * (dp + Jpba * dbiasa_) + 0.5 * gw * dt * dt );
                Eigen::Vector3d veleig = Converter::toVector3d(vel);
                pKF->SetNavStateVel(veleig);
            }
            else
            {
                // If this is the last KeyFrame, no 'next' KeyFrame exists
                KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
                const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
                double dt = imupreint_prev_cur.getDeltaTime();
                Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                //
                Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                Eigen::Vector3d veleig = velpre + gweig * dt + rotpre * ( dv + Jvba * dbiasa_eig );
                pKF->SetNavStateVel(veleig);
            }
        }

        // Re-compute IMU pre-integration at last.
        for (vector<KeyFrame*>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend; vit++)
        {
            KeyFrame* pKF = *vit;
            pKF->ComputePreInt();
        }
    }

    return bVIOInited;
}

void LocalMapping::AddToLocalWindow(KeyFrame* pKF)
{
    mlLocalKeyFrames.push_back(pKF);
    if(mlLocalKeyFrames.size() > mnLocalWindowSize)
    {
        mlLocalKeyFrames.pop_front();
    }
}

void LocalMapping::DeleteBadInLocalWindow(void)
{
    std::list<KeyFrame*>::iterator lit = mlLocalKeyFrames.begin();
    while(lit != mlLocalKeyFrames.end())
    {
        KeyFrame* pKF = *lit;
        //Test log
        if(!pKF) cout<<"pKF null?"<<endl;
        if(pKF->isBad())
        {
            lit = mlLocalKeyFrames.erase(lit);
        }
        else
        {
            lit++;
        }
    }
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

LocalMapping::LocalMapping(Map *pMap, const float bMonocular, ConfigParam* pParams):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
    mpParams = pParams;
    mnLocalWindowSize = ConfigParam::GetLocalWindowSize();
    cout<<"mnLocalWindowSize:"<<mnLocalWindowSize<<endl;

    mbVINSInited = false;
    mbFirstTry = true;
    mbFirstVINSInited = false;
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
      //syl***add
    Tbc_true = ConfigParam::GetMatTbc();
    Rbc_true = Tbc_true.rowRange(0,3).colRange(0,3);
    pbc_true = Tbc_true.rowRange(0,3).col(3);
    Rcb_true = Rbc_true.t();
    pcb_true = -Rcb_true*pbc_true;
    
    ofstream fInitCalibTime;
    string tmpfilepath = ConfigParam::getTmpFilePath();
    fInitCalibTime.open(tmpfilepath+"InitCalibTime.txt");
    //syl***add end

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Local Window also updated in below function
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                {
                    if(!GetVINSInited())
                    {
                        //Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,mlLocalKeyFrames,&mbAbortBA, mpMap, this);
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA,mpMap,this);
                    }
                    else
                    {
                        Optimizer::LocalBundleAdjustmentNavState(mpCurrentKeyFrame,mlLocalKeyFrames,&mbAbortBA, mpMap, mGravityVec, this);
                    }
                }

                // Try to initialize VIO, if not inited
                if(!GetVINSInited())
                {
                     bool tmpbool;
                    
                     if(!mbMonocular)
                     {
                         tmpbool=TryInitStereoVIO();//如果是不是单目，则scale为1
                     }
                     else
                     {
			 std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		         tmpbool=TryInitVIOWithoutPreCalibration();
			// tmpbool = TryInitVIO();//单目的vio初始化
			 std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
			 double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
			 fInitCalibTime<<t<<endl;
                         
                     }
                              
                    SetVINSInited(tmpbool);
                    if(tmpbool)
                    {
                        // Update map scale
                        mpMap->UpdateScale(mnVINSInitScale);
                        // Set initialization flag
                        SetFirstVINSInited(true);
                    }
                }

                // May set bad for KF in LocalWindow
                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Delete bad KF in LocalWindow
    DeleteBadInLocalWindow();
    // Add Keyframe to LocalWindow
    AddToLocalWindow(mpCurrentKeyFrame);

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    KeyFrame* pOldestLocalKF = mlLocalKeyFrames.front();
    KeyFrame* pPrevLocalKF = pOldestLocalKF->GetPrevKeyFrame();
    KeyFrame* pNewestLocalKF = mlLocalKeyFrames.back();
    // Test log
    if(pOldestLocalKF->isBad()) cerr<<"pOldestLocalKF is bad, check 1. id: "<<pOldestLocalKF->mnId<<endl;
    if(pPrevLocalKF) if(pPrevLocalKF->isBad()) cerr<<"pPrevLocalKF is bad, check 1. id: "<<pPrevLocalKF->mnId<<endl;
    if(pNewestLocalKF->isBad()) cerr<<"pNewestLocalKF is bad, check 1. id: "<<pNewestLocalKF->mnId<<endl;

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;

        // Don't cull the oldest KF in LocalWindow,
        // And the KF before this KF
        if(pKF == pOldestLocalKF || pKF == pPrevLocalKF)
            continue;

        // Check time between Prev/Next Keyframe, if larger than 0.5s(for local)/3s(others), don't cull
        // Note, the KF just out of Local is similarly considered as Local
        KeyFrame* pPrevKF = pKF->GetPrevKeyFrame();
        KeyFrame* pNextKF = pKF->GetNextKeyFrame();
        if(pPrevKF && pNextKF)
        {
            double timegap=0.5;
            if(GetVINSInited())
                timegap = 3;

            // Test log
            if(pOldestLocalKF->isBad()) cerr<<"pOldestLocalKF is bad, check 1. id: "<<pOldestLocalKF->mnId<<endl;
            if(pPrevLocalKF) if(pPrevLocalKF->isBad()) cerr<<"pPrevLocalKF is bad, check 1. id: "<<pPrevLocalKF->mnId<<endl;
            if(pNewestLocalKF->isBad()) cerr<<"pNewestLocalKF is bad, check 1. id: "<<pNewestLocalKF->mnId<<endl;

            if(pKF->mnId >= pOldestLocalKF->mnId)
            {
                timegap = 0.1;    // third tested, good
                if(GetVINSInited())
                    timegap = 0.5;
                // Test log
                if(pKF->mnId >= pNewestLocalKF->mnId)
                    cerr<<"Want to cull Newer KF than LocalWindow? id/currentKFid:"<<pKF->mnId<<"/"<<mpCurrentKeyFrame->mnId<<endl;
            }
            if(fabs(pNextKF->mTimeStamp - pPrevKF->mTimeStamp) > timegap)
                continue;
        }


        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;

        mlLocalKeyFrames.clear();

        // Add resetting init flags
        mbVINSInited = false;
        mbFirstTry = true;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
