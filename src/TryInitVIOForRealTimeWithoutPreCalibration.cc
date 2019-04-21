#include "LocalMapping.h"

/**
  Estimate Rcb (the rotation from body to camera)
 * @brief LocalMapping::ExtrinsicOrientationCalib_MonoVI
 * @param vKFInit
 * @param q_cb_est_last: used to calculate weight
 * @param bWeight
 * @return q_cb_est
 */
Quaterniond LocalMapping::ExtrinsicOrientationCalib_MonoVI(
        const vector<KeyFrameInit *> &vKFInit,
        const bool bWeight)
{
 /*   int N = vKFInit.size();

    Eigen::Quaterniond q_cb_est;

    // Solve A*q=0 for q=[w,x,y,z]^T
    // We can get N-1 equations
    cv::Mat A = cv::Mat::zeros(4*(N-1),4,CV_32F);
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);

    /// Step 1.2.1: construct A

    for(int i=0; i<N-1; i++)
    {
        KeyFrameInit* pKF1 = vKFInit[i];
        KeyFrameInit* pKF2 = vKFInit[i+1];
        // Step 1.2.1-a: Compute the delta_R_B and corresponding quaternion q_B (normalized)
        Eigen::Matrix3d delta_R_B = pKF2->GetIMUPreInt().getDeltaR();
        Eigen::Quaterniond q_B = Eigen::Quaterniond(delta_R_B); q_B.normalize();    // Note: the order is (x,y,z,w)
        // Step 1.2.1-b: Compute the delta_R_C and corresponding quaternion q_C (normalized)
        Eigen::Matrix3d delta_R_C = Converter::toMatrix3d(pKF1->GetRotation() * pKF2->GetRotation().t());
        Eigen::Quaterniond q_C = Eigen::Quaterniond(delta_R_C); q_C.normalize();    // Note: the order is (x,y,z,w)

        // Step 1.2.1-c: Compute the matrix Q
        Eigen::Vector3d q_B_v(q_B.x(), q_B.y(), q_B.z());
        double q_B_w = q_B.w();
        Eigen::Vector3d q_C_v(q_C.x(), q_C.y(), q_C.z());
        double q_C_w = q_C.w();

        cv::Mat Q = cv::Mat::zeros(4, 4, CV_32F);
        Q.at<float>(0,0) = q_B_w - q_C_w;
        Eigen::Vector3d tmp_v(q_B_v-q_C_v);
        cv::Mat Q_10_30 = Converter::toCvMat( Eigen::Matrix<double,3,1>(tmp_v(0), tmp_v(1), tmp_v(2)));
        Q_10_30.copyTo(Q.rowRange(1,4).col(0));
        cv::Mat Q_01_03= -Q_10_30.t();
        Q_01_03.copyTo(Q.row(0).colRange(1,4));
        cv::Mat Q_11_33 = (q_B_w - q_C_w)*I3-Converter::toSkew(q_B_v)-Converter::toSkew(q_C_v);
        Q_11_33.copyTo(Q.rowRange(1,4).colRange(1,4));

        // Compute the wight wi
        if(bWeight)
        {
            // error
            cv::Mat e = Q * ( cv::Mat_<float>(4,1) << q_cb_est_last.w(), q_cb_est_last.x(), q_cb_est_last.y(), q_cb_est_last.z() );
            // exponential weight function
            double wi = std::exp(- cv::norm(e) * 200.0);
            if (cv::norm(e) > 0.05)   // can be deleted
                wi = 0.0;

            Q = wi * Q;
        }

        // Stack to matrix A
        Q.copyTo(A.rowRange(4*i+0, 4*i+4).colRange(0,4));
    }

    /// Solve A*q_cb=0  ==> q_cb = argmin|A*q_cb|, with constrain: |q_cb|=1
    cv::Mat q_cb_solved;   // the order is [w,x,y,z]
    cv::SVD::solveZ(A,q_cb_solved);

    // Debug log
    if(q_cb_solved.empty()){
        std::cerr << "[Warning] cv::SVD::solveZ() result is empty" << std::endl;
//        return;
    }
    if(q_cb_solved.at<float>(0,0)< 1.0e-10) q_cb_solved = -q_cb_solved;

    // ref in Eigen/Quaternion.h
    // inline Quaternion(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) : m_coeffs(x, y, z, w){}
    q_cb_est = Eigen::Quaterniond(q_cb_solved.at<float>(0,0), q_cb_solved.at<float>(1,0), q_cb_solved.at<float>(2,0), q_cb_solved.at<float>(3,0));
    q_cb_est.normalize();

    // update q_cb_est_last
    q_cb_est_last = q_cb_est;

    return q_cb_est;*/

}


bool LocalMapping::ScaleGravityTranslationApproximation_MonoVI(
        const vector<KeyFrameInit *> &vKFInit)
{
  /*  int N = vKFInit.size();

    // Solve A*x=B for x=[s,gw, pcb] 7x1 vector
    cv::Mat A = cv::Mat::zeros(3*(N-2),7,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);

    cv::Mat A_without_weight = cv::Mat::zeros(3*(N-2),7,CV_32F);    // Just for Debug
    cv::Mat B_without_weight = cv::Mat::zeros(3*(N-2),1,CV_32F);    // Just for Debug

    /// Step 1.3.1: Construct A
    for(int i=0; i<N-2; i++)
    {
        KeyFrameInit* pKF1 = vKFInit[i];
        KeyFrameInit* pKF2 = vKFInit[i+1];
        KeyFrameInit* pKF3 = vKFInit[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        // Test log
        if(dt12!=pKF2->mTimeStamp-pKF1->mTimeStamp) cerr<<"dt12 != pKF2->mTimeStamp-pKF1->mTimeStamp"<<endl;
        if(dt23!=pKF3->mTimeStamp-pKF2->mTimeStamp) cerr<<"dt23 != pKF3->mTimeStamp-pKF2->mTimeStamp"<<endl;

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
        // lambda*s + beta*g + phi*pcb = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
        cv::Mat phi = (Rc2-Rc3)*dt12 - (Rc1-Rc2)*dt23;
        cv::Mat gamma = Rc1*Rcbstar*dp12*dt23 - Rc2*Rcbstar*dp23*dt12 - Rc1*Rcbstar*dv12*dt12*dt23; // use the estimated Rcbstar

//        // compute the weight wi
//        if(sstar_last>0){
//            // predict gamma
//            cv::Mat gamma_pred = lambda * sstar_last + beta * gwstar_last + phi * pcbstar_last;
//            // error
//            cv::Mat e = gamma_pred - gamma;
//            // exponential weight function
//            double  wi = std::exp(- cv::norm(e) * 100.0);
//            if (cv::norm(e) > 0.05)   // can be deleted
//                wi = 0.0;

//            lambda *= wi;
//            beta *= wi;
//            phi *= wi;
//            gamma *= wi;
//        }

        // Stack to matrix A
        lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
        phi.copyTo(A.rowRange(3*i+0,3*i+3).colRange(4,7));
        gamma.copyTo(B.rowRange(3*i+0,3*i+3));

    }

    // Step 1.3.2: Solve Ax=B ==> x* = argmin |A*x-B|
    cv::Mat x;
        cv::solve(A,B,x,cv::DECOMP_SVD);
//    cv::solve(A,B,x,cv::DECOMP_LU|cv::DECOMP_NORMAL);

    // Debuug log
    if(x.empty()){
        std::cerr << "[Warning] cv::solve() result is empty" << std::endl;
        return false;
    }

    sstar = x.at<float>(0);     // scale should be positive
    // Debug log
    if(sstar <= 1.0e-10)    {std::cerr << "[Warning] Scale should be positive, but sstar < 0, why? return false." << std::endl; return false;}
    gwstar = x.rowRange(1,4);   // |gwstar| should be about ~9.8
    pcbstar = x.rowRange(4,7);

    // Update last record
    sstar_last = sstar;
    gwstar_last = gwstar;
    pcbstar_last = pcbstar;

    LOG(INFO) << "sstar = " << sstar;
    LOG(INFO) << "gwstar = " << gwstar.t();
    LOG(INFO) << "pcbstar = " << pcbstar.t();*/

    return true;
}

// accelerometer bias estimation, and scale, gravity and translation refinement
bool LocalMapping::AccBiasEstAndScaleGravityTransRefine_MonoVI(
        const vector<KeyFrameInit*> &vKFInit
        )
{
 /*   int N = vKFInit.size();
    cv::Mat dthetaxy = cv::Mat::zeros(2,1,CV_32F);

    // Use gravity magnitude 9.8 as constraint
    cv::Mat GI = cv::Mat::zeros(3,1,CV_32F);
    GI.at<float>(2) = -1 * mpParams->GetG(); // ConfigParam::GetG();//9.810;
    cv::Mat GIxgwstar = GI.cross(gwstar);
    cv::Mat vhat = GIxgwstar / cv::norm(GIxgwstar);
    double theta = std::atan2(cv::norm(GIxgwstar), GI.dot(gwstar));

    // calculate Rwi according to theta
    cv::Mat Rwi = cv::Mat::eye(3, 3, CV_32F);
    Eigen::Matrix3d RWIeig = Eigen::Matrix3d::Identity();
    if (theta < 1.0e-10)
    {
        LOG(WARNING) << YELLOW"theta < 1.0e-10" << RESET;
        LOG(WARNING) << YELLOW"GI = " << GI.t() << RESET;
    }
    else
    {
        Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
        RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
        Rwi = Converter::toCvMat(RWIeig);
    }

    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3*(N-2),9,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

    for(int i=0; i<N-2; i++)
    {
        KeyFrameInit* pKF1 = vKFInit[i];
        KeyFrameInit* pKF2 = vKFInit[i+1];
        KeyFrameInit* pKF3 = vKFInit[i+2];
        // Delta time between frames
        double dt12 = pKF2->GetIMUPreInt().getDeltaTime();
        double dt23 = pKF3->GetIMUPreInt().getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(pKF2->GetIMUPreInt().getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(pKF3->GetIMUPreInt().getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(pKF2->GetIMUPreInt().getJVBiasa()); // ??
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
        cv::Mat lambda = (pc2-pc1)*dt23 - (pc3-pc2)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*Converter::toSkew(GI);
        cv::Mat zeta = Rc2*Rcbstar*Jpba23*dt12 - Rc1*Rcbstar*Jpba12*dt23 + Rc1*Rcbstar*Jvba12*dt12*dt23;
        cv::Mat ksi = (Rc2-Rc3)*dt12 - (Rc1-Rc2)*dt23;
        cv::Mat psi =  Rc1*Rcbstar*dp12*dt23 - Rc2*Rcbstar*dp23*dt12 - Rc1*Rcbstar*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper

//        // compute the weight wi
//        if(s_refined_last > 0){
//            cv::Mat a = lambda * s_refined_last;
//            cv::Mat b = phi * dtheta_last;
//            cv::Mat c = zeta * biasa_last;
//            cv::Mat d = ksi * pcb_refined_last;
//            cv::Mat psi_pred = a+b+c+d;

//            cv::Mat e = psi_pred - psi;
//            double wi = std::exp(-cv::norm(e));
//            if (cv::norm(e) > 0.05)   // 0.05  // can be deleted
//                wi = 0.0;


//            lambda *= wi;
//            phi *= wi;
//            zeta *= wi;
//            ksi *= wi;
//        }

        // Stack into matrix C & D
        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        ksi.copyTo(C.rowRange(3*i+0,3*i+3).colRange(6,9));
        psi.copyTo(D.rowRange(3*i+0,3*i+3));

    } // end for

    cv::Mat y;
    cv::solve(C,D,y,cv::DECOMP_SVD);
//    cv::solve(C,D,y,cv::DECOMP_LU|cv::DECOMP_NORMAL);
    // Debug log
    if(y.empty()){
        std::cerr << "[Warning] In refinement, cv::solve() result is empty" << std::endl;
        return false;
    }

    s_refined = y.at<float>(0);
    dthetaxy = y.rowRange(1,3);
    biasa_ = y.rowRange(3,6);
    pcb_refined = y.rowRange(6,9);


    // dtheta = [dx;dy;0]
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*Exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    cv::Mat Rwi_ = Converter::toCvMat(Rwieig_); // Rwi_: after refined
    gw_refined = Rwi_*GI;

    mRwiInit = Rwi_;

    // update last
    s_refined_last = s_refined;
    dtheta_last = dtheta;
    biasa_last = biasa_;
    pcb_refined_last = pcb_refined;*/

    return true;
}

/**
 * @brief LocalMapping::TryInitVIOForRealTimeWithoutPreCalibration
 * @return
 * try to initial the Mono VIO in real-time, as well as estimating the extrinsic parameters
 */
bool LocalMapping::TryInitVIOWithoutPreCalibration(void)
{
 /*   // set initial temp variables once
    {
        if (mbInitialSetting)
        {
            mbInitialSetting = false;

            Eigen::Matrix3d rotationMatrixRand = Eigen::Matrix3d::Identity();
            if (mpParams->GetCreateRandomMatrixLastForFirstVIOInit())
            {
                std::srand(time(NULL));
                double yaw_rng      = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)
                double pitch_rng    = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)
                double roll_rng     = (std::rand()%1000/1000.0 - 0.5) * 2;  // (-1, 1)

                Eigen::AngleAxisd yawAngle(yaw_rng*M_PI, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd pitchAngle(pitch_rng*M_PI, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rollAngle(roll_rng*M_PI, Eigen::Vector3d::UnitX());
                Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
                rotationMatrixRand = q.matrix();

                Eigen::Vector3d euler_angles = rotationMatrixRand.eulerAngles(2, 1, 0); // ZYX顺序,即yaw, pitch, roll顺序
                cout << "\n[INFO] create random Rbc: yaw pitch roll = " << euler_angles.transpose()  << endl;
            }

            Rcbstar_last = Converter::toCvMat(rotationMatrixRand);   // random
            sstar_last = -1;
            gwstar_last = cv::Mat::zeros(3,1,CV_32F);
            pcbstar_last = cv::Mat::zeros(3,1,CV_32F);

            s_refined_last = -1;
            dtheta_last = cv::Mat::zeros(3,1,CV_32F);;
            biasa_last = cv::Mat::zeros(3,1,CV_32F);;
            pcb_refined_last = cv::Mat::zeros(3,1,CV_32F);

            // init as a unit quaternion
            q_cb_est_last = Eigen::Quaterniond(1,0,0,0);

            bFirstEstimateRcb = true;
        }
    }

    // reset bgest before initialization process
    Vector3d bgest = Eigen::Vector3d(0, 0, 0);

    // only when the map has enough keyframes, can the system try to initialize
    if(mpMap->KeyFramesInMap()<=mnLocalWindowSize)
        return false;

    // record start time
    if(mbFirstTry)
    {
       mbFirstTry = false;
       mnStartTime = mpCurrentKeyFrame->mTimeStamp;
    }

    Timer timerOffProcessing_Time;  // start time-counter

    // perform GBA for all map point before initialization
     Optimizer::GlobalBundleAdjustemnt(mpMap, 10);

    // Wait KeyFrame Culling.
    // 1. if KeyFrame Culling is running, wait until finished.
    // 2. if KFs are being copied, then don't run KeyFrame Culling (in KeyFrameCulling function)
    while(GetFlagCopyInitKFs())
        usleep(1000);

    SetFlagCopyInitKFs(true);
    // use all KeyFrames in map to compute
    vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
    int N = vScaleGravityKF.size();
    KeyFrame* pNewestKF = vScaleGravityKF[N-1];
    vector<cv::Mat> vTwc;
    vector<IMUPreintegrator> vIMUPreInt;
    // Store initialization-required KeyFrame data
    vector<KeyFrameInit*> vKFInit;

    for(int i=0; i<N; i++)
    {
        KeyFrame* pKF = vScaleGravityKF[i];
        vTwc.push_back(pKF->GetPoseInverse());
        vIMUPreInt.push_back(pKF->GetIMUPreInt());
        KeyFrameInit* pkfi = new KeyFrameInit(*pKF);
        vKFInit.push_back(pkfi);
        if(i!=0)
            pkfi->mpPrevKeyFrame  = vKFInit[i-1];

    }
    SetFlagCopyInitKFs(false);


    /// Step 0. performa once, assume biasg=0, and estimate an initial Rcbstar_last which will be used to estimate gyroscope bias
    if(sstar_last==-1)
    {
        cout << "// performa once, assume biasg=0, and estimate Rcbstar_last" << endl;
        sstar_last = -2;

        /// Estimate Rcb (the rotation from body to camera)
        {
            Eigen::Quaterniond q_cb_est = ExtrinsicOrientationCalib_MonoVI(vKFInit, false); // first estimate, do not use weight

            q_cb_est_last = q_cb_est;

            /// get Rcbstar, and check convergence
            /// If the rotate euler angles of three axises (yaw, pitch, roll) are both convergent,
            /// we consider the Rcbstar is convergent
            Eigen::Matrix3d Rcb_est_matrix = q_cb_est.toRotationMatrix();
            Rcbstar = Converter::toCvMat(Rcb_est_matrix);
            Rcbstar_last = Rcbstar;

            // write R_bc_approximate.txt and check convergence
            {
                Eigen::Vector3d euler_bc_est = Converter::toEulerAngles(Rcb_est_matrix.inverse(), true); //Rbc_est_matrix.eulerAngles(2, 1, 0);   // yaw, pitch, roll
                fR_bc_appro << pNewestKF->mTimeStamp<<" " << euler_bc_est(0) << " " << euler_bc_est(1) << " " << euler_bc_est(2) << endl;
                fcalibrated_errors_Rbc << pNewestKF->mTimeStamp<<" " << gt_yaw-euler_bc_est(0) << " " << gt_pitch-euler_bc_est(1) << " " << gt_roll-euler_bc_est(2) << endl;

                {
                    // record estimated results and timestamps
                    mvRecordEstimatedRbc.push_back(euler_bc_est);
                    mvRecordEstimatedRbcTimeStamps.push_back(pNewestKF->mTimeStamp);
                }
            }
        }

    } // End of performa once, assume biasg=0, and estimate Rcbstar_last


    // Try to compute initial gyro bias, using optimization with Gauss-Newtion
    bgest = Optimizer::OptimizeInitialGyroBias(vTwc,vIMUPreInt, Rcbstar_last);

    // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
    for(int i=0; i<N; i++)
        vKFInit[i]->bg = bgest;
    for(int i=0; i<N; i++)
        vKFInit[i]->ComputePreInt();

    /// My innovation 1
    /// Step 1.2: Estimate Rcb (the rotation from body to camera)
    {
        // estimate q_cb_est with weight
        Eigen::Quaterniond q_cb_est = ExtrinsicOrientationCalib_MonoVI(vKFInit, true);

        /// Step 1.2.3: get Rcbstar, and check convergence
        /// If the rotate euler angles of three axises (yaw, pitch, roll) are both convergent,
        /// we consider the Rcbstar is convergent
        Eigen::Matrix3d Rcb_est_matrix = q_cb_est.toRotationMatrix();
        Rcbstar = Converter::toCvMat(Rcb_est_matrix);

        Rcbstar_last = Rcbstar;

        // write R_bc_approximate.txt and check convergence
        {
            Eigen::Vector3d euler_bc_est =  Converter::toEulerAngles(Rcb_est_matrix.inverse(), true); //Rbc_est_matrix.eulerAngles(2, 1, 0);   // yaw, pitch, roll
            fR_bc_appro << pNewestKF->mTimeStamp<<" " << euler_bc_est(0) << " " << euler_bc_est(1) << " " << euler_bc_est(2) << endl;
            fcalibrated_errors_Rbc << pNewestKF->mTimeStamp<<" " << gt_yaw-euler_bc_est(0) << " " << gt_pitch-euler_bc_est(1) << " " << gt_roll-euler_bc_est(2) << endl;

            // check whether the estimated Rbc has converged
            {
                // record estimated results and timestamps
                mvRecordEstimatedRbc.push_back(euler_bc_est);
                mvRecordEstimatedRbcTimeStamps.push_back(pNewestKF->mTimeStamp);

                mbVINSInitRbcConverged = CheckRbcEstimationConverge();
            }
        }
    }   // End of Step 1.2: Estimate Rcb (the rotation from body to camera)


    // Only when the Rcb converge, can the following code be executed.
//    if (!mbVINSInitRbcConverged)
//        return false;

    /// My innovation 2
    /// Step 1.3: Scale, Gravity and pcb (no accelerometer bias) Estimation
    /// Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
    {
        bool bValid = ScaleGravityTranslationApproximation_MonoVI(vKFInit);
        if(!bValid)
            return false;

        // write p_bc_approximate.txt
        {
            cv::Mat pbcstar = -Rcbstar.t()*pcbstar;   // use estimated Rcb_est
            fp_bc_appro << pNewestKF->mTimeStamp<<" "
                           << pbcstar.at<float>(0) << " " << pbcstar.at<float>(1) << " " << pbcstar.at<float>(2) << endl;
        }


    }   // End of Step 1.3: Scale, Gravity and pcb (no accelerometer bias) Estimation

    /// My innovation 3:
    /// Step 1.4: Accelerometer Bias Estimation, and Scale, Gravity Direction and Pcb Refinement
    {
        bool bValid = AccBiasEstAndScaleGravityTransRefine_MonoVI(vKFInit);
        if (!bValid)
            return false;

        ////////////////////////////////

        mnScaleRefined = s_refined;

        // write p_bc_refined.txt and check convergence
        {
            cv::Mat pbc_refined = -Rcbstar.t()*pcb_refined;   // use estimated Rcbstar
            fp_bc_refined << pNewestKF->mTimeStamp<<" " << pbc_refined.at<float>(0) << " " << pbc_refined.at<float>(1) << " " << pbc_refined.at<float>(2) << endl;
            fcalibrated_errors_pbc << pNewestKF->mTimeStamp<<" " << gt_x- pbc_refined.at<float>(0) << " " << gt_y-pbc_refined.at<float>(1) << " " << gt_z-pbc_refined.at<float>(2) << endl;

            // check whether scale estimation has converge
            {
                mvRecordEstimatedScale.push_back(s_refined);
                mvRecordEstimatedScaleTimeStamps.push_back(pNewestKF->mTimeStamp);
                mbVINSInitScaleConverged = CheckScaleEstimationConverge();
            }

            // check whether the Pbc estimation has converge
            {
                // record estimated results and timestamps
                mvRecordEstimatedPbc.push_back(Converter::toVector3d(pbc_refined));
                mvRecordEstimatedPbcTimeStamps.push_back(pNewestKF->mTimeStamp);
                mvRecordEstimatedPbcKFPose.push_back(pNewestKF->GetPose());

                mbVINSInitPbcConverged = CheckPbcEstimationConverge();
            }
        }

    }   // End of Step 1.4: Accelerometer Bias Estimation, and Scale, Gravity Direction and Pcb Refinement


    // Debug log, write the result into txt
    {
        // cout<<"Time: "<<pNewestKF->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;

        fgw<<pNewestKF->mTimeStamp<<" "
           <<gw_refined.at<float>(0)<<" "<<gw_refined.at<float>(1)<<" "<<gw_refined.at<float>(2)<<" "
           <<gwstar.at<float>(0)<<" "<<gwstar.at<float>(1)<<" "<<gwstar.at<float>(2)<<" "<<endl;

        fscale<<pNewestKF->mTimeStamp<<" "
              <<s_refined<<" "<<sstar<<" "<<endl;
        fbiasa<<pNewestKF->mTimeStamp<<" "
              <<biasa_.at<float>(0)<<" "<<biasa_.at<float>(1)<<" "<<biasa_.at<float>(2)<<" "<<endl;

        fbiasg<<pNewestKF->mTimeStamp<<" "
              <<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;

        // write processing time into file
        fProcessing_Time << pNewestKF->mTimeStamp << " " << timerOffProcessing_Time.runTime_ms() << endl;
    }


    // ********************************
    bool bVIOInited = false;

    // Criteria: if Rbc and Pbc are both convergent, then VIO has been successfully initialized.
    if(mbVINSInitRbcConverged && mbVINSInitPbcConverged && mbVINSInitScaleConverged)
//    if(pNewestKF->mTimeStamp - mnStartTime >= 50.0)
    {
        bVIOInited = true;
        // Debug log
        std::cout << GREEN"Estimated Rbc, Pbc and scale are both convergent." << RESET << std::endl;
        std::cout << "mvRecordEstimatedPbc.size() = " << mvRecordEstimatedPbc.size() << std::endl;
    }


    if(bVIOInited)  // successful initialization
    {
       // Set NavState , scale and bias for all KeyFrames
       // Scale
       double scale = s_refined;
       mnVINSInitScale = s_refined;
       // gravity vector in world frame
       cv::Mat gw = gw_refined.clone();
       mGravityVec = gw_refined.clone();
       Vector3d gweig = Converter::toVector3d(gw_refined);

       // mVINSInitBiasg and mVINSInitBiasa
       Eigen::Vector3d biasa_eig = Converter::toVector3d(biasa_);

       mVINSInitBiasg = bgest;
       mVINSInitBiasa = biasa_eig;

       // mVINSInitRbc and mVINSInitPbc
       mVINSInitRbc = Rcbstar.t();
       mVINSInitPbc = -Rcbstar.t()*pcb_refined;

       // mVINSInitTbc
       mVINSInitTbc = cv::Mat::eye(4,4,CV_32F);
       mVINSInitRbc.copyTo(mVINSInitTbc.rowRange(0,3).colRange(0,3));
       mVINSInitPbc.copyTo(mVINSInitTbc.rowRange(0,3).col(3));



       // Update NavState for the KeyFrame that is not in vScaleGravityKF
       // Update Tcw-type pose for these KeyFrames. Need mutex lock
       {
           // wait until all the KeyFrames appended in the mlNewKeyFrames have been processed
           while (KeyframesInQueue()>0)
               usleep(1000);

           // Stop local mapping
           RequestStop(); // "Local Mapping STOP"

           // Wait until Local Mapping has effectively stopped
           LOG(INFO) << "Wait until Local Mapping has effectively stopped";
           while(!isStopped() && !isFinished())
               usleep(500);

           // wait until tracking is idle
           LOG(INFO) << "LocalMapping VIO init: wait until tracking is idle";
           while(mpTracker->mbTrackMonoVIOIsBusy)               
               usleep(500);

           LOG(INFO) << "SetUpdatingInitPoses(true);";
           SetUpdatingInitPoses(true);
       }

       {
           unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

           LOG(INFO) << "Velocity Estimation";

           int cnt = 0;
           for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++, cnt++)
           {
               KeyFrame* pKF = *vit;
               if(pKF->isBad()) continue;
               if(pKF!=vScaleGravityKF[cnt]) cerr<<"pKF!=vScaleGravityKF[cnt], id: "<<pKF->mnId<<" != "<<vScaleGravityKF[cnt]->mnId<<endl;
               // Position and rotation of visual SLAM
               cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
               cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
               // Set position and rotation of navstate
               cv::Mat wPb = scale*wPc + Rwc*pcb_refined;   // pcb
               pKF->SetNavStatePos(Converter::toVector3d(wPb));
               pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcbstar)); // Rcb
               // Update bias of Gyr & Acc
               pKF->SetNavStateBiasGyr(bgest);
               pKF->SetNavStateBiasAcc(biasa_eig);
               // Set delta_bias to zero. (only updated during optimization)
               pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
               pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
               /// Step 4. IV-D. Velocity Estimation
               // compute velocity
               if(pKF != vScaleGravityKF.back())
               {
                   KeyFrame* pKFnext = pKF->GetNextKeyFrame();
                   if(!pKFnext) cerr<<"pKFnext is NULL, cnt="<<cnt<<", pKFnext:"<<pKFnext<<endl;
                   if(pKFnext!=vScaleGravityKF[cnt+1]) cerr<<"pKFnext!=vScaleGravityKF[cnt+1], cnt="<<cnt<<", id: "<<pKFnext->mnId<<" != "<<vScaleGravityKF[cnt+1]->mnId<<endl;
                   // IMU pre-int between pKF ~ pKFnext
                   const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                   // Time from this(pKF) to next(pKFnext)
                   double dt = imupreint.getDeltaTime();                                       // deltaTime
                   cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
                   cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
                   cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);           // wPc next
                   cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);    // Rwc next

                   cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb_refined + Rwc*Rcbstar*(dp + Jpba*biasa_) + 0.5*gw*dt*dt );   // Rcb, pcb
                   Eigen::Vector3d veleig = Converter::toVector3d(vel);
                   pKF->SetNavStateVel(veleig);
               }
               else
               {
//                   cerr<<"-----------here is the last KF in vScaleGravityKF------------"<<endl;
                   // If this is the last KeyFrame, no 'next' KeyFrame exists
                   KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
                   if(!pKFprev) cerr<<"pKFprev is NULL, cnt = " << cnt<<endl;
                   if(pKFprev!=vScaleGravityKF[cnt-1]) cerr<<"pKFprev!=vScaleGravityKF[cnt-1], cnt="<<cnt<<", id: "<<pKFprev->mnId<<" != "<<vScaleGravityKF[cnt-1]->mnId<<endl;
                   const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
                   double dt = imupreint_prev_cur.getDeltaTime();
                   Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                   Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                   //
                   Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                   Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                   Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*biasa_eig );
                   pKF->SetNavStateVel(veleig);
               }
           }
           LOG(INFO) << "Re-compute IMU pre-integration at last";

           // Re-compute IMU pre-integration at last. Should after usage of pre-int measurements.
           for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; vit++)
           {
               KeyFrame* pKF = *vit;
               if(pKF->isBad()) continue;
               pKF->ComputePreInt();
           }

           LOG(INFO) << "update all map keyframe poses";
           /// Update pose (multiply metric scale)
           // update all map keyframe poses
           vector<KeyFrame*> mvpKeyFrames = mpMap->GetAllKeyFrames();
           for(std::vector<KeyFrame*>::iterator vit=mvpKeyFrames.begin(), vend=mvpKeyFrames.end(); vit!=vend; vit++)
           {
               KeyFrame* pKF = *vit;
               cv::Mat Tcw = pKF->GetPose();
               cv::Mat tcw = Tcw.rowRange(0,3).col(3)*scale;
               tcw.copyTo(Tcw.rowRange(0,3).col(3));
               pKF->SetPose(Tcw);
           }

           LOG(INFO) << "update all map points";
           // update all map points
           vector<MapPoint*> mvpMapPoints = mpMap->GetAllMapPoints();
           for(std::vector<MapPoint*>::iterator vit=mvpMapPoints.begin(),vend=mvpMapPoints.end(); vit!=vend; vit++)
           {
               MapPoint* pMP = *vit;
               pMP->UpdateScale(scale);
           }
           std::cout<<std::endl<<"... Map scale updated ..."<<std::endl<<std::endl;


           /// Update NavStates
           if(pNewestKF!=mpCurrentKeyFrame) // New keyframe has been inserted into LocalMapping after starting initialization
           {
               cout << "New keyframe has been inserted into LocalMapping after starting initialization" << endl;
               KeyFrame* pKF;

               // Step 1. bias & d_bias
               pKF = pNewestKF;
               do
               {
                   pKF = pKF->GetNextKeyFrame();

                   // Update bias of Gyr & Acc
                   pKF->SetNavStateBiasGyr(bgest);
                   pKF->SetNavStateBiasAcc(biasa_eig);
                   // Set delta_bias to zero. (only updated during optimization)
                   pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
                   pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());

               }while(pKF!=mpCurrentKeyFrame);

               // Step 2. re-compute pre-integration
               pKF = pNewestKF;
               do
               {
                   pKF = pKF->GetNextKeyFrame();
                   pKF->ComputePreInt();

               }while(pKF!=mpCurrentKeyFrame);

               // Step 3. update pos/rot
               pKF = pNewestKF;
               do
               {
                   pKF = pKF->GetNextKeyFrame();

                   // Update rot/pos
                   // Position and rotation of visual SLAM
                   cv::Mat wPc = pKF->GetPoseInverse().rowRange(0,3).col(3);                   // wPc
                   cv::Mat Rwc = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);            // Rwc
                   cv::Mat wPb = wPc + Rwc*pcb_refined; // pcb
                   pKF->SetNavStatePos(Converter::toVector3d(wPb));
                   pKF->SetNavStateRot(Converter::toMatrix3d(Rwc*Rcbstar)); // Rcb

                   // compute velocity
                   if(pKF != mpCurrentKeyFrame)
                   {
                       KeyFrame* pKFnext = pKF->GetNextKeyFrame();
                       // IMU pre-int between pKF ~ pKFNext
//                       const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                       // Time from this(pKF) to net(pKFNext)
                       // IMU pre-int between pKF ~ pKFnext
                       const IMUPreintegrator& imupreint = pKFnext->GetIMUPreInt();
                       // Time from this(pKF) to next(pKFnext)
                       double dt = imupreint.getDeltaTime();                                       // deltaTime
                       cv::Mat dp = Converter::toCvMat(imupreint.getDeltaP());       // deltaP
                       cv::Mat Jpba = Converter::toCvMat(imupreint.getJPBiasa());    // J_deltaP_biasa
                       cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0,3).col(3);           // wPc next
                       cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);    // Rwc next

                       // note that camera pose has been updated
                       // cv::Mat vel = - 1./dt*( scale*(wPc - wPcnext) + (Rwc - Rwcnext)*pcb_refined + Rwc*Rcbstar*(dp + Jpba*biasa_) + 0.5*gw*dt*dt ); // Rcb, pcb
                       cv::Mat vel = - 1./dt*( (wPc - wPcnext) + (Rwc - Rwcnext)*pcb_refined + Rwc*Rcbstar*(dp + Jpba*biasa_) + 0.5*gw*dt*dt ); // Rcb, pcb
                       Eigen::Vector3d veleig = Converter::toVector3d(vel);
                       pKF->SetNavStateVel(veleig);
                   }
                   else
                   {
                       // If this is the last KeyFrame, no 'next' KeyFrame exists
                       KeyFrame* pKFprev = pKF->GetPrevKeyFrame();
                       if(!pKFprev) cerr<<"pKFprev is NULL, cnt = " << cnt<<endl;
                       if(pKFprev!=vScaleGravityKF[cnt-1]) cerr<<"pKFprev!=vScaleGravityKF[cnt-1], cnt="<<cnt<<", id: "<<pKFprev->mnId<<" != "<<vScaleGravityKF[cnt-1]->mnId<<endl;
                       const IMUPreintegrator& imupreint_prev_cur = pKF->GetIMUPreInt();
                       double dt = imupreint_prev_cur.getDeltaTime();
                       Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                       Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                       //
                       Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                       Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                       Eigen::Vector3d veleig = velpre + gweig*dt + rotpre*( dv + Jvba*biasa_eig );
                       pKF->SetNavStateVel(veleig);
                   }

               }while(pKF!=mpCurrentKeyFrame);

           }

           std::cout<<std::endl<<"... Map NavState updated ..."<<std::endl<<std::endl;

           LOG(INFO) << "SetInitKeyFrame(). ";
           SetInitKeyFrame(mpCurrentKeyFrame);

           SetFirstVINSInited(true);
           SetVINSInited(true);
           SetMapUpdateFlagInTracking(true);
           std::cout << "mpCurrentKeyFrame->mnId = " << mpCurrentKeyFrame->mnId
                     << ", mTimeStample = " << std::to_string(mpCurrentKeyFrame->mTimeStamp) << std::endl;

       }
       LOG(INFO) << "SetUpdatingInitPoses(false);";
       SetUpdatingInitPoses(false);

       // "Local Mapping RELEASE"
       Release();

       /// Run global BA after init
       Timer timerOfGBAForInit;
       std::cout << YELLOW"Running global BA after VIO init ..." << RESET << std::endl;
       mbGBAForVIOInitIsRunning = true;

       KeyFrame* pNewestKFBeforeBA = mpMap->GetAllKeyFrames().back();

       std::cout << "before gba, mnVINSInitScale = " << mnVINSInitScale << ", pbc = " << mVINSInitTbc.rowRange(0,3).col(3).t() << std::endl;
       unsigned long nGBAKF = mpCurrentKeyFrame->mnId;
       double scale_ratio;
//       Optimizer::GlobalBundleAdjustmentNavStateWithTbcAndScale(mpMap,mGravityVec, mVINSInitTbc, mnVINSInitScale, 10,NULL,nGBAKF,false, scale_ratio, this, mpTracker);
       Optimizer::GlobalBundleAdjustmentNavStateWithTbcAndScale(mpMap,mGravityVec, mVINSInitTbc, mnVINSInitScale, 10,NULL,nGBAKF,true, scale_ratio, this, mpTracker);

       std::cout << "after gba, mnVINSInitScale = " << mnVINSInitScale << ", pbc = "<< mVINSInitTbc.rowRange(0,3).col(3).t() << std::endl;


       std::cout << BOLDGREEN"finish GlobalBundleAdjustmentNavStateWithTbcAndScale() after VIO init" << RESET << std::endl;
       std::cout << "time of GBAForInit = " << timerOfGBAForInit.runTime_ms() << std::endl;


       { // write to .txt
           // pbc
           cv::Mat pbc_refined = mVINSInitTbc.rowRange(0,3).col(3);
           fp_bc_refined << pNewestKFBeforeBA->mTimeStamp<<" "
                          << pbc_refined.at<float>(0) << " " << pbc_refined.at<float>(1) << " " << pbc_refined.at<float>(2) << endl;

           fcalibrated_errors_pbc << pNewestKFBeforeBA->mTimeStamp<<" "
                                  << gt_x- pbc_refined.at<float>(0) << " " << gt_y-pbc_refined.at<float>(1) << " " << gt_z-pbc_refined.at<float>(2) << endl;

           // Rbc
           Eigen::Matrix3d Rbc_gba_matrix = Converter::toMatrix3d(mVINSInitTbc.rowRange(0,3).colRange(0,3));
           Eigen::Vector3d euler_bc_gba =  Converter::toEulerAngles(Rbc_gba_matrix, true); //Rbc_est_matrix.eulerAngles(2, 1, 0);   // yaw, pitch, roll
           fR_bc_appro << pNewestKF->mTimeStamp<<" " << euler_bc_gba(0) << " " << euler_bc_gba(1) << " " << euler_bc_gba(2) << endl;
           fcalibrated_errors_Rbc << pNewestKF->mTimeStamp<<" " << gt_yaw-euler_bc_gba(0) << " " << gt_pitch-euler_bc_gba(1) << " " << gt_roll-euler_bc_gba(2) << endl;
       }


       {
           // Update NavState for the KeyFrame that is not in vScaleGravityKF
           // Update Tcw-type pose for these KeyFrames. Need mutex lock
           {
               // wait until all the KeyFrames appended in the mlNewKeyFrames have been processed
               while(KeyframesInQueue()>0)
               {
                   LOG(INFO) << "wait until all the KeyFrames appended in the mlNewKeyFrames have been processed";
                   usleep(1000);
               }

               // Stop local mapping
               RequestStop(); // "Local Mapping STOP"

               // Wait until Local Mapping has effectively stopped
               LOG(INFO) << BLUE"Wait until Local Mapping has effectively stopped" << RESET << std::endl;
               while(!isStopped() && !isFinished())
                   usleep(500);

               if(KeyframesInQueue()>0)
                   LOG(ERROR) << "KeyframesInQueue()>0, why???" << "KeyframesInQueue() = " << KeyframesInQueue();

               // wait until tracking is idle since we will update camera pose
               LOG(INFO) << "LocalMapping VIO init: wait until tracking is idle" << std::endl;
               while(mpTracker->mbTrackMonoVIOIsBusy)
                   usleep(500);
           }


           LOG(INFO) << "SetUpdatingInitPoses(true);";
           SetUpdatingInitPoses(true);

           KeyFrame* maxidkf = mpCurrentKeyFrame;
           LOG(INFO) << CYAN"maxidkf->mnId = " << maxidkf->mnId << ", pNewestKFBeforeBA->mnId = " << pNewestKFBeforeBA->mnId << RESET;
           cv::Mat cvTbc = mVINSInitTbc;    // mpParams->GetMatTbc();
           {
               unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

               LOG(INFO) << "Correct keyframes starting at map first keyframe";
               // Correct keyframes starting at map first keyframe
               list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

               while(!lpKFtoCheck.empty())
               {
                   KeyFrame* pKF = lpKFtoCheck.front();
                   const set<KeyFrame*> sChilds = pKF->GetChilds();
                   cv::Mat Twc = pKF->GetPoseInverse();
                   for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                   {
                       KeyFrame* pChild = *sit;
                       if(pChild->mnBAGlobalForKF!=nGBAKF)
                       {
                           if (pChild->mnId > maxidkf->mnId)
                               maxidkf = pChild;


                           cv::Mat Tchildc = pChild->GetPose()*Twc;
                           pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                           pChild->mnBAGlobalForKF=nGBAKF;

                           // Set NavStateGBA and correct the P/V/R
                           pChild->mNavStateGBA = pChild->GetNavState();
                           cv::Mat TwbGBA = Converter::toCvMatInverse(cvTbc*pChild->mTcwGBA);
                           Matrix3d RwbGBA = Converter::toMatrix3d(TwbGBA.rowRange(0,3).colRange(0,3));
                           Vector3d PwbGBA = Converter::toVector3d(TwbGBA.rowRange(0,3).col(3));
                           Matrix3d Rw1 = pChild->mNavStateGBA.Get_RotMatrix();
                           Vector3d Vw1 = pChild->mNavStateGBA.Get_V();
                           Vector3d Vw2 = RwbGBA*Rw1.transpose()*Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1
                           pChild->mNavStateGBA.Set_Pos(PwbGBA);
                           pChild->mNavStateGBA.Set_Rot(RwbGBA);
                           pChild->mNavStateGBA.Set_Vel(Vw2);

                           lpKFtoCheck.push_back(pChild);   // right
                       }
//                       lpKFtoCheck.push_back(pChild);     // wrong
                   }

                   pKF->mTcwBefGBA = pKF->GetPose();
                   //pKF->SetPose(pKF->mTcwGBA);
                   pKF->mNavStateBefGBA = pKF->GetNavState();
                   pKF->SetNavState(pKF->mNavStateGBA);
                   pKF->UpdatePoseFromNS(cvTbc);

                   lpKFtoCheck.pop_front();

                   // log
                   if(pKF->mnId > maxidkf->mnId && !pKF->GetVINSInitedState())
                       LOG(INFO) << "pKF->GetVINSInitedState() is false, why ????" << " pKF->mnId = " << pKF->mnId;

               }

               LOG(INFO) << "Correct MapPoints";
               // Correct MapPoints
               const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

               for(size_t i=0; i<vpMPs.size(); i++)
               {
                   MapPoint* pMP = vpMPs[i];

                   if(pMP->isBad())
                       continue;

                   if(pMP->mnBAGlobalForKF==nGBAKF)
                   {
                       // If optimized by Global BA, just update
                       pMP->SetWorldPos(pMP->mPosGBA);
                   }
                   else
                   {
                       // Update according to the correction of its reference keyframe
                       KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                       if(pRefKF->mnBAGlobalForKF!=nGBAKF)
                           continue;

                       // Map to non-corrected camera
                       cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                       cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                       cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                       // Backproject using corrected camera
                       cv::Mat Twc = pRefKF->GetPoseInverse();
                       cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                       cv::Mat twc = Twc.rowRange(0,3).col(3);

                       pMP->SetWorldPos(Rwc*Xc+twc);
                   }
               }

               std::cout << "... Map updated! ..." << std::endl << std::endl;

               // Map updated, set flag for Tracking
               SetMapUpdateFlagInTracking(true);

               SetGBAFinishTimestample(maxidkf->mTimeStamp);

               std::cout << YELLOW"maxidkf->mTimeStamp = " << std::to_string(maxidkf->mTimeStamp) << RESET << std::endl;

               // Release LocalMapping

               Release();
           }
           LOG(INFO) << "SetUpdatingInitPoses(false);";
           SetUpdatingInitPoses(false);



       }
       mbGBAForVIOInitIsRunning = false;
       SetFlagInitGBAFinish(true);

    }

//    SetFlagCopyInitKFs(false);

    for(int i=0; i<N; i++)
    {
        if(vKFInit[i])
            delete vKFInit[i];
    }

    // Debug Log
    if(bVIOInited && !GetResetVINSInitRequested())
    {
        // file close
        {
            cout << "file closing ..." << endl;
            fgw.close();
            fscale.close();
            fbiasa.close();
            fcondnum.close();
            ftime.close();
            fbiasg.close();

            fR_bc_appro.close();
            fp_bc_appro.close();
            fp_bc_refined.close();

            fp_bc_appro_without_weight.close();
            fR_bc_appro_without_weight.close();
            fcalibrated_errors_Rbc.close();
            fcalibrated_errors_pbc.close();

            fProcessing_Time.close();
        }

        std::cout<<"Time: "<<mpCurrentKeyFrame->mTimeStamp - mnStartTime<<", sstar: "<<sstar<<", s: "<<s_refined<<std::endl;
        LOG(INFO) << BOLDGREEN"[INFO] VIO inited Success ! mnCameraID = " << mpTracker->mnCameraID << RESET << std::endl;
    }

    return bVIOInited;*/

}
