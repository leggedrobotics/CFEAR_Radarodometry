#include "cfear_radarodometry/odometrykeyframefuser.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace CFEAR_Radarodometry {

OdometryKeyframeFuser::OdometryKeyframeFuser(const Parameters& pars, rclcpp::Node* node) : par(pars), node_(node) {
  assert (!par.scan_registered_latest_topic.empty() && !par.scan_registered_keyframe_topic.empty() && !par.odom_latest_topic.empty() && !par.odom_keyframe_topic.empty() );
  assert(par.res>0.05 && par.submap_scan_size>=1 );
  radar_reg = boost::shared_ptr<CFEAR_Radarodometry::n_scan_normal_reg>(new n_scan_normal_reg(Str2Cost(par.cost_type),
                                                                                              Str2loss(par.loss_type_),
                                                                                              par.loss_limit_,
                                                                                              par.weight_opt));

  radar_reg->SetD2dPar(par.covar_scale_,par.regularization_);

  Tprev_fused = Eigen::Affine3d::Identity();
  Tcurrent = Eigen::Affine3d::Identity();
  cov_current = Covariance::Identity();
  T_prev = Eigen::Affine3d::Identity();
  Tmot = Eigen::Affine3d::Identity();
  Tsensor = 1.0/std::max(par.rotation_rate_hz, 0.1);

  if(node_ != nullptr){
    pose_current_publisher = node_->create_publisher<nav_msgs::msg::Odometry>(par.odom_latest_topic,50);
    pose_keyframe_publisher = node_->create_publisher<nav_msgs::msg::Odometry>(par.odom_keyframe_topic,50);
    pubsrc_cloud_latest = node_->create_publisher<sensor_msgs::msg::PointCloud2>(par.scan_registered_latest_topic, 10);
    pub_cloud_keyframe = node_->create_publisher<sensor_msgs::msg::PointCloud2>(par.scan_registered_keyframe_topic, 10);
    if(par.publish_tf_)
      Tbr = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  }
}
/*OdometryKeyframeFuser::~OdometryKeyframeFuser(){
  //cout<<"destruct fuser"<<endl;
  keyframes_.clear();
  ///  cout<<"destructed fuser"<<endl;
}*/



bool OdometryKeyframeFuser::KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist, double min_keyframe_rot_deg){

  if(!use_keyframe)
    return true;
  Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0,1,2);
  CFEAR_Radarodometry::normalizeEulerAngles(Tmotion_euler);
  bool fuse_frame = false;
  //cout<<"diff (trans[m]/rot[deg])=("<<diff.translation().norm()<<"/"<<diff.rotation().eulerAngles(0,1,2).norm()*180.0/M_PI<<") limit=("<<min_keyframe_dist_<<"/"<<min_keyframe_rot_deg_<<")"<<endl;
  if(diff.translation().norm()>min_keyframe_dist || Tmotion_euler.norm()>(min_keyframe_rot_deg*M_PI/180.0))
    fuse_frame = true;
  return fuse_frame;
}


bool OdometryKeyframeFuser::AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev, const Eigen::Affine3d& Tmot_curr) {
  const double dt = Tsensor; // scan period, e.g. 0.25 s at 4 Hz
  const double vel_limit = 200; // m/s
  const double acc_limit = 200; // m/s
  const double vel = (Tmot_curr.translation()/dt).norm();
  const double acc = ((Tmot_curr.translation() - Tmot_prev.translation())/(dt*dt)).norm();

  //cout<<"vel: "<<vel<<", acc: "<<acc<<endl;
  if(acc>acc_limit){
    cout<<"acceleration exceeds limit"<<acc<<" > "<<acc_limit<<endl;
    return false;
  }
  else if(vel>vel_limit){
    cout<<"velocity exceeds limit"<<vel<<" > "<<vel_limit<<endl;
    return false;
  }
  else return true;

}


Eigen::Affine3d OdometryKeyframeFuser::Interpolate(const Eigen::Affine3d &T2, double factor, const Eigen::Affine3d &T1) {
  // std::cout<<"factor="<<factor<<std::endl;
  Eigen::Affine3d pose;
  Eigen::Quaterniond q1(T1.linear());
  Eigen::Quaterniond q2(T2.linear());
  pose.linear() = (q1.slerp(factor, q2)).toRotationMatrix();
  Eigen::Vector3d tdiff = T2.translation() - T1.translation();
  pose.translation() = T1.translation() + tdiff * factor;
  return pose;
}
pcl::PointXYZI OdometryKeyframeFuser::Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p){
  Eigen::Vector3d v(p.x,p.y,p.z);
  Eigen::Vector3d v2 = T*v;
  pcl::PointXYZI p2;
  p2.x = v2(0);
  p2.y = v2(1);
  p2.z = v2(2);
  p2.intensity = p.intensity;
  return p2;
}
nav_msgs::msg::Odometry OdometryKeyframeFuser::FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, const rclcpp::Time& t, Matrix6d& Cov){
  nav_msgs::msg::Odometry odom_msg;

  //double d = Tmot.translation().norm();
  Eigen::MatrixXd cov_1_36(Cov);
  //reg_cov(0,0) = reg_cov(1,1) = (d*0.02)*(d*0.02);
  //reg_cov(5,5) = (d*0.005)*(d*0.005);
  cov_1_36.resize(1,36);
  for(int i=0;i<36;i++){
    odom_msg.pose.covariance[i] = cov_1_36.data()[i];
  }

  odom_msg.header.stamp = t;
  odom_msg.header.frame_id = par.odometry_link_id;
  odom_msg.child_frame_id = par.child_frame_id;
  odom_msg.pose.pose = tf2::toMsg(Eigen::Isometry3d(T.matrix()));
  return odom_msg;
}
pcl::PointCloud<pcl::PointXYZI> OdometryKeyframeFuser::FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in, Eigen::Affine3d& T){
  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  pcl::transformPointCloud(cloud_in, cloud_out, T);
  cloud_out.header.frame_id = par.odometry_link_id;
  cloud_out.header.stamp = cloud_in.header.stamp;
  return cloud_out;
}

void OdometryKeyframeFuser::processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const rclcpp::Time& t) {

  const auto t0 = std::chrono::steady_clock::now();
  Eigen::Affine3d TprevMot(Tmot);
  if(par.compensate){
    Compensate(*cloud, TprevMot, par.radar_ccw);
    Compensate(*cloud_peaks, TprevMot, par.radar_ccw);
  }


  std::vector<Matrix6d> cov_vek;
  Covariance cov_sampled;
  std::vector<CFEAR_Radarodometry::MapNormalPtr> scans_vek;
  std::vector<Eigen::Affine3d> T_vek;
  const auto t1 = std::chrono::steady_clock::now();

  //std::cout << "par.res: " << par.res << std::endl;

  CFEAR_Radarodometry::MapNormalPtr Pcurrent = CFEAR_Radarodometry::MapNormalPtr(new MapPointNormal(cloud, par.res, Eigen::Vector2d(0,0), par.weight_intensity_, par.use_raw_pointcloud));

  const auto t2 = std::chrono::steady_clock::now();
  Eigen::Affine3d Tguess;
  if(par.use_guess)
    Tguess = T_prev*TprevMot;
  else
    Tguess = T_prev;


  if(keyframes_.empty()){
    scan_ = RadarScan(Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), cloud_peaks, cloud, Pcurrent, t);
    AddToGraph(keyframes_, scan_, Eigen::Matrix<double,6,6>::Identity());
    updated = true;
    AddToReference(keyframes_, scan_, par.submap_scan_size);
    return;
  }
  else
    FormatScans(keyframes_, Pcurrent, Tguess, cov_vek, scans_vek, T_vek);

  //Only for generating plots


  bool success = true;
  if(!par.disable_registration)
    success = radar_reg->Register(scans_vek, T_vek, cov_vek, par.soft_constraint);

  const auto t3 = std::chrono::steady_clock::now();

  if(success==false){
    // Online this must be recoverable: fall back to the constant-velocity prediction
    // instead of terminating (the ROS1 offline pipeline called exit(0) here).
    nr_reg_failures_++;
    cerr<<"registration failure ("<<nr_reg_failures_<<" total) - falling back to motion prediction"<<endl;
    T_vek.back() = Tguess;
    cov_vek.back() = Identity66;
  }

  Tcurrent = T_vek.back();
  cov_current = cov_vek.back();
  Eigen::Affine3d Tmot_current = T_prev.inverse()*Tcurrent;
  if(!AccelerationVelocitySanityCheck(Tmot, Tmot_current))
    Tcurrent = Tguess; //Insane acceleration and speed, lets use the guess.
  Tmot = T_prev.inverse()*Tcurrent;

  // Try to approximate the covariance by the cost-sampling approach
  if(par.estimate_cov_by_sampling){
      if(approximateCovarianceBySampling(scans_vek, T_vek, cov_sampled)){ // if success, cov_sampled contains valid data
          cov_current = cov_sampled;
          cov_vek.back() = cov_sampled;
      }
  }

  nav_msgs::msg::Odometry msg_current = FormatOdomMsg(Tcurrent, Tmot, t, cov_current);
  if(node_ != nullptr){
    if(par.visualize){
      pcl::PointCloud<pcl::PointXYZI> cld_latest = FormatScanMsg(*cloud, Tcurrent);
      sensor_msgs::msg::PointCloud2 cld_latest_msg;
      pcl::toROSMsg(cld_latest, cld_latest_msg);
      pubsrc_cloud_latest->publish(cld_latest_msg);
    }
    pose_current_publisher->publish(msg_current);
    if(par.publish_tf_ && Tbr != nullptr){
      geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(Eigen::Isometry3d(Tcurrent.matrix()));
      transformStamped.header.stamp = msg_current.header.stamp;
      transformStamped.header.frame_id = par.odometry_link_id;
      transformStamped.child_frame_id = par.child_frame_id;
      Tbr->sendTransform(transformStamped);
    }
  }

  const Eigen::Affine3d Tkeydiff = keyframes_.back().GetPose().inverse()*Tcurrent;
  bool fuse = KeyFrameBasedFuse(Tkeydiff, par.use_keyframe, par.min_keyframe_dist_, par.min_keyframe_rot_deg_);


  CFEAR_Radarodometry::timing.Document("velocity", Tmot.translation().norm()/Tsensor);


  if(success && fuse){
    //cout << "fuse" <<endl;
    distance_traveled += Tkeydiff.translation().norm();
    Tprev_fused = Tcurrent;
    if(node_ != nullptr){
      nav_msgs::msg::Odometry msg_keyframe = FormatOdomMsg(Tcurrent, Tkeydiff, t, cov_vek.back());
      pose_keyframe_publisher->publish(msg_keyframe);
      if(par.visualize){
        pcl::PointCloud<pcl::PointXYZI> cld_keyframe = FormatScanMsg(*cloud, Tcurrent);
        sensor_msgs::msg::PointCloud2 cld_keyframe_msg;
        pcl::toROSMsg(cld_keyframe, cld_keyframe_msg);
        pub_cloud_keyframe->publish(cld_keyframe_msg);
      }
    }

    frame_nr_++;
    scan_ = RadarScan(Tcurrent, TprevMot, cloud_peaks, cloud, Pcurrent, t);
    AddToGraph(keyframes_, scan_, cov_current);
    AddToReference(keyframes_,scan_, par.submap_scan_size);
    updated = true;

  }
  //else
    //cout << "no fuse" <<endl;
  const auto t4 = std::chrono::steady_clock::now();
  CFEAR_Radarodometry::timing.Document("compensate", ToMs(t1-t0));
  CFEAR_Radarodometry::timing.Document("build_normals", ToMs(t2-t1));
  CFEAR_Radarodometry::timing.Document("register", ToMs(t3-t2));
  CFEAR_Radarodometry::timing.Document("publish_etc", ToMs(t4-t3));
  T_prev = Tcurrent;

}

bool OdometryKeyframeFuser::approximateCovarianceBySampling(std::vector<CFEAR_Radarodometry::MapNormalPtr> &scans_vek,
                                                            const std::vector<Eigen::Affine3d> &T_vek,
                                                            Covariance &cov_sampled){
  bool cov_sampled_success = true;
  string path;   // for dumping samples to a file
  std::ofstream cov_samples_file;

  std::vector<Eigen::Affine3d> T_vek_copy(T_vek);
  Eigen::Affine3d T_best_guess = T_vek_copy.back();

  if(par.cov_samples_to_file_as_well){
    path = par.cov_sampling_file_directory + string("/cov_samples_") + std::to_string(nr_callbacks_) + string(".csv");
    cov_samples_file.open(path);
  }

  double xy_sample_range = par.cov_sampling_xy_range*0.5; // sampling in x and y axis around the estimated pose
  double theta_range = par.cov_sampling_yaw_range*0.5;  // sampling on the yaw axis plus minus
  unsigned int number_of_steps_per_axis = par.cov_sampling_samples_per_axis;
  unsigned int number_of_samples = number_of_steps_per_axis*number_of_steps_per_axis*number_of_steps_per_axis;

  Eigen::Affine3d sample_T = Eigen::Affine3d::Identity();
  double sample_cost = 0;  // return of the getCost function
  std::vector<double> residuals; // return of the getCost function
  Eigen::VectorXd samples_x_values(number_of_samples);
  Eigen::VectorXd samples_y_values(number_of_samples);
  Eigen::VectorXd samples_yaw_values(number_of_samples);
  Eigen::VectorXd samples_cost_values(number_of_samples);

  std::vector<double> xy_samples = linspace<double>(-xy_sample_range, xy_sample_range, number_of_steps_per_axis);
  std::vector<double> theta_samples = linspace<double>(-theta_range, theta_range, number_of_steps_per_axis);

  // Sample the cost function according to the settings, optionally dump the samples into a file
  int vector_pointer = 0;
  for (int theta_sample_id = 0; theta_sample_id < number_of_steps_per_axis; theta_sample_id++) {
    for (int x_sample_id = 0; x_sample_id < number_of_steps_per_axis; x_sample_id++) {
      for (int y_sample_id = 0; y_sample_id < number_of_steps_per_axis; y_sample_id++) {

        sample_T.translation() = Eigen::Vector3d(xy_samples[x_sample_id],
                                                 xy_samples[y_sample_id],
                                                 0.0) + T_best_guess.translation();
        sample_T.linear() = Eigen::AngleAxisd(theta_samples[theta_sample_id],
                                              Eigen::Vector3d(0.0, 0.0, 1.0)) * T_best_guess.linear();

        T_vek_copy.back() = sample_T;
        radar_reg->GetCost(scans_vek, T_vek_copy, sample_cost, residuals);

        samples_x_values[vector_pointer] = xy_samples[x_sample_id];
        samples_y_values[vector_pointer] = xy_samples[y_sample_id];
        samples_yaw_values[vector_pointer] = theta_samples[theta_sample_id];
        samples_cost_values[vector_pointer] = sample_cost;
        vector_pointer ++;

        if(par.cov_samples_to_file_as_well) {
            cov_samples_file << xy_samples[x_sample_id] << " " << xy_samples[y_sample_id] <<
                             " " << theta_samples[theta_sample_id] << " " << sample_cost << std::endl;
        }
      }
    }
  }
  if(cov_samples_file.is_open()) cov_samples_file.close();

  //Find the approximating quadratic function by linear least squares
  // f(x,y,z) = ax^2 + by^2 + cz^2 + dxy + eyz + fzx + gx+ hy + iz + j
  // A = [x^2, y^2, z^2, xy, yz, zx, x, y, z, 1]
  Eigen::MatrixXd A(number_of_samples, 10);
  A << samples_x_values.array().square().matrix(),
          samples_y_values.array().square().matrix(),
          samples_yaw_values.array().square().matrix(),
          (samples_x_values.array()*samples_y_values.array()).matrix(),
          (samples_y_values.array()*samples_yaw_values.array()).matrix(),
          (samples_yaw_values.array()*samples_x_values.array()).matrix(),
          samples_x_values,
          samples_y_values,
          samples_yaw_values,
          Eigen::MatrixXd::Ones(number_of_samples,1);

  Eigen::VectorXd quad_func_coefs = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(samples_cost_values);

  // With the coefficients, we can contruct the Hessian matrix (constant for a given function)
  Eigen::Matrix3d hessian_matrix;
  hessian_matrix << 2*quad_func_coefs[0],   quad_func_coefs[3],   quad_func_coefs[5],
          quad_func_coefs[3], 2*quad_func_coefs[1],   quad_func_coefs[4],
          quad_func_coefs[5],   quad_func_coefs[4], 2*quad_func_coefs[2];

  // We need to check if the approximating function is convex (all eigs positive, we don't went the zero eigs either)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(hessian_matrix);
  Eigen::Vector3d eigValues;
  Eigen::Matrix3d covariance_3x3;

  if (eigensolver.info() != Eigen::Success) {
    cov_sampled_success = false;
    std::cout << "Covariance sampling warning: Eigenvalues search failed." << std::endl;
  }
  else{
    eigValues = eigensolver.eigenvalues();
    if(eigValues[0] <= 0.0 || eigValues[1] <= 0.0 || eigValues[2] <= 0.0){
        cov_sampled_success = false;
        std::cout << "Covariance sampling warning: Quadratic approximation not convex. Sampling will not be used for this scan." << std::endl;
    }
    else{
      // Compute the covariance from the hessian and scale by the score
      double score_scale = 1.0;
      if(radar_reg->GetCovarianceScaler(score_scale)) {

        covariance_3x3 = 2.0 * hessian_matrix.inverse() * score_scale * par.cov_sampling_covariance_scaler;

        //We need to construct the full 6DOF cov matrix
        cov_sampled = Eigen::Matrix<double, 6, 6>::Identity();
        cov_sampled.block<2, 2>(0, 0) = covariance_3x3.block<2, 2>(0, 0);
        cov_sampled(5, 5) = covariance_3x3(2, 2);
        cov_sampled(0, 5) = covariance_3x3(0, 2);
        cov_sampled(1, 5) = covariance_3x3(1, 2);
        cov_sampled(5, 0) = covariance_3x3(2, 0);
        cov_sampled(5, 1) = covariance_3x3(2, 1);
      }
      else cov_sampled_success = false;
    }
  }
  return cov_sampled_success;
}

void OdometryKeyframeFuser::pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const rclcpp::Time& t){
  const auto t1 = std::chrono::steady_clock::now();
  updated = false;
  this->processFrame(cloud_filtered, cloud_filtered_peaks, t);
  nr_callbacks_++;
  Tcurr = Tcurrent;
  const auto t2 = std::chrono::steady_clock::now();
  CFEAR_Radarodometry::timing.Document("Registration",CFEAR_Radarodometry::ToMs(t2-t1));

}

void OdometryKeyframeFuser::pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const rclcpp::Time& t, Covariance &cov_curr){
    pointcloudCallback(cloud_filtered, cloud_filtered_peaks, Tcurr, t);
    cov_curr = cov_current;
}


void OdometryKeyframeFuser::PrintSurface(const std::string& path, const Eigen::MatrixXd& surface){
  std::ofstream myfile;
  myfile.open (path);
  for(int i=0;i<surface.rows();i++){
    for(int j=0;j<surface.cols();j++){
      if(j==surface.cols()-1)
        myfile<<surface(i,j)<<std::endl;
      else
        myfile<<surface(i,j)<<" ";
    }
  }
  myfile.close();
}

void OdometryKeyframeFuser::AddToGraph(PoseScanVector& reference, RadarScan& scan,  const Eigen::Matrix<double,6,6>& Cov){
  if(frame_nr_ == 0){
    graph_.push_back(std::make_pair(scan,std::vector<Constraint3d>()));
  }
  else{
    std::vector<Constraint3d> constraints;
    for(auto itr = reference.rbegin() ; itr!=reference.rend() ; itr++){
      const Eigen::Affine3d Tfrom = scan.GetPose();
      const Eigen::Affine3d Tto = itr->GetPose();
      Eigen::Affine3d Tdiff = Tfrom.inverse()*Tto;
      Eigen::Matrix<double,6,6> C = Cov;
      C.block<3,3>(0,0) = Tfrom.inverse().rotation()*Cov.block<3,3>(0,0)*Tfrom.inverse().rotation().transpose(); //Change frame to Tprev
      constraints.push_back({scan.idx_, itr->idx_, PoseEigToCeres(Tdiff), C.inverse(), ConstraintType::odometry});
      break;
    }
    graph_.push_back(std::make_pair(scan, constraints));
  }
}
void OdometryKeyframeFuser::SaveGraph(const std::string& path){
  if(par.store_graph)
    CFEAR_Radarodometry::SaveSimpleGraph(path, graph_);
}

void AddToReference(PoseScanVector& reference, RadarScan& scan, size_t submap_scan_size){

  reference.push_back(scan);
  if(reference.size() > submap_scan_size){
    reference.erase(reference.begin());
  }
}

void FormatScans(const PoseScanVector& reference,
                 const CFEAR_Radarodometry::MapNormalPtr& Pcurrent,
                 const Eigen::Affine3d& Tcurrent,
                 std::vector<Matrix6d>& cov_vek,
                 std::vector<MapNormalPtr>& scans_vek,
                 std::vector<Eigen::Affine3d>& T_vek
                 ){

  for (int i=0;i<reference.size();i++) {
    cov_vek.push_back(Identity66);
    scans_vek.push_back(reference[i].cloud_normal_);
    T_vek.push_back(reference[i].GetPose());
  }
  cov_vek.push_back(Identity66);
  scans_vek.push_back(Pcurrent);
  T_vek.push_back(Tcurrent);
}



template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

}
