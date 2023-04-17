//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque.h> //library of the computed torque 

namespace panda_controllers{

bool ComputedTorque::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
	this->cvc_nh = node_handle;
	
	std::string arm_id; //checking up the arm id of the robot
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		return false;
	}

	/* Inizializing the Kp and Kv gains */

	double kp1, kp2, kp3, kp4, kp5, kp6, kp7, kv1, kv2, kv3, kv4, kv5, kv6, kv7;

	if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3) 
		|| !node_handle.getParam("kp4", kp4) || !node_handle.getParam("kp5", kp5) || !node_handle.getParam("kp6", kp6) 
		|| !node_handle.getParam("kp7", kp7) 
		|| !node_handle.getParam("kv1", kv1) || !node_handle.getParam("kv2", kv2) || !node_handle.getParam("kv3", kv3) 
		|| !node_handle.getParam("kv4", kv4) || !node_handle.getParam("kv5", kv5) || !node_handle.getParam("kv6", kv6) 
		|| !node_handle.getParam("kv7", kv7) ) {
		ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		return false;
	}

	Kp = Eigen::MatrixXd::Identity(7, 7);
	Kp(0,0) = kp1; Kp(1,1) = kp2; Kp(2,2) = kp3; Kp(3,3) = kp4; Kp(4,4) = kp5; Kp(5,5) = kp6; Kp(6,6) = kp7;
	
	Kv = Eigen::MatrixXd::Identity(7, 7);
	Kv(0,0) = kv1; Kv(1,1) = kv2; Kv(2,2) = kv3; Kv(3,3) = kv4; Kv(4,4) = kv5; Kv(5,5) = kv6; Kv(6,6) = kv7;

	extra_torque << 0, 0, 0, 0, 0, 0, 0;
	
	/* Assigning the time */
   
	if (!node_handle.getParam("dt", dt)) {
		ROS_ERROR("Computed Torque: Could not get parameter dt!");
		return false;
	}

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR("Computed Torque: Error in parsing joints name!");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
		return false;
	}

	try {
		model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
		return false;
	}

	try {
		state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
		return false;
	}

	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
			return false;
		}
	}
	
	/* Initialize joint (torque,velocity) limits */
	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber */
	this->sub_command_ = node_handle.subscribe("command", 1, &ComputedTorque::setCommandCB, this);   //it verify with the callback that the command has been received
	this->sub_command_param_ = node_handle.subscribe("command_param", 1, &ComputedTorque::setCommandParam, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 1);
	
	return true;
}

void ComputedTorque::starting(const ros::Time& time)
{
	/* Getting Robot State in order to get q_curr and dot_q_curr */
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

	/* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Eigen form  */
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

	/* Secure Initialization */
	command_q_d = q_curr;
	command_q_d_old = q_curr;

	command_dot_q_d = dot_q_curr;
	command_dot_q_d_old = dot_q_curr;

	command_dot_dot_q_d.setZero();

	/* Defining the NEW gains */
	Kp_apix = Kp;
	Kv_apix = Kv;

}

void ComputedTorque::update(const ros::Time&, const ros::Duration& period)
{
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	
	/* Actual position and velocity of the joints */

	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Saturate desired velocity to avoid limits */
	for (int i = 0; i < 7; ++i){
		double ith_des_vel = abs(command_dot_q_d(i)/q_dot_limit(i));
		if( ith_des_vel > 1)
		command_dot_q_d = command_dot_q_d / ith_des_vel; 
	}

	// command_dot_dot_q_d = (command_dot_q_d - command_dot_q_d_old) / dt;

	// // Estimate friction
	// std::array<double, 7> gravity_array = model_handle_->getGravity();
	// Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
	// double alpha = 0.02;
	// tau_J = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
	// Eigen::Matrix<double, 7, 1> new_friction = previous_torque - tau_J + gravity;
	// Eigen::Matrix<double, 7, 1> filtered_friction = alpha*new_friction + (1-alpha)*joint_friction;
	// joint_friction = filtered_friction;
	// double friction_compensation = 0.0;
	// Eigen::Matrix<double, 7, 1> current_v = Eigen::Map<Eigen::Matrix<double, 7, 1>>((robot_state.dq).data());
	// if (current_v.isZero()) friction_compensation = 0.3;
	// else friction_compensation = 1.0;
	// Eigen::Matrix<double, 7, 1> friction_correction = friction_compensation*joint_friction;

	/* Computed Torque control law */

	error = command_q_d - q_curr;
	dot_error = command_dot_q_d - dot_q_curr;

	// Publish tracking errors as joint states
	sensor_msgs::JointState error_msg;
	std::vector<double> err_vec(error.data(), error.data() + error.rows()*error.cols());
	std::vector<double> dot_err_vec(dot_error.data(), dot_error.data() + dot_error.rows()*dot_error.cols());
	error_msg.header.stamp = ros::Time::now();
	error_msg.position = err_vec;
	error_msg.velocity = dot_err_vec;
	this->pub_err_.publish(error_msg);

	Kp_apix = Kp;
	Kv_apix = Kv;

	tau_cmd = M * command_dot_dot_q_d + C + Kp_apix * error + Kv_apix * dot_error + extra_torque; // + friction_correction;  // C->C*dq

	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
	previous_torque = tau_cmd;
	
	/* Set the command for each joint */
	for (size_t i = 0; i < 7; i++) {
		joint_handles_[i].setCommand(tau_cmd[i]);
	}
}

void ComputedTorque::stopping(const ros::Time&)
{
	//TO DO
}

/* Check for the effort commanded */
Eigen::Matrix<double, 7, 1> ComputedTorque::saturateTorqueRate(
	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	const Eigen::Matrix<double, 7, 1>& tau_J_d)
{
	Eigen::Matrix<double, 7, 1> tau_d_saturated {};
	for (size_t i = 0; i < 7; i++) {

		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

	}
	return tau_d_saturated;
}

void ComputedTorque::setCommandCB(const panda_controllers::Commands& msg)
{
	if ((msg.joint_commands.position).size() != 7 || (msg.joint_commands.position).empty()) {

		ROS_FATAL("Desired position has not dimension 7 or is empty!", (msg.joint_commands.position).size());
	}

	if ((msg.joint_commands.velocity).size() != 7 || (msg.joint_commands.velocity).empty()) {

		ROS_FATAL("Desired velocity has not dimension 7 or is empty!", (msg.joint_commands.velocity).size());
	}

	// TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryMessage)
	if ((msg.joint_commands.effort).size() != 7 || (msg.joint_commands.effort).empty()) {

		ROS_FATAL("Desired effort (acceleration) has not dimension 7 or is empty!", (msg.joint_commands.effort).size());
	}

	if ((msg.extra_torque).size() != 7) {

		ROS_FATAL("Desired extra_torque has not dimension 7!", (msg.extra_torque).size());
	}
	if (!(msg.extra_torque).empty()) {
		extra_torque = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg.extra_torque).data());
	}

	command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg.joint_commands.position).data());
	command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg.joint_commands.velocity).data());
	command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg.joint_commands.effort).data());

}

void ComputedTorque::setCommandParam(const panda_controllers::CommandParams& msg)
{

	if ((msg.stiffness).size() != 7) {

		ROS_FATAL("Desired stiffness has not dimension 7!", (msg.stiffness).size());
	}

	if ((msg.damping).size() != 7) {

		ROS_FATAL("Desired damping has not dimension 7!", (msg.damping).size());
	}
	
	if (!(msg.stiffness).empty()) {
		Kp(0,0) = msg.stiffness[0]; Kp(1,1) = msg.stiffness[1]; Kp(2,2) = msg.stiffness[2]; 
		Kp(3,3) = msg.stiffness[3]; Kp(4,4) = msg.stiffness[4]; Kp(5,5) = msg.stiffness[5]; 
		Kp(6,6) = msg.stiffness[6];
	}

	if (!(msg.damping).empty()) {
		Kv(0,0) = msg.damping[0]; Kv(1,1) = msg.damping[1]; Kv(2,2) = msg.damping[2]; 
		Kv(3,3) = msg.damping[3]; Kv(4,4) = msg.damping[4]; Kv(5,5) = msg.damping[5]; 
		Kv(6,6) = msg.damping[6];
	}
}


}

PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorque, controller_interface::ControllerBase);
