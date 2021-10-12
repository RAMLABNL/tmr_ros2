#ifdef NO_INCLUDE_DIR
#include "tm_command.h"
#else
#include "tm_driver/tm_command.h"
#endif

#include <sstream>
#include <iomanip>

std::string TmCommand::change_tcp(const std::string &toolname)
{
	std::stringstream ss;
	ss << "ChangeTCP(\"" << toolname << "\")";
	return ss.str();
}
std::string TmCommand::change_tcp(const std::vector<double> &tcp)
{
	std::stringstream ss;
	ss << "ChangeTCP(";
    ss << tcp[0] << "," << tcp[1] << "," << tcp[2] << "," << deg(tcp[3]) << "," << deg(tcp[4])  << "," << deg(tcp[5]);
    ss << ")";
	return ss.str();
}
std::string TmCommand::change_tcp(const std::vector<double> &tcp, double weight)
{
	std::stringstream ss;
	ss << "ChangeTCP(";
    ss << tcp[0] << "," << tcp[1] << "," << tcp[2] << "," << deg(tcp[3]) << "," << deg(tcp[4])  << "," << deg(tcp[5]);
    ss << "," << weight;
    ss << ")";
	return ss.str();
}
std::string TmCommand::change_tcp(const std::vector<double> &tcp, double weight, const std::vector<double> &inertia)
{
	std::stringstream ss;
	ss << "ChangeTCP(";
    ss << tcp[0] << "," << tcp[1] << "," << tcp[2] << "," << deg(tcp[3]) << "," << deg(tcp[4])  << "," << deg(tcp[5]);
    ss << "," << weight;
    ss << "," << inertia[0] << "," << inertia[1] << "," << inertia[2];
    ss << "," << inertia[3] << "," << inertia[4] << "," << inertia[5];
    ss << "," << deg(inertia[6]) << "," << deg(inertia[7]) << "," << deg(inertia[8]);
    ss << ")";
	return ss.str();
}
std::string TmCommand::set_tag(int tag, int wait)
{
	std::stringstream ss;
	ss << "QueueTag(" << tag << "," << wait << ")";
	return ss.str();
}
std::string TmCommand::set_wait_tag(int tag, int timeout_ms)
{
	std::stringstream ss;
	ss << "WaitQueueTag(" << tag << "," << timeout_ms << ")";
	return ss.str();
}
std::string TmCommand::set_io(TmIOModule module, TmIOType type, int pin, float state)
{
	static std::string io_module_name[] = { "ControlBox", "EndModule" };
	static std::string io_type_name[] = { "DI", "DO", "InstantDO", "AI", "AO", "InstantAO" };

	std::string script =
		"IO[" + io_module_name[int(module)]
		+ "]." + io_type_name[int(type)]
		+ "[" + std::to_string(pin) + "]=";
	if (type == TmIOType::DI || type == TmIOType::DO || type == TmIOType::InstantDO) {
		if (state == 0.0f)
			script += "0";
		else
			script += "1";
	}
	else {
		script += std::to_string(state);
	}
	return script;
}
std::string TmCommand::set_joint_pos_PTP(const std::vector<double> &angs,
	int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision)
{
	auto angs_deg = degs(angs);
	int acct_ms = int(1000.0 * acc_time);
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << "PTP(\"JPP\",";
	for (auto &value : angs_deg) { ss << value << ","; }
	ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
	ss << std::boolalpha << fine_goal << ")";
	return ss.str();
}
std::string TmCommand::set_tool_pose_PTP(const std::vector<double> &pose,
	int vel_percent, double acc_time, int blend_percent, bool fine_goal, int precision)
{
	auto pose_mmdeg = mmdeg_pose(pose);
	int acct_ms = int(1000.0 * acc_time);
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << "PTP(\"CPP\",";
	for (auto &value : pose_mmdeg) { ss << value << ","; }
	ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
	ss << std::boolalpha << fine_goal << ")";
	return ss.str();
}
std::string TmCommand::set_tool_pose_Line(const std::vector<double> &pose,
	double vel, double acc_time, int blend_percent, bool fine_goal, int precision)
{
	auto pose_mmdeg = mmdeg_pose(pose);
	int vel_mm = int(1000.0 * vel);
	int acct_ms = int(1000.0 * acc_time);
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << "Line(\"CAP\",";
	for (auto &value : pose_mmdeg) { ss << value << ","; }
	ss << vel_mm << "," << acct_ms << "," << blend_percent << ",";
	ss << std::boolalpha <<fine_goal << ")";
	return ss.str();
}

std::string TmCommand::set_pvt_point(TmPvtMode mode,
	double t, const std::vector<double> &pos, const std::vector<double> &vel, int precision)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << "PVTPoint(";
	if (mode == TmPvtMode::Joint) {
		for (auto &value : pos) { ss << deg(value) << ","; }
		for (auto &value : vel) { ss << deg(value) << ","; }
	}
	else {
		auto pv = mmdeg_pose(pos);
		for (auto &value : pv) { ss << value << ","; }
		auto vv = mmdeg_pose(vel);
		for (auto &value : vv) { ss << value << ","; }
	}
	ss << t << ")";
	return ss.str();
}
std::string TmCommand::set_pvt_traj(const TmPvtTraj &pvts, int precision)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision);
	if (pvts.mode == TmPvtMode::Joint) {
		ss << "PVTEnter(0)\r\n";
		for (auto &point : pvts.points) {
			ss << "PVTPoint(";
			for (auto &value : point.positions) { ss << deg(value) << ","; }
			for (auto &value : point.velocities) { ss << deg(value) << ","; }
			ss << point.time << ")\r\n";
		}
	}
	else {
		ss << "PVTEnter(1)\r\n";
		for (auto &point : pvts.points) {
			ss << "PVTPoint(";
			auto pv = mmdeg_pose(point.positions);
			for (auto &value : pv) { ss << value << ","; }
			auto vv = mmdeg_pose(point.velocities);
			for (auto &value : vv) { ss << value << ","; }
			ss << point.time << ")\r\n";
		}
	}
	ss << "PVTExit()";
	return ss.str();
}