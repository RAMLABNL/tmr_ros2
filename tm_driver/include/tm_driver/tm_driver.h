#pragma once
#include "tm_communication.h"
#include "tm_svr_communication.h"
#include "tm_sct_communication.h"
#include "tm_command.h"

class TmDriver
{
public:
	TmSvrCommunication svr;
	TmSctCommunication sct;
	TmRobotState &state;

	const TmCommRC RC_OK = TmCommRC::OK;
private:
	std::condition_variable *_svr_cv = nullptr;
	std::condition_variable *_sct_cv = nullptr;
	bool _has_svr_thrd = false;
	bool _has_sct_thrd = false;

	bool _is_executing_traj = false;

	////////////////////////////////
	// tm_driver Param.
	////////////////////////////////

	double _max_velocity = M_PI;
	double _max_tcp_speed = 1.0;
	double _max_payload = 4.0;

public:
	explicit TmDriver(const std::string &ip);
	explicit TmDriver(const std::string &ip,
		std::condition_variable *psvr_cv,
		std::condition_variable *psct_cv);

	// start: connect to server, run project, connect to listen node
	bool start(int timeout_ms = -1, bool stick_play = true);

	// halt: disconnect to listen node, stop project, disconnect to server
	void halt();

	////////////////////////////////
	// tm_driver Param.
	////////////////////////////////

	void set_this_max_velocity(double max_vel) { _max_velocity = max_vel; }
	void set_this_max_tcp_speed(double max_spd) { _max_tcp_speed = max_spd; }
	void set_this_max_payload(double payload) { _max_payload = payload; }

	////////////////////////////////
	// SVR Robot Function (write_XXX)
	////////////////////////////////


	////////////////////////////////
	// SCT Robot Function (set_XXX)
	////////////////////////////////

	bool script_exit(int priority = -1, const std::string &id = "Exit");
	bool set_tag(int tag, int wait = 0, const std::string &id = "Tag");
	bool set_wait_tag(int tag, int timeout_ms = 0, const std::string &id = "WaitTag");
	bool set_stop(int priority = -1, const std::string &id = "Stop");
	bool set_pause(const std::string &id = "Pause");
	bool set_resume(const std::string &id = "Resume");

	bool change_tcp(const std::string &toolname, const std::string &id = "ChangeTCP");
	bool change_tcp(const std::vector<double> &tcp, const std::string &id = "ChangeTCP");
	bool change_tcp(const std::vector<double> &tcp, double weight, const std::string &id = "ChangeTCP");
	bool change_tcp(const std::vector<double> &tcp, double weight, const std::vector<double> &inertia, const std::string &id = "ChangeTCP");

	//enum class IOModule { ControlBox, EndEffector };
	//enum class IOType { DI, DO, InstantDO, AI, AO, InstantAO };
	bool set_io(TmIOModule module, TmIOType type, int pin, float state, const std::string &id = "io");
	bool set_joint_pos_PTP(const std::vector<double> &angs,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPJ");
	bool set_tool_pose_PTP(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "PTPT");
	bool set_tool_pose_Line(const std::vector<double> &pose,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "Line");
	bool set_tool_pose_Line_rel(const std::vector<double> &pose,
		bool tool_frame,
		double vel, double acc_time, int blend_percent, bool fine_goal = false, const std::string &id = "LineRel");
	// set_tool_pose_PLINE

	//
	// PVT Trajectory
	//

	/*enum class PvtMode { Joint, Tool };

	struct PvtPoint {
		double time;
		std::vector<double> positions;
		std::vector<double> velocities;
	};*/

	/*struct PvtTraj {
		PvtMode mode;
		std::vector<double> time_vec;
		std::vector<std::vector<double> > positions_vec;
		std::vector<std::vector<double> > velocities_vec;
	};*/

	/*struct PvtTraj {
		PvtMode mode;
		std::vector<PvtPoint> points;
		double total_time;
	};*/

	bool set_pvt_enter(TmPvtMode mode, const std::string &id = "PvtEnter");
	bool set_pvt_exit(const std::string &id = "PvtExit");
	bool set_pvt_point(TmPvtMode mode,
		double t, const std::vector<double> &pos, const std::vector<double> &vel, const std::string &id = "PvtPt");
	bool set_pvt_point(TmPvtMode mode, const TmPvtPoint &point, const std::string &id = "PvtPt");

	bool set_pvt_traj(const TmPvtTraj &pvts, const std::string &id = "PvtTraj");

	bool run_pvt_traj(const TmPvtTraj &pvts);
	void stop_pvt_traj();
};
