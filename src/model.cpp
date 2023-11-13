//-----------------Model and control of Whole body with four legs_E2------------------------//
//
//   {leg_3}<motor 6, 7, 8>                         {leg_2}<motor 3, 4, 5>
//
//			    []---o								o----[]  
//				 	  \			  0.126		       /
//				 	   \ 	    kBodyWidth        /
//				 	    o---o________________o---o    
//				 			|                |             Z *----------------* Y
//				 			|                |                                |
//				 			|                |                                |
//				 			|                |                                |
//	<@---Top View---@>		|                |   0.60398                      |
//				 			|       O        |  kBodylong                     |
//				 			|                |                                |            
//				 			|                |                               \_/ X 
//				[]---o		|                |      o----[]
//				 	  \		|                |     /    
// 					   \	|                |    /
//				 		o---o________________o---o            
//		                                  
//        {leg_4}<motor 9, 10, 11>      {leg_1}<motor 0, 1, 2>
//						     
//   < The dog's forward move direction is along the positive direction of X axis >
//------------------------------------------------------------------------------------------//
#include "model.h"

void robot::QuadrupedRbtModel::createQuadrupedRbtModel() 
{
	//---part Length-->unit(m)---//
	const double L1 = 0.134;
	const double L2 = 0.306;
	const double L3 = 0.310;
	const double PI = 3.14159265358979323846;

	//---set body's size---//
	const double kBodyLong  = 0.60398; // x方向
	const double kBodyWidth = 0.126;   // z方向
	const double kBodyHigh  = 0.530;   // y方向

	//---define ee pose---//
	const double ee_pose[4][6]
	{
		{ kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1, 0.0, 0.0, 0.0},   //leg1 ->0 1  2
		{-kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1, 0.0, 0.0, 0.0},   //leg2 ->3 4  5
		{-kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1, 0.0, 0.0, 0.0},   //leg3 ->6 7  8
		{ kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1, 0.0, 0.0, 0.0},   //leg4 ->9 10 11
	};


	//---set gravity---//
	const double gravity[6]{ 0.0, -9.8, 0.0, 0.0, 0.0, 0.0 };
	this->environment().setGravity(gravity);

	//---define joint pos---//
	const double joint_pos[12][3]
	{
		{  kBodyLong / 2,   0, -kBodyWidth / 2      },
		{  kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
		{  kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
		{ -kBodyLong / 2,   0, -kBodyWidth / 2      },
		{ -kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
		{ -kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
		{ -kBodyLong / 2,   0,  kBodyWidth / 2      },
		{ -kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
		{ -kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
		{  kBodyLong / 2,   0,  kBodyWidth / 2      },
		{  kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
		{  kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
	};

	//---define iv param---//
	//iv:10x1 inertia vector [m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]//
	const double body_iv[10]{ 15.1007177439,0,0,0,0.68976308,0.612989762,0.1389151407,0,0,0 };
	//leg1
	const double lf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
	const double lf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
	const double lf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
	//leg2
	const double lr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
	const double lr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
	const double lr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
	//leg3
	const double rr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
	const double rr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
	const double rr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
	//leg4
	const double rf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
	const double rf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
	const double rf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };


	//---add part---//
	//abreviation--<left, right>-<forward, rear>//
	auto& body = this->partPool().add<aris::dynamic::Part>("BODY", body_iv);
	//leg1
	auto& lf_p1 = this->partPool().add<aris::dynamic::Part>("LF_P1", lf_p1_iv);
	auto& lf_p2 = this->partPool().add<aris::dynamic::Part>("LF_P2", lf_p2_iv);
	auto& lf_p3 = this->partPool().add<aris::dynamic::Part>("LF_P3", lf_p3_iv);
	//leg2
	auto& lr_p1 = this->partPool().add<aris::dynamic::Part>("LR_P1", lr_p1_iv);
	auto& lr_p2 = this->partPool().add<aris::dynamic::Part>("LR_P2", lr_p2_iv);
	auto& lr_p3 = this->partPool().add<aris::dynamic::Part>("LR_P3", lr_p3_iv);
	//leg3
	auto& rr_p1 = this->partPool().add<aris::dynamic::Part>("RR_P1", rr_p1_iv);
	auto& rr_p2 = this->partPool().add<aris::dynamic::Part>("RR_P2", rr_p2_iv);
	auto& rr_p3 = this->partPool().add<aris::dynamic::Part>("RR_P3", rr_p3_iv);
	//leg4
	auto& rf_p1 = this->partPool().add<aris::dynamic::Part>("RF_P1", rf_p1_iv);
	auto& rf_p2 = this->partPool().add<aris::dynamic::Part>("RF_P2", rf_p2_iv);
	auto& rf_p3 = this->partPool().add<aris::dynamic::Part>("RF_P3", rf_p3_iv);

	//---add joints---//
	//leg1
	auto& lf_j1 = this->addRevoluteJoint(lf_p1, body, joint_pos[0], std::array<double, 3>{1, 0, 0}.data());
	auto& lf_j2 = this->addRevoluteJoint(lf_p2, lf_p1, joint_pos[1], std::array<double, 3>{0, 0, 1}.data());
	auto& lf_j3 = this->addRevoluteJoint(lf_p3, lf_p2, joint_pos[2], std::array<double, 3>{0, 0, 1}.data());
	//leg2
	auto& lr_j1 = this->addRevoluteJoint(lr_p1, body, joint_pos[3], std::array<double, 3>{1, 0, 0}.data());
	auto& lr_j2 = this->addRevoluteJoint(lr_p2, lr_p1, joint_pos[4], std::array<double, 3>{0, 0, 1}.data());
	auto& lr_j3 = this->addRevoluteJoint(lr_p3, lr_p2, joint_pos[5], std::array<double, 3>{0, 0, 1}.data());
	//leg3
	auto& rr_j1 = this->addRevoluteJoint(rr_p1, body, joint_pos[6], std::array<double, 3>{1, 0, 0}.data());
	auto& rr_j2 = this->addRevoluteJoint(rr_p2, rr_p1, joint_pos[7], std::array<double, 3>{0, 0, 1}.data());
	auto& rr_j3 = this->addRevoluteJoint(rr_p3, rr_p2, joint_pos[8], std::array<double, 3>{0, 0, 1}.data());
	//leg4
	auto& rf_j1 = this->addRevoluteJoint(rf_p1, body, joint_pos[9], std::array<double, 3>{1, 0, 0}.data());
	auto& rf_j2 = this->addRevoluteJoint(rf_p2, rf_p1, joint_pos[10], std::array<double, 3>{0, 0, 1}.data());
	auto& rf_j3 = this->addRevoluteJoint(rf_p3, rf_p2, joint_pos[11], std::array<double, 3>{0, 0, 1}.data());


	// add motion //
	//leg1
	auto& lf_m1 = this->addMotion(lf_j1);
	auto& lf_m2 = this->addMotion(lf_j2);
	auto& lf_m3 = this->addMotion(lf_j3);
	//leg2
	auto& lr_m1 = this->addMotion(lr_j1);
	auto& lr_m2 = this->addMotion(lr_j2);
	auto& lr_m3 = this->addMotion(lr_j3);
	//leg3
	auto& rr_m1 = this->addMotion(rr_j1);
	auto& rr_m2 = this->addMotion(rr_j2);
	auto& rr_m3 = this->addMotion(rr_j3);
	//leg4
	auto& rf_m1 = this->addMotion(rf_j1);
	auto& rf_m2 = this->addMotion(rf_j2);
	auto& rf_m3 = this->addMotion(rf_j3);


	// add end-effector //
	auto body_ee_maki = body.addMarker("body_ee_mak_i");
	auto body_ee_makj = this->ground().addMarker("body_ee_mak_j");

	auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
	body_ee.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);
	
	auto& lf_ee = this->addPointMotion(lf_p3, this->ground(), ee_pose[0]);
	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
	auto& lr_ee = this->addPointMotion(lr_p3, this->ground(), ee_pose[1]);
	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
	auto& rr_ee = this->addPointMotion(rr_p3, this->ground(), ee_pose[2]);
	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
	auto& rf_ee = this->addPointMotion(rf_p3, this->ground(), ee_pose[3]);
	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());


	// add force //
	auto& f1 = this->forcePool().add<aris::dynamic::GeneralForce>("f1", lf_ee.makI(), lf_ee.makJ());
	auto& f2 = this->forcePool().add<aris::dynamic::GeneralForce>("f2", lr_ee.makI(), lr_ee.makJ());
	auto& f3 = this->forcePool().add<aris::dynamic::GeneralForce>("f3", rr_ee.makI(), rr_ee.makJ());
	auto& f4 = this->forcePool().add<aris::dynamic::GeneralForce>("f4", rf_ee.makI(), rf_ee.makJ());

	// add solver //
	auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

	auto& fix_body_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
	auto& stand_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
	auto& other = this->solverPool().add<aris::dynamic::UniversalSolver>();

	// add simulator and simulation results //
	auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
	auto& result = this->simResultPool().add<aris::dynamic::SimResult>();

	// initialization  the model and output to verify it //
	this->init();
	std::cout << "Successful modeling Haha !" << std::endl;
}

robot::QuadrupedRbtModel::QuadrupedRbtModel()
{
	this-> createQuadrupedRbtModel();
}

robot::QuadrupedRbtModel::~QuadrupedRbtModel() = default;


auto robot::createQuadrupedRbtModelPtr() -> std::unique_ptr<aris::dynamic::Model>
{
	return std::unique_ptr<aris::dynamic::Model>(new QuadrupedRbtModel);
}


// //-----------------Model and control of Whole body with four legs------------------------//
// //
// //   {leg_3}<motor 6, 7, 8>                         {leg_2}<motor 3, 4, 5>
// //
// //			    []---o								o----[]  
// //				 	  \			  0.126		       /
// //				 	   \ 	    kBodyWidth        /
// //				 	    o---o________________o---o    
// //				 			|                |             Z *----------------* Y
// //				 			|                |                                |
// //				 			|                |                                |
// //				 			|                |                                |
// //	<@---Top View---@>		|                |   0.60398                      |
// //				 			|       O        |  kBodylong                     |
// //				 			|                |                                |            
// //				 			|                |                               \_/ X 
// //				[]---o		|                |      o----[]
// //				 	  \		|                |     /    
// // 					   \	|                |    /
// //				 		o---o________________o---o            
// //		                                  
// //        {leg_4}<motor 9, 10, 11>      {leg_1}<motor 0, 1, 2>
// //						     
// //   < The dog's forward move direction is along the positive direction of X axis >
// //------------------------------------------------------------------------------------------//
// #include "model.h"

// void robot::QuadrupedRbtModel::createQuadrupedRbtModel() 
// {
// 	//part Length-->unit(m)//
// 	const double L1 = 0.134;
// 	const double L2 = 0.306;
// 	const double L3 = 0.310;
// 	const double PI = 3.14159265358979323846;

// 	//set body's size information//
// 	const double kBodyLong  = 0.60398; //m  x方向
// 	const double kBodyWidth = 0.126;   //m  z方向
// 	const double kBodyHigh  = 0.530;   //m  y方向

// 	//initial position of Foot 
// 	static double foot_position_start_point[12] = 
// 	{
// 		 kBodyLong / 2 , -kBodyHigh, -(kBodyWidth / 2) - L1,   // leg1 -> 0 1  2
// 		-kBodyLong / 2 , -kBodyHigh, -(kBodyWidth / 2) - L1,   // leg2 -> 3 4  5
// 		-kBodyLong / 2 , -kBodyHigh,  (kBodyWidth / 2) + L1,   // leg3 -> 6 7  8
// 		 kBodyLong / 2 , -kBodyHigh,  (kBodyWidth / 2) + L1    // leg4 -> 9 10 11
// 	};

// 	static double body_position_start_point[16] = 
// 	{ 
// 		1,0,0,0,
// 		0,1,0,0,
// 		0,0,1,0,
// 		0,0,0,1 
// 	};


// 	// set gravity //
// 	const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
// 	this->environment().setGravity(gravity);

// 	//define joint pos //
// 	const double leg_pe[12][3]
// 	{
// 		{  kBodyLong / 2,   0, -kBodyWidth / 2      },
// 		{  kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
// 		{  kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
// 		{ -kBodyLong / 2,   0, -kBodyWidth / 2      },
// 		{ -kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
// 		{ -kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
// 		{ -kBodyLong / 2,   0,  kBodyWidth / 2      },
// 		{ -kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
// 		{ -kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
// 		{  kBodyLong / 2,   0,  kBodyWidth / 2      },
// 		{  kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
// 		{  kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
// 	};

// 	//define ee pos //
// 	const double ee_pos[4][6]
// 	{
// 		{ kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg1 ->012
// 		{-kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg2 ->345
// 		{-kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg3 ->678
// 		{ kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg4 ->91011
// 	};

// 	//iv:10x1 inertia vector [m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
// 	//define iv param //
// 	const double body_iv[10]{ 15.1007177439,0,0,0,0.68976308,0.612989762,0.1389151407,0,0,0 };
// 	//leg1
// 	const double lf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
// 	const double lf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
// 	const double lf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
// 	//leg2
// 	const double lr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
// 	const double lr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
// 	const double lr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
// 	//leg3
// 	const double rr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
// 	const double rr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
// 	const double rr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
// 	//leg4
// 	const double rf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
// 	const double rf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
// 	const double rf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };


// 	// add part // abreviation--<left, right>-<forward, rear>	
// 	auto& body = this->partPool().add<aris::dynamic::Part>("BODY", body_iv);
// 	//leg1
// 	auto& lf_p1 = this->partPool().add<aris::dynamic::Part>("LF_P1", lf_p1_iv);
// 	auto& lf_p2 = this->partPool().add<aris::dynamic::Part>("LF_P2", lf_p2_iv);
// 	auto& lf_p3 = this->partPool().add<aris::dynamic::Part>("LF_P3", lf_p3_iv);
// 	//leg2
// 	auto& lr_p1 = this->partPool().add<aris::dynamic::Part>("LR_P1", lr_p1_iv);
// 	auto& lr_p2 = this->partPool().add<aris::dynamic::Part>("LR_P2", lr_p2_iv);
// 	auto& lr_p3 = this->partPool().add<aris::dynamic::Part>("LR_P3", lr_p3_iv);
// 	//leg3
// 	auto& rr_p1 = this->partPool().add<aris::dynamic::Part>("RR_P1", rr_p1_iv);
// 	auto& rr_p2 = this->partPool().add<aris::dynamic::Part>("RR_P2", rr_p2_iv);
// 	auto& rr_p3 = this->partPool().add<aris::dynamic::Part>("RR_P3", rr_p3_iv);
// 	//leg4
// 	auto& rf_p1 = this->partPool().add<aris::dynamic::Part>("RF_P1", rf_p1_iv);
// 	auto& rf_p2 = this->partPool().add<aris::dynamic::Part>("RF_P2", rf_p2_iv);
// 	auto& rf_p3 = this->partPool().add<aris::dynamic::Part>("RF_P3", rf_p3_iv);

// 	// add joints //
// 	//leg1
// 	auto& lf_r1 = this->addRevoluteJoint(lf_p1, body, leg_pe[0], std::array<double, 3>{1, 0, 0}.data());
// 	auto& lf_r2 = this->addRevoluteJoint(lf_p2, lf_p1, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());
// 	auto& lf_r3 = this->addRevoluteJoint(lf_p3, lf_p2, leg_pe[2], std::array<double, 3>{0, 0, 1}.data());
// 	//leg2
// 	auto& lr_r1 = this->addRevoluteJoint(lr_p1, body, leg_pe[3], std::array<double, 3>{1, 0, 0}.data());
// 	auto& lr_r2 = this->addRevoluteJoint(lr_p2, lr_p1, leg_pe[4], std::array<double, 3>{0, 0, 1}.data());
// 	auto& lr_r3 = this->addRevoluteJoint(lr_p3, lr_p2, leg_pe[5], std::array<double, 3>{0, 0, 1}.data());
// 	//leg3
// 	auto& rr_r1 = this->addRevoluteJoint(rr_p1, body, leg_pe[6], std::array<double, 3>{1, 0, 0}.data());
// 	auto& rr_r2 = this->addRevoluteJoint(rr_p2, rr_p1, leg_pe[7], std::array<double, 3>{0, 0, 1}.data());
// 	auto& rr_r3 = this->addRevoluteJoint(rr_p3, rr_p2, leg_pe[8], std::array<double, 3>{0, 0, 1}.data());
// 	//leg4
// 	auto& rf_r1 = this->addRevoluteJoint(rf_p1, body, leg_pe[9], std::array<double, 3>{1, 0, 0}.data());
// 	auto& rf_r2 = this->addRevoluteJoint(rf_p2, rf_p1, leg_pe[10], std::array<double, 3>{0, 0, 1}.data());
// 	auto& rf_r3 = this->addRevoluteJoint(rf_p3, rf_p2, leg_pe[11], std::array<double, 3>{0, 0, 1}.data());


// 	// add motion //
// 	//leg1
// 	auto& lf_m1 = this->addMotion(lf_r1);
// 	auto& lf_m2 = this->addMotion(lf_r2);
// 	auto& lf_m3 = this->addMotion(lf_r3);
// 	//leg2
// 	auto& lr_m1 = this->addMotion(lr_r1);
// 	auto& lr_m2 = this->addMotion(lr_r2);
// 	auto& lr_m3 = this->addMotion(lr_r3);
// 	//leg3
// 	auto& rr_m1 = this->addMotion(rr_r1);
// 	auto& rr_m2 = this->addMotion(rr_r2);
// 	auto& rr_m3 = this->addMotion(rr_r3);
// 	//leg4
// 	auto& rf_m1 = this->addMotion(rf_r1);
// 	auto& rf_m2 = this->addMotion(rf_r2);
// 	auto& rf_m3 = this->addMotion(rf_r3);


// 	// add end-effector //
// 	auto body_ee_maki = body.addMarker("body_ee_mak_i");
// 	auto body_ee_makj = this->ground().addMarker("body_ee_mak_j");

// 	auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
// 	body_ee.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);
	
// 	auto& lf_ee = this->addPointMotion(lf_p3, this->ground(), ee_pos[0]);
// 	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
// 	auto& lr_ee = this->addPointMotion(lr_p3, this->ground(), ee_pos[1]);
// 	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
// 	auto& rr_ee = this->addPointMotion(rr_p3, this->ground(), ee_pos[2]);
// 	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
// 	auto& rf_ee = this->addPointMotion(rf_p3, this->ground(), ee_pos[3]);
// 	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());


// 	// add force //
// 	auto& f1 = this->forcePool().add<aris::dynamic::GeneralForce>("f1", lf_ee.makI(), lf_ee.makJ());
// 	auto& f2 = this->forcePool().add<aris::dynamic::GeneralForce>("f2", lr_ee.makI(), lr_ee.makJ());
// 	auto& f3 = this->forcePool().add<aris::dynamic::GeneralForce>("f3", rr_ee.makI(), rr_ee.makJ());
// 	auto& f4 = this->forcePool().add<aris::dynamic::GeneralForce>("f4", rf_ee.makI(), rf_ee.makJ());

// 	// add solver //
// 	auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
// 	auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
// 	auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
// 	auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

// 	auto& fix_body_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
// 	auto& stand_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
// 	auto& other = this->solverPool().add<aris::dynamic::UniversalSolver>();

// 	// add simulator and simulation results //
// 	auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
// 	auto& result = this->simResultPool().add<aris::dynamic::SimResult>();

// 	// initialization  the model and output to verify it //
// 	this->init();
// 	std::cout << "Successful modeling Haha !" << std::endl;

// //===============test function usage of aris/dynamics/math_matrix================//
//     const int rows = 2;
//     const int cols = 2;
//     double matrixData[rows * cols];

//     // Access and modify matrix elements
//     matrixData[at(0, 0, RowMajor(2))] = 1.0;  // Element at row 0, column 0
//     matrixData[at(1, 0, RowMajor(2))] = 2.0;  // Element at row 1, column 2
//     matrixData[at(0, 1, RowMajor(2))] = 3.0;  // Element at row 2, column 3
//     matrixData[at(1, 1, RowMajor(2))] = 4.0;  // Element at row 2, column 3


//     // Display the matrix
//     dsp(rows, cols, matrixData, RowMajor(cols));

// 	double matrix[2 * 2];
// 	matrix[at(0, 0)] = 1.0;  // Element at row 0, column 0
//     matrix[at(1, 0)] = 1.0;  // Element at row 1, column 2
//     matrix[at(0, 1)] = 1.0;  // Element at row 2, column 3
//     matrix[at(1, 1)] = 4.0;  // Element at row 2, column 3

// 	dsp(2, 2, matrix, RowMajor(2));

// 	s_mc(2,2,matrix,matrixData);
// 	dsp(rows, cols, matrixData, RowMajor(2));

// 	double testMatrix[3 * 4]{};
// 	dsp(3,4,testMatrix,RowMajor(4));



// //======================================test end===============//
// }

// robot::QuadrupedRbtModel::QuadrupedRbtModel()
// {
// 	this-> createQuadrupedRbtModel();
// }

// robot::QuadrupedRbtModel::~QuadrupedRbtModel() = default;


// auto robot::createQuadrupedRbtModelPtr() -> std::unique_ptr<aris::dynamic::Model>
// {
// 	return std::unique_ptr<aris::dynamic::Model>(new QuadrupedRbtModel);
// }


// 单腿的建模与控制  <SingleLeg Model Edition 2  --对应于实际的本条单腿 >
#include "model.h"
//-------------------------------------------------------------------------------------------------------//
//两个杆件，三个电机，髋关节两个，膝关节一个
//上端身体固定点，末端足尖作为轨迹规划点      ^ Y 
//模型示意：                                \
//                                          \
//        O--                                \
//        |\                                  \
//        |                                M0  o--------------->  X     
//       / \\                                  | --->L1=a            
// [See From Ls to Rs]                     M1  o
//                                             | \ --->L2=b       
//                                             |  \
//                                             |   o M2
//                                             |    \ 
//                                             |     \ --->L3=c
//                                            \_/     *
//                                             Z      
//-------------------------------------------------------------------------------------------------------//

void robot::singleLegModel::createSingleLegModel() 
{
	double a = 0.1315;
	double b = 0.306;
	double c = 0.330;
	const double PI = 3.141592653589793;



	// 设置重力,重力在y轴
	const double gravity[6]{ 0.0, -9.81, 0.0, 0.0, 0.0, 0.0 };
	this->environment().setGravity(gravity);

	// 定义关节的位置，以及轴线，有3个转动副，具体轴线和编号如模型所示
	const double joint1_position[3]{ 0 , 0 , 0 };
	const double joint1_axis[3]{ 0 , 0 , 1 };
	const double joint2_position[3]{ 0 , 0 , a };
	const double joint2_axis[3]{ 1 , 0 , 0 };
	const double joint3_position[3]{ 0 , -b , a };
	const double joint3_axis[3]{ 1 , 0 , 0 };

	// 定义末端的位置与321欧拉角，以及10维的惯量向量
	const double foot_position_and_euler321[6]{ 0 , -b - c, a, 0 , 0 , 0 };
	const double foot_intertia_vecter[10]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	const double link1_pos_euler[6]{ 0, 0, a / 2, 0, 0, 0 };
	const double link1_intertia_vector[10]{ 1 , 0 , 0 , 0 ,0, 0, 0, 0, 0, 0 };
	const double link2_pos_euler[6]{ 0, -b / 2, a, 0, 0, 0 };
	const double link2_intertia_vector[10]{ 1 , 0 , 0 , 0 ,0, 0, 0, 0, 0, 0 };
	const double link3_pos_euler[6]{ 0, -b - c / 2 , a, 0, 0, 0 };
	const double link3_intertia_vecter[10]{ 1 , 0 , 0 , 0 ,0, 0, 0, 0, 0, 0 };


	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto& link1 = this->addPartByPe(link1_pos_euler, "321", link1_intertia_vector);
	auto& link2 = this->addPartByPe(link2_pos_euler, "321", link2_intertia_vector);
	auto& link3 = this->addPartByPe(link3_pos_euler, "321", link3_intertia_vecter);


	// 添加末端，第一个参数表明末端位于link3上，第二个参数表明末端的位姿是相对于地面ground的
	//（这里的地面就是body，因为body是固定的），后两个参数定义了末端的起始位姿
	//auto& foot = model.addGeneralMotionByPe(link3, model.ground(), foot_position_and_euler321, "321");

	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto& joint1 = this->addRevoluteJoint(link1, this->ground(), joint1_position, joint1_axis);
	auto& joint2 = this->addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto& joint3 = this->addRevoluteJoint(link3, link2, joint3_position, joint3_axis);


	// 添加驱动 
	auto& motion1 = this->addMotion(joint1);
	auto& motion2 = this->addMotion(joint2);
	auto& motion3 = this->addMotion(joint3);

	auto& foot = this->addPointMotion(link3, this->ground(), foot_position_and_euler321);
	this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());

	
	//-------------------------------------------- 添加求解器 --------------------------------------------//
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

	this->init();
	std::cout << "Successful modeling haha ! \n" << std::endl;	
}

robot::singleLegModel::singleLegModel() 
{
	this->createSingleLegModel();
}
robot::singleLegModel::~singleLegModel() = default;
auto robot::createSingleLegModelPtr() -> std::unique_ptr<aris::dynamic::Model> 
{
	return std::unique_ptr<aris::dynamic::Model> (new singleLegModel);
}