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



/*  <SingleLeg Model Edition 1> <原始模型代码--可以顺利进行一切操作与运算><单腿搭建后发现该模型与实际不符而更改为 SingleLeg Model Edition 2>

// 单腿的建模与控制
#include "model.h"
//-------------------------------------------------------------------------------------------------------//
//两个杆件，三个电机，髋关节两个，膝关节一个
//上端身体固定点，末端足尖作为轨迹规划点
//模型示意：
//                              |*******| ---> Body[Fixed Point]    
//        O--                   |**0****| ---> Hip Joint [-X1-HAA-]
//        |\                       || ------>>Link1(a)
//        |                        O---->Hip Joint  [-Y2-HFE-]                  
//       / \\                     //                                            Y   
// [See From Ls to Rs]           // ---->>Link2(b)                              ^
//                              //                                              |   
//                              O ------->Knee Joint [-Y3-HFE-]                 |         
//                              \\                                              |
//                               \\ ----->>Link3(c)                             | L1
//                                \\                                            o---o------> X  
//                                 ** ---->Foot tip                            /    | L2
//                                                                            /     |
//                                                                           /      o
//                                                                          /       | L3
//                                                                         Z        * 
//                                                                                  ee
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
	const double joint2_position[3]{ a , 0 , 0 };
	const double joint2_axis[3]{ 1 , 0 , 0 };
	const double joint3_position[3]{ a , -b , 0 };
	const double joint3_axis[3]{ 1 , 0 , 0 };

	// 定义末端的位置与321欧拉角，以及10维的惯量向量
	const double foot_position_and_euler321[6]{ a , -b - c, 0, 0 , 0 , 0 };
	const double foot_intertia_vecter[10]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	const double link1_pos_euler[6]{ a / 2, 0, 0, 0, 0, 0 };
	const double link1_intertia_vector[10]{ 1 , 0 , 0 , 0 ,0, 0, 0, 0, 0, 0 };
	const double link2_pos_euler[6]{ a, -b / 2, 0, 0, 0, 0 };
	const double link2_intertia_vector[10]{ 1 , 0 , 0 , 0 ,0, 0, 0, 0, 0, 0 };
	const double link3_pos_euler[6]{ a , -b - c / 2 , 0, 0, 0, 0 };
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

	//添加力
	//auto& force1 = model.forcePool().add<aris::dynamic::SingleComponentForce>("f1", motion1.makI(), motion1.makJ(), 5);
	//auto& force2 = model.forcePool().add<aris::dynamic::SingleComponentForce>("f2", motion2.makI(), motion2.makJ(), 5);
	//auto& force3 = model.forcePool().add<aris::dynamic::SingleComponentForce>("f3", motion3.makI(), motion3.makJ(), 5);

	//-------------------------------------------- 添加求解器 --------------------------------------------//
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

	this->init();
	std::cout << "Successful modeling haha ! \n" << std::endl;	
}

robot::singleLegModel::singleLegModel() //构造函数
{
	this->createSingleLegModel();
}
robot::singleLegModel::~singleLegModel() = default;//析构函数
auto robot::createSingleLegModelPtr() -> std::unique_ptr<aris::dynamic::Model> 
{
	return std::unique_ptr<aris::dynamic::Model> (new singleLegModel);
}



// 	//--------------------------------------------运动学正解--------------------------------------------//
// 	double motionPos[3]{ 0.0, 0.38, 0.45 };
// 	motion1.setMp(motionPos[0]);
// 	motion1.setMp(motionPos[0]);	
// 	motion1.setMp(motionPos[0]);

// 	if (this->solverPool()[1].kinPos())
// 		throw std::runtime_error("forward kinematic position failed");

// 	double outPos[3]{ 0.0, 0.0, 0.0};
// 	this->getOutputPos(outPos);
// 	std::cout << "output position : " << outPos[0] << " " << outPos[1] << " " << outPos[2]  << std::endl;


// //-------------------------------------------- 位置反解 --------------------------------------------//
// // 现在求位置反解，首先设置末端的位置与theta角
// 	// double ee_xyz_theta[3]{ 0.1315 ,  -0.512151562,  -0.14515615123 };
// 	//double ee_xyz_theta[3]{ 0.1315 ,  -0.3542151562,  -0.151544523 };//Output in Cmd--> Input Position : -3.01048e-17  -0.566348  1.84199
// 	double ee_xyz_theta[3]{ 0.1315 ,  -0.3542151562,  -0.151544523 };

// 	this->setOutputPos(ee_xyz_theta);

// 	if (this->solverPool()[0].kinPos())   //   inverse_kinematic_solver.kinPos()
// 		throw std::runtime_error("inverse kinematic position failed");

// 	std::cout << " Input Position : " << motion1.mp() << "  " << motion2.mp() << "  " << motion3.mp() << std::endl;

*/