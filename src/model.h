#ifndef MODEL_H
#define MODEL_H

#include <aris.hpp>
#include <iostream>

namespace robot
{
	using namespace aris::dynamic;

	////---Model for singleLeg---//
	//class singleLegModel : public aris::dynamic::Model
	//{
	//public:
	//	void createSingleLegModel();
	//	singleLegModel();
	//	~singleLegModel();
	//};

	//auto createSingleLegModelPtr() -> std::unique_ptr<aris::dynamic::Model>;

	//--Model for Quadruped Robot---//
	class QuadrupedRbtModel : public aris::dynamic::Model
	{
	public:
		void createQuadrupedRbtModel();
		QuadrupedRbtModel();
		~QuadrupedRbtModel();
	};
	auto createQuadrupedRbtModelPtr() -> std::unique_ptr<aris::dynamic::Model>;
}
#endif 