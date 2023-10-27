#ifndef MODEL_H
#define MODEL_H

#include <aris.hpp>
#include <iostream>

namespace robot
{

	using namespace aris::dynamic;

	class singleLegModel : public aris::dynamic::Model
	{
	public:
		void createSingleLegModel();

		singleLegModel();
		~singleLegModel();

	private:
		// aris::dynamic::Model* model ;
	};

	auto createSingleLegModelPtr() -> std::unique_ptr<aris::dynamic::Model>;

}
#endif 