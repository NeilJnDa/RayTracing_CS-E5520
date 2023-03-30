#pragma once
#include <base/Math.hpp>
#include <map>
namespace FW {
	class QMC {
	public:
		QMC() {};
		~QMC() {};
		Vec2f GetVec2fSampleByHalton(int n);
	private:
		float vanDerCorput(int n, const int& base);
	};
} // namespace FW