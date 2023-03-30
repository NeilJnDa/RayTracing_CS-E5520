#pragma once
#include <base/Math.hpp>
#include "QMC.hpp"
#include <map>
namespace FW {
	Vec2f QMC::GetVec2fSampleByHalton(int n) {

		Vec2f res(0);
		res.x = vanDerCorput(n, 2);
		res.y = vanDerCorput(n, 3);

		return res;

	}
	float QMC::vanDerCorput(int n, const int& base = 2)
	{
		float rand = 0, denom = 1, invBase = 1.f / base;
		while (n) {
			denom *= base;  //2, 4, 8, 16, etc, 2^1, 2^2, 2^3, 2^4 etc. 
			rand += (n % base) / denom;
			n *= invBase;  //divide by 2 
		}
		return rand;
	}


}