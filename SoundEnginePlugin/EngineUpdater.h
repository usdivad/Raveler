#pragma once

#include <string>

class RaveWwiseFX;

namespace RaveWwise
{
	void UpdateEngineJob(RaveWwiseFX* processor, const std::string& modelFile, size_t fadeOutWaitTimeMs = 0);
}
