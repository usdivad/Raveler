#pragma once

#include <string>

class RaveWwiseFX;

namespace RAVEWwise
{
	void UpdateEngineJob(RaveWwiseFX* processor, const std::string& modelFile, size_t fadeOutWaitTimeMs = 0);
}
