#include "EngineUpdater.h"

#include <chrono>
#include <thread>

#include "RaveWwiseFX.h"

namespace RaveWwise
{
	void UpdateEngineJob(RaveWwiseFX* processor, const std::string& modelFile, size_t fadeOutWaitTimeMs /* 0 */)
	{
		// Wait for fade out
		for (size_t i = 0; i < fadeOutWaitTimeMs && processor->GetIsMuted(); ++i) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		// Update engine
		processor->_rave->load_model(modelFile);
		processor->UpdateBufferSizes();
		processor->Unmute();
		processor->SetModelLoaded(true);
	}
}
