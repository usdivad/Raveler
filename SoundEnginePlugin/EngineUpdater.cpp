#include "EngineUpdater.h"

#include "RaveWwiseFX.h"

UpdateEngineJob::UpdateEngineJob(RaveWwiseFX& processor, const std::string modelFile)
	: ThreadPoolJob("UpdateEngineJob"), mProcessor(processor),
	mModelFile(modelFile) {}

UpdateEngineJob::~UpdateEngineJob() {}

bool UpdateEngineJob::waitForFadeOut(size_t waitTimeMs) {
	// TODO
	//for (size_t i = 0; i < waitTimeMs && mProcessor.getIsMuted(); ++i) {
	//  juce::Thread::sleep(1);
	//}
	//return (mProcessor.getIsMuted());

	return false;
}

auto UpdateEngineJob::runJob() -> JobStatus {
	if (shouldExit()) {
		return JobStatus::jobNeedsRunningAgain;
	}

	// TODO
	//mProcessor.mute();

	while (!waitForFadeOut(1)) {
		if (shouldExit()) {
			return JobStatus::jobNeedsRunningAgain;
		}
	}

	// TODO
	//mProcessor._rave->load_model(mModelFile);
	//mProcessor.updateBufferSizes();
	//mProcessor.unmute();

	DBG("Job finished");

	return JobStatus::jobHasFinished;
}
