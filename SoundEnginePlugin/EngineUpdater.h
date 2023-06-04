#pragma once

#include <JuceHeader.h>

class RaveWwiseFX; // forward declaration

class UpdateEngineJob : public juce::ThreadPoolJob {
public:
	explicit UpdateEngineJob(RaveWwiseFX& processor, const std::string modelPath);
	virtual ~UpdateEngineJob();
	virtual auto runJob()->JobStatus;
	bool waitForFadeOut(size_t waitTimeMs);

private:
	RaveWwiseFX& mProcessor;
	const std::string mModelFile;
	// Prevent uncontrolled usage
	UpdateEngineJob(const UpdateEngineJob&);
	UpdateEngineJob& operator=(const UpdateEngineJob&);
};
