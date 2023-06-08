#pragma once

#include <string>

class RaveWwiseFX; // forward declaration

namespace RAVEWwise
{
	void UpdateEngineJob(RaveWwiseFX* processor, const std::string& modelFile);
}


// TODO: Replace juce::ThreadPoolJob with custom implementation
//class UpdateEngineJob : public juce::ThreadPoolJob {
//public:
//	explicit UpdateEngineJob(RaveWwiseFX& processor, const std::string modelPath);
//	virtual ~UpdateEngineJob();
//	virtual auto runJob()->JobStatus;
//	bool waitForFadeOut(size_t waitTimeMs);
//
//private:
//	RaveWwiseFX& mProcessor;
//	const std::string mModelFile;
//	// Prevent uncontrolled usage
//	UpdateEngineJob(const UpdateEngineJob&);
//	UpdateEngineJob& operator=(const UpdateEngineJob&);
//};
//// TODO: Replace juce::ThreadPoolJob with custom implementation
//class UpdateEngineJob : public juce::ThreadPoolJob {
//public:
//	explicit UpdateEngineJob(RaveWwiseFX& processor, const std::string modelPath);
//	virtual ~UpdateEngineJob();
//	virtual auto runJob()->JobStatus;
//	bool waitForFadeOut(size_t waitTimeMs);
//
//private:
//	RaveWwiseFX& mProcessor;
//	const std::string mModelFile;
//	// Prevent uncontrolled usage
//	UpdateEngineJob(const UpdateEngineJob&);
//	UpdateEngineJob& operator=(const UpdateEngineJob&);
//};
