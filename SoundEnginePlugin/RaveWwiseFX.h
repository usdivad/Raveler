/*******************************************************************************
The content of this file includes portions of the AUDIOKINETIC Wwise Technology
released in source code form as part of the SDK installer package.

Commercial License Usage

Licensees holding valid commercial licenses to the AUDIOKINETIC Wwise Technology
may use this file in accordance with the end user license agreement provided
with the software or, alternatively, in accordance with the terms contained in a
written agreement between you and Audiokinetic Inc.

Apache License Usage

Alternatively, this file may be used under the Apache License, Version 2.0 (the
"Apache License"); you may not use this file except in compliance with the
Apache License. You may obtain a copy of the Apache License at
http://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed
under the Apache License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
OR CONDITIONS OF ANY KIND, either express or implied. See the Apache License for
the specific language governing permissions and limitations under the License.

  Copyright (c) 2021 Audiokinetic Inc.
*******************************************************************************/

#ifndef RaveWwiseFX_H
#define RaveWwiseFX_H

#include "RaveWwiseFXParams.h"

#include "CircularBuffer.h"
#include "EngineUpdater.h"
#include "Rave.h"

#include <algorithm>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <torch/script.h>
#include <torch/torch.h>
#include <JuceHeader.h>
#include <BS_thread_pool.hpp>

#define EPSILON 0.0000001
#define DEBUG 0

const size_t AVAILABLE_DIMS = 8;
const std::vector<std::string> channel_modes = { "L", "R", "L + R" };

namespace rave_parameters {
	const std::string model_selection{ "model_selection" };
	const std::string input_gain{ "input_gain" };
	const std::string channel_mode{ "channel_mode" };
	const std::string input_thresh{ "input_threshold" };
	const std::string input_ratio{ "input_ratio" };
	const std::string latent_jitter{ "latent_jitter" };
	const std::string output_width{ "output_width" };
	const std::string output_gain{ "output_gain" };
	const std::string output_limit{ "output_limit" };
	const std::string output_drywet{ "output_drywet" };
	const std::string latent_scale{ "latent_scale" };
	const std::string latent_bias{ "latent_bias" };
	const std::string latency_mode{ "latency_mode" };
	const std::string use_prior{ "use_prior" };
	const std::string prior_temperature{ "prior_temperature" };
} // namespace rave_parameters

// TODO: Replace juce::NormalisableRange with custom implementation
namespace rave_ranges {
	const juce::NormalisableRange<float> gainRange(-70.f, 12.f);
	const juce::NormalisableRange<float> latentScaleRange(0.0f, 5.0f);
	const juce::NormalisableRange<float> latentBiasRange(-3.0f, 3.0f);
} // namespace rave_ranges


//----------------------------------------------------------------------------------------------------------------------
// RaveWwiseFX
//----------------------------------------------------------------------------------------------------------------------

/// Wwise plugin that runs RAVE.

/// See https://www.audiokinetic.com/library/edge/?source=SDK&id=soundengine__plugins__effects.html
/// for the documentation about effect plug-ins

// TODO: Capitalize ported function and variable names

class RaveWwiseFX
    : public AK::IAkOutOfPlaceEffectPlugin
{
public:
    //------------------------------------------------------------------------------------------------------------------

    RaveWwiseFX();
    ~RaveWwiseFX();

	//------------------------------------------------------------------------------------------------------------------
    // IAkEffectPlugin::

    /// Plug-in initialization.
    /// Prepares the plug-in for data processing, allocates memory and sets up the initial conditions.
    AKRESULT Init(AK::IAkPluginMemAlloc* in_pAllocator, AK::IAkEffectPluginContext* in_pContext, AK::IAkPluginParam* in_pParams, AkAudioFormat& in_rFormat) override;

    /// Release the resources upon termination of the plug-in.
    AKRESULT Term(AK::IAkPluginMemAlloc* in_pAllocator) override;

    /// The reset action should perform any actions required to reinitialize the
    /// state of the plug-in to its original state (e.g. after Init() or on effect bypass).
    AKRESULT Reset() override;

    /// Plug-in information query mechanism used when the sound engine requires
    /// information about the plug-in to determine its behavior.
    AKRESULT GetPluginInfo(AkPluginInfo& out_rPluginInfo) override;

    /// Effect plug-in DSP execution.
    void Execute(AkAudioBuffer* in_pBuffer, AkUInt32 in_ulnOffset, AkAudioBuffer* out_pBuffer) override;

    /// Skips execution of some frames, when the voice is virtual playing from elapsed time.
    /// This can be used to simulate processing that would have taken place (e.g. update internal state).
    /// Return AK_DataReady or AK_NoMoreData, depending if there would be audio output or not at that point.
    AKRESULT TimeSkip(AkUInt32 &io_uFrames) override;

	//------------------------------------------------------------------------------------------------------------------
    // RaveAP functions

	void modelPerform();
	void detectAvailableModels();

	void mute();
	void unmute();
    bool getIsMuted() const { return _isMuted; }
	
    void updateBufferSizes();

	void updateEngine(const std::string& modelFile);

	double getSampleRate() const { return _sampleRate; }


    //------------------------------------------------------------------------------------------------------------------
    // RaveAP variables

    std::unique_ptr<RAVE> _rave;
    float _inputAmplitudeL;
    float _inputAmplitudeR;
    float _outputAmplitudeL;
    float _outputAmplitudeR;
    bool _plays = false;

private:
    //------------------------------------------------------------------------------------------------------------------

    RaveWwiseFXParams* m_pParams;
    AK::IAkPluginMemAlloc* m_pAllocator;
    AK::IAkEffectPluginContext* m_pContext;

    //------------------------------------------------------------------------------------------------------------------
	// RaveAP variables

    std::mutex _engineUpdateMutex;
    std::unique_ptr<BS::thread_pool> _engineThreadPool;
    std::string _loadedModelName;

    /*
     *Allocate some memory to use as the circular_buffer storage
     *for each of the circular_buffer types to be created
     */
    double _sampleRate = 0;
    std::unique_ptr<circular_buffer<float, float>[]> _inBuffer;
    std::unique_ptr<circular_buffer<float, float>[]> _outBuffer;
    std::vector<std::unique_ptr<float[]>> _inModel, _outModel;
    std::unique_ptr<std::thread> _computeThread;

    bool _editorReady;

    float* _inFifoBuffer{ nullptr };
    float* _outFifoBuffer{ nullptr };

    std::atomic<float>* _inputGainValue;
    std::atomic<float>* _thresholdValue;
    std::atomic<float>* _ratioValue;
    std::atomic<float>* _latentJitterValue;
    std::atomic<float>* _widthValue;
    std::atomic<float>* _outputGainValue;
    std::atomic<float>* _dryWetValue;
    std::atomic<float>* _limitValue;
    std::atomic<float>* _channelMode;
    // latency mode contains the power of 2 of the current refresh rate.
    std::atomic<float>* _latencyMode;
    std::atomic<float>* _usePrior;
    std::atomic<float>* _priorTemperature;

    std::array<std::atomic<float>*, AVAILABLE_DIMS>* _latentScale;
    std::array<std::atomic<float>*, AVAILABLE_DIMS>* _latentBias;
    std::atomic<bool> _isMuted{ true };

    enum class muting : int { ignore = 0, mute, unmute };

    std::atomic<muting> _fadeScheduler{ muting::mute };
    juce::LinearSmoothedValue<float> _smoothedFadeInOut;
    juce::LinearSmoothedValue<float> _smoothedWetGain;
    juce::LinearSmoothedValue<float> _smoothedDryGain;

    // TODO
    // DSP effect
    //juce::dsp::Compressor<float> _compressorEffect;
    //juce::dsp::Gain<float> _inputGainEffect;
    //juce::dsp::Gain<float> _outputGainEffect;
    //juce::dsp::Limiter<float> _limiterEffect;
    //juce::dsp::DryWetMixer<float> _dryWetMixerEffect;
};

#endif // RaveWwiseFX_H
