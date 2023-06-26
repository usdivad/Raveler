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
#include <cassert>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <torch/script.h>
#include <torch/torch.h>
#include <BS_thread_pool.hpp>

//----------------------------------------------------------------------------------------------------------------------

#define EPSILON 0.0000001
#define DEBUG 0

const size_t AVAILABLE_DIMS = 8;
const std::vector<std::string> channel_modes = { "L", "R", "L + R" };

//----------------------------------------------------------------------------------------------------------------------
// RaveWwiseFX
//----------------------------------------------------------------------------------------------------------------------

/// Wwise plugin that runs RAVE.

/// See https://www.audiokinetic.com/library/edge/?source=SDK&id=soundengine__plugins__effects.html
/// for the documentation about effect plug-ins

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
    void Execute(AkAudioBuffer* in_pBuffer, AkUInt32 in_ulOffset, AkAudioBuffer* out_pBuffer) override;

    /// Skips execution of some frames, when the voice is virtual playing from elapsed time.
    /// This can be used to simulate processing that would have taken place (e.g. update internal state).
    /// Return AK_DataReady or AK_NoMoreData, depending if there would be audio output or not at that point.
    AKRESULT TimeSkip(AkUInt32 &io_uFrames) override;

	//------------------------------------------------------------------------------------------------------------------
    // RaveAP ported functions

    /// Perform RAVE model inference for an input buffer
	void ModelPerform();

    /// Detect the currently available models
	void DetectAvailableModels();

    /// Mute the effect plugin
	void Mute();

    /// Unmute the effect plugin
	void Unmute();

    /// Whether or not the plugin is currently muted
    bool GetIsMuted() const { return _isMuted.load(); }
	
    /// Update internal buffer sizes based on latency settings
    void UpdateBufferSizes();

    /// Update the engine (i.e. our internal RAVE instance) with a new model
	void UpdateEngine(const std::string& modelFile);

	/// Get whether a model has been loaded
	bool GetModelLoaded() { return _modelLoaded; }

    /// Set whether a model has been loaded
    void SetModelLoaded(bool loaded) { _modelLoaded = loaded; }

    //------------------------------------------------------------------------------------------------------------------
    // RaveAP ported variables

    std::unique_ptr<RAVE> _rave { nullptr };

private:
    //------------------------------------------------------------------------------------------------------------------
    // Effect plugin variables

    RaveWwiseFXParams* _fxParams { nullptr };
    AK::IAkPluginMemAlloc* _fxAllocator { nullptr };
    AK::IAkEffectPluginContext* _fxContext { nullptr };

    //------------------------------------------------------------------------------------------------------------------
	// RaveAP ported variables

    std::mutex _engineUpdateMutex { };
    std::unique_ptr<BS::thread_pool> _engineThreadPool { nullptr };
    std::string _loadedModelName { };

    std::unique_ptr<circular_buffer<float, float>[]> _inBuffer { nullptr };
    std::unique_ptr<circular_buffer<float, float>[]> _dryBuffer { nullptr };
    std::unique_ptr<circular_buffer<float, float>[]> _outBuffer { nullptr };

    std::vector<std::unique_ptr<float[]>> _inModel{ }, _outModel{ };
    std::unique_ptr<std::thread> _computeThread { nullptr };

    // Latency mode contains the power of 2 of the current refresh rate.
    float _latencyMode { 13 }; // min = 9, max = 15, default = 13

    std::array<float, AVAILABLE_DIMS> _latentScale { 1.f };
    std::array<float, AVAILABLE_DIMS> _latentBias { 0.f };
    
    std::atomic<bool> _isMuted { true };

	//------------------------------------------------------------------------------------------------------------------
	// Custom variables

    bool _modelLoaded = false;
    int _modelLoadTimeSamples = 0;
    bool _modelPerformed = false;
    int _modelPerformTimeSamples = 0;
    int _dryLatencySamplesElapsed = 0;

};

#endif // RaveWwiseFX_H
