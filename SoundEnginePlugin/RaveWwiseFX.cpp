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

#include "RaveWwiseFX.h"
#include "../RaveWwiseConfig.h"

#include <AK/AkWwiseSDKVersion.h>

#include <algorithm>
#include <cmath>

//----------------------------------------------------------------------------------------------------------------------

#define DEBUG_PERFORM 1

namespace RaveWwise
{
	void modelPerform_callback(RaveWwiseFX* ap) { ap->ModelPerform(); }
}

//----------------------------------------------------------------------------------------------------------------------

AK::IAkPlugin* CreateRaveWwiseFX(AK::IAkPluginMemAlloc* in_pAllocator)
{
    return AK_PLUGIN_NEW(in_pAllocator, RaveWwiseFX());
}

AK::IAkPluginParam* CreateRaveWwiseFXParams(AK::IAkPluginMemAlloc* in_pAllocator)
{
    return AK_PLUGIN_NEW(in_pAllocator, RaveWwiseFXParams());
}

AK_IMPLEMENT_PLUGIN_FACTORY(RaveWwiseFX, AkPluginTypeEffect, RaveWwiseConfig::CompanyID, RaveWwiseConfig::PluginID)

RaveWwiseFX::RaveWwiseFX() : _fxParams(nullptr)
                           , _fxAllocator(nullptr)
                           , _fxContext(nullptr)
{
    _loadedModelName = "";
    _computeThread = nullptr;

	// TODO: Perform dynamic allocations in Init() using m_pAllocator
	//      (this may require replacing std::unique_ptr's with raw pointers)

    _inBuffer = std::make_unique<circular_buffer<float, float>[]>(1);
	_dryBuffer = std::make_unique<circular_buffer<float, float>[]>(1);
    _outBuffer = std::make_unique<circular_buffer<float, float>[]>(2);
	_inModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
	_outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
	_outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
    
    _engineThreadPool = std::make_unique<BS::thread_pool>(1);
    
    _rave.reset(new RAVE());
}

RaveWwiseFX::~RaveWwiseFX()
{
    if (_computeThread)
    {
        _computeThread->join();
    }
}

AKRESULT RaveWwiseFX::Init(AK::IAkPluginMemAlloc* in_pAllocator, AK::IAkEffectPluginContext* in_pContext, AK::IAkPluginParam* in_pParams, AkAudioFormat& in_rFormat)
{
	// --------
	// Store effect plugin members

    _fxParams = (RaveWwiseFXParams*)in_pParams;
    _fxAllocator = in_pAllocator;
    _fxContext = in_pContext;

    // --------
    // Initialize circular buffers (from RaveAP::prepareToPlay())

	_inBuffer[0].initialize(BUFFER_LENGTH);
	_dryBuffer[0].initialize(DRY_BUFFER_LENGTH);
	_outBuffer[0].initialize(BUFFER_LENGTH);
	_outBuffer[1].initialize(BUFFER_LENGTH);


	// --------
	// Update non-RTPC params
    // NOTE: _loadedModelName gets updated in UpdateEngine()

	_latencyMode = _fxParams->NonRTPC.uLatencyMode;

    // --------
    // Load the model

    // NOTE: We do direct AkOSChar*-to-char* conversion via casting here, since CONVERT_OSCHAR_TO_CHAR() returns incorrect results
    //
    // TODO: - Escape backslashes for file paths ("\" --> "\\")
    //       - Move the char-conversion/string-creation to RaveWwiseFXParams?
	//       - Move entire model loading routine to plugin library registration step, to avoid re-loading unnecessarily with every effect instantiation at runtime? See https://www.audiokinetic.com/en/library/edge/?source=SDK&id=soundengine_plugins.html#fx_global_hooks
    //       - Allow relative paths + figure out platform-dependent file path handling

    SetModelLoaded(false);
    _modelLoadTimeSamples = 0;
    _modelPerformed.store(false);
    _modelPerformTimeSamples = 0;
    _dryLatencySamplesElapsed = 0;

    AkOSChar* modelFilePathOsStr = _fxParams->NonRTPC.sModelFilePath;
    char* modelFilePathCStr = reinterpret_cast<char*>(modelFilePathOsStr);
    std::string modelFilePath = std::string(modelFilePathCStr);
    UpdateEngine(modelFilePath);

    return AK_Success;
}

AKRESULT RaveWwiseFX::Term(AK::IAkPluginMemAlloc* in_pAllocator)
{
    AK_PLUGIN_DELETE(in_pAllocator, this);
    return AK_Success;
}

AKRESULT RaveWwiseFX::Reset()
{
    return AK_Success;
}

AKRESULT RaveWwiseFX::GetPluginInfo(AkPluginInfo& out_rPluginInfo)
{
    out_rPluginInfo.eType = AkPluginTypeEffect;
    out_rPluginInfo.bIsInPlace = false;
	out_rPluginInfo.bCanProcessObjects = false;
    out_rPluginInfo.uBuildVersion = AK_WWISESDK_VERSION_COMBINED;
    return AK_Success;
}

void RaveWwiseFX::Execute(AkAudioBuffer* in_pBuffer, AkUInt32 in_ulnOffset, AkAudioBuffer* out_pBuffer)
{
    const AkUInt32 uNumChannelsIn = in_pBuffer->NumChannels();
    const AkUInt32 uNumChannelsOut = out_pBuffer->NumChannels();

	// ----------------------------------------------------------------
    // Update RTPC params
    // TODO: Remaining params from RAVE VST

    const float outputDryWetPercentage = _fxParams->RTPC.fOutputDryWet;
    const int additionalLatencyCompensation = _fxParams->RTPC.iLatencyCompensationSamples;

	_latentScale.at(0) = _fxParams->RTPC.fLatent1Scale;
	_latentScale.at(1) = _fxParams->RTPC.fLatent2Scale;
	_latentScale.at(2) = _fxParams->RTPC.fLatent3Scale;
	_latentScale.at(3) = _fxParams->RTPC.fLatent4Scale;
	_latentScale.at(4) = _fxParams->RTPC.fLatent5Scale;
	_latentScale.at(5) = _fxParams->RTPC.fLatent6Scale;
	_latentScale.at(6) = _fxParams->RTPC.fLatent7Scale;
	_latentScale.at(7) = _fxParams->RTPC.fLatent8Scale;

	_latentBias.at(0) = _fxParams->RTPC.fLatent1Bias;
	_latentBias.at(1) = _fxParams->RTPC.fLatent2Bias;
	_latentBias.at(2) = _fxParams->RTPC.fLatent3Bias;
	_latentBias.at(3) = _fxParams->RTPC.fLatent4Bias;
	_latentBias.at(4) = _fxParams->RTPC.fLatent5Bias;
	_latentBias.at(5) = _fxParams->RTPC.fLatent6Bias;
	_latentBias.at(6) = _fxParams->RTPC.fLatent7Bias;
	_latentBias.at(7) = _fxParams->RTPC.fLatent8Bias;

	// ----------------------------------------------------------------
    // Setup book-keeping

	// RAVE
    const int nSamples = in_pBuffer->uValidFrames;
	const int nChannelsIn = (const int)uNumChannelsIn;
    const int nChannelsOut = (const int)uNumChannelsOut;
    
    if (!_modelLoaded.load())
    {
        _modelLoadTimeSamples += nSamples;
    }

    if (!_modelPerformed.load())
    {
        _modelPerformTimeSamples += nSamples;
    }
    
    // Effect plugin
	AkUInt16 uFramesConsumed = 0;
	AkUInt16 uFramesProduced = 0;

	// ----------------------------------------------------------------
	// Push input buffer contents to circular buffer

    if (nChannelsIn < 1)
    {
        return;
    }

    // Model input is mono, so we average all the input channels' sample values
    for (size_t i = 0; i < nSamples; ++i)
    {
        float sampleValue = 0.f;

        for (size_t inChannelIdx = 0; inChannelIdx < nChannelsIn; ++inChannelIdx)
        {
            AkReal32* AK_RESTRICT pInBuf = (AkReal32 * AK_RESTRICT)in_pBuffer->GetChannel(inChannelIdx) + in_ulnOffset;
            sampleValue += pInBuf[i];
        }

        sampleValue /= nChannelsIn;

        _inBuffer[0].push(sampleValue);
    }
    
    uFramesConsumed = nSamples;

	// ----------------------------------------------------------------
	// Create processing thread

	int currentRefreshRate = pow(2, _latencyMode);

	if (_inBuffer[0].len() >= currentRefreshRate) {
		if (_computeThread) {

#if DEBUG_PERFORM
			AKPLATFORM::OutputDebugMsg("Joining... ");
			AKPLATFORM::OutputDebugMsg("\n");
#endif

			_computeThread->join();
		}
		_inBuffer[0].get(_inModel[0].get(), currentRefreshRate);

        for (size_t i = 0; i < currentRefreshRate; ++i)
        {
            _dryBuffer[0].push(_inModel[0][i]);
        }

		_outBuffer[0].put(_outModel[0].get(), currentRefreshRate);
		_outBuffer[1].put(_outModel[1].get(), currentRefreshRate);

		_computeThread = std::make_unique<std::thread>(RaveWwise::modelPerform_callback, this);
	}

#if DEBUG_PERFORM
	AKPLATFORM::OutputDebugMsg("_inBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_inBuffer[0].len()).c_str());
	AKPLATFORM::OutputDebugMsg(", _dryBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_dryBuffer[0].len()).c_str());
	AKPLATFORM::OutputDebugMsg(", _outBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_outBuffer[0].len()).c_str());
    AKPLATFORM::OutputDebugMsg("\n");
#endif

    // ----------------------------------------------------------------
    // Write to final output buffer

    const int dryWetLatency = _modelLoadTimeSamples + additionalLatencyCompensation;

#if DEBUG_PERFORM
	AKPLATFORM::OutputDebugMsg("dryWetLatency = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(dryWetLatency).c_str());
	AKPLATFORM::OutputDebugMsg(" (currentRefreshRate = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(currentRefreshRate).c_str());
	AKPLATFORM::OutputDebugMsg(", nSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(nSamples).c_str());
	AKPLATFORM::OutputDebugMsg(", additionalLatencyCompensation = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(additionalLatencyCompensation).c_str());
	AKPLATFORM::OutputDebugMsg(", _modelLoadTimeSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_modelLoadTimeSamples).c_str());
	AKPLATFORM::OutputDebugMsg(", _modelPerformTimeSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_modelPerformTimeSamples).c_str());
	AKPLATFORM::OutputDebugMsg(")");
	AKPLATFORM::OutputDebugMsg("\n");
#endif

	if (_outBuffer[0].len() >= nSamples) {
		AkReal32* AK_RESTRICT pOutBufL = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(0) + out_pBuffer->uValidFrames;
		AkReal32* AK_RESTRICT pOutBufR = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(1) + out_pBuffer->uValidFrames;

        for (size_t i = 0; i < nSamples; i++)
        {
            const float wetSampleValueL = _outBuffer[0].pop();
            const float wetSampleValueR = _outBuffer[1].pop();

            // Delay dry signal based on latency to align with wet signal
            float drySampleValue = 0.f;
            if (_dryLatencySamplesElapsed >= dryWetLatency)
            { 
                drySampleValue = _dryBuffer[0].pop();
            }
            else
            {
                _dryLatencySamplesElapsed++;
            }

            if (i < out_pBuffer->MaxFrames())
            {
                const float wetAmount = outputDryWetPercentage * 0.01f;
                const float dryAmount = 1.f - wetAmount;

				const float wetSampleValue = (wetSampleValueL + wetSampleValueR) * 0.5f;

                // Copy averaged L/R output sample value to all channels of output buffer
                // TODO: Allow different behavior types, e.g. splitting  L/R output amongst the output channels
				const AkReal32 outputSampleValue = (dryAmount * drySampleValue) + (wetAmount * wetSampleValue);

				for (size_t outChannelIdx = 0; outChannelIdx < nChannelsOut; ++outChannelIdx)
				{
					AkReal32* AK_RESTRICT pOutBuf = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(outChannelIdx) + out_pBuffer->uValidFrames;
					pOutBuf[i] = outputSampleValue;
				}
            }
        }

        uFramesProduced = nSamples;

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Produced ");
		AKPLATFORM::OutputDebugMsg(std::to_string(nSamples).c_str());
		AKPLATFORM::OutputDebugMsg(" frames");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

	}
	else {

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Waiting...");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

	}

    // ----------------------------------------------------------------
    // Update effect plugin state

	in_pBuffer->uValidFrames -= uFramesConsumed;
	out_pBuffer->uValidFrames += uFramesProduced;

	if (in_pBuffer->eState == AK_NoMoreData && in_pBuffer->uValidFrames == 0)
		out_pBuffer->eState = AK_NoMoreData;
	else if (out_pBuffer->uValidFrames == out_pBuffer->MaxFrames())
		out_pBuffer->eState = AK_DataReady;
	else
		out_pBuffer->eState = AK_DataNeeded;

}

AKRESULT RaveWwiseFX::TimeSkip(AkUInt32 &io_uFrames)
{
    return AK_DataReady;
}

//----------------------------------------------------------------------------------------------------------------------

void RaveWwiseFX::ModelPerform()
{
    if (_rave.get() && !_isMuted.load())
    {
        c10::InferenceMode guard(true);

		at::Tensor latent_traj;
		at::Tensor latent_traj_mean;

        // --------------------------------
        // Setup parameters

        // Non-RTPC
        int input_size = static_cast<int>(pow(2, _latencyMode));

        // RTPC
		const bool use_prior = _fxParams->RTPC.bUsePrior;
		const float prior_temperature = _fxParams->RTPC.fPriorTemperature;
        const float jitter_amount = _fxParams->RTPC.fLatentJitter;
        const float width = _fxParams->RTPC.fOutputWidth * 0.01f;

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("\n");
        AKPLATFORM::OutputDebugMsg("ModelPerform():");
		AKPLATFORM::OutputDebugMsg("\n");

        // Dump full model to string
		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg(_rave->dump_to_str().c_str());
		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg("\n");

		AKPLATFORM::OutputDebugMsg("Latency mode exponent: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_latencyMode).c_str());
		AKPLATFORM::OutputDebugMsg(", Input size value: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(input_size).c_str());
		AKPLATFORM::OutputDebugMsg("\n");

		AKPLATFORM::OutputDebugMsg("Has prior: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_rave->hasPrior()).c_str());
		AKPLATFORM::OutputDebugMsg(", Use prior: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(use_prior).c_str());
		AKPLATFORM::OutputDebugMsg("\n");

		AKPLATFORM::OutputDebugMsg("Prior temperature: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(prior_temperature).c_str());
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        if (_rave->hasPrior() && use_prior)
        {
            // TODO: Prior performance seems choppy -- this is also the case with RAVE VST on Windows
            auto n_trajs = pow(2, _latencyMode) / _rave->getModelRatio();
            latent_traj = _rave->sample_prior((int)n_trajs, prior_temperature);
            latent_traj_mean = latent_traj;
        }
        else
        {
            int64_t sizes = { input_size };
            at::Tensor frame = torch::from_blob(_inModel[0].get(), sizes);
            frame = torch::reshape(frame, { 1, 1, input_size });

#if DEBUG_PERFORM
            AKPLATFORM::OutputDebugMsg("Current input size: ");
            for (const auto input_size : frame.sizes())
            {
	            AKPLATFORM::OutputDebugMsg(std::to_string(input_size).c_str());
	            AKPLATFORM::OutputDebugMsg(", ");
            }
            AKPLATFORM::OutputDebugMsg("\n");
#endif

			// --------------------------------
            // Encode
            if (_rave->hasMethod("encode_amortized"))
            {

                // Mean and std from encode_amortized()
                std::vector<torch::Tensor> latent_probs = _rave->encode_amortized(frame);
                latent_traj_mean = latent_probs[0];
                at::Tensor latent_traj_std = latent_probs[1];

#if DEBUG_PERFORM
				AKPLATFORM::OutputDebugMsg("Mean shape: ");
				for (const auto mean_size : latent_traj_mean.sizes())
				{
					AKPLATFORM::OutputDebugMsg(std::to_string(mean_size).c_str());
					AKPLATFORM::OutputDebugMsg(", ");
				}
				AKPLATFORM::OutputDebugMsg("\n");

				AKPLATFORM::OutputDebugMsg("Std shape: ");
				for (const auto std_size : latent_traj_std.sizes())
				{
					AKPLATFORM::OutputDebugMsg(std::to_string(std_size).c_str());
					AKPLATFORM::OutputDebugMsg(", ");
				}
				AKPLATFORM::OutputDebugMsg("\n");
#endif

                latent_traj = latent_traj_mean +
                    latent_traj_std * torch::randn_like(latent_traj_mean);
            }
            else
            {
                // Regular encode()
                latent_traj = _rave->encode(frame);
                latent_traj_mean = latent_traj;

            }
        }

#if DEBUG_PERFORM
        AKPLATFORM::OutputDebugMsg("Latent traj shape: ");
        for (const auto latent_traj_size : latent_traj.sizes())
        {
	        AKPLATFORM::OutputDebugMsg(std::to_string(latent_traj_size).c_str());
	        AKPLATFORM::OutputDebugMsg(", ");
        }
        AKPLATFORM::OutputDebugMsg("\n");
#endif
        // --------------------------------
        // Latent modifications

        // Apply scale and bias
        int64_t n_dimensions =
            std::min((int)latent_traj.size(1), (int)AVAILABLE_DIMS);
        for (int i = 0; i < n_dimensions; i++)
        {
            const float scale = _latentScale.at(i);
            const float bias = _latentBias.at(i);

            latent_traj.index_put_({ 0, i },
                (latent_traj.index({ 0, i }) * scale + bias));
            
            latent_traj_mean.index_put_({ 0, i },
                (latent_traj_mean.index({ 0, i }) * scale + bias));
        }
        _rave->writeLatentBuffer(latent_traj_mean);

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Latent scale & bias applied");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        // Add latent jitter on meaningful dimensions
        latent_traj = latent_traj + jitter_amount * torch::randn_like(latent_traj);

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Jitter applied");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        // Fill missing dimensions with width parameter

		int missing_dims = _rave->getFullLatentDimensions() - latent_traj.size(1);

		if (_rave->isStereo() && missing_dims > 0)
		{

			torch::Tensor latent_trajL = latent_traj,
				latent_trajR = latent_traj.clone();

#if DEBUG_PERFORM
		    AKPLATFORM::OutputDebugMsg("Latent size: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(_rave->getFullLatentDimensions()).c_str());
		    AKPLATFORM::OutputDebugMsg(", latent traj size: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(latent_trajL.size(1)).c_str());
		    AKPLATFORM::OutputDebugMsg(", missing dimensions: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(missing_dims).c_str());

		    AKPLATFORM::OutputDebugMsg("\n");
#endif

			at::Tensor latent_noiseL =
				torch::randn({ 1, missing_dims, latent_trajL.size(2) });
			at::Tensor latent_noiseR =
				(1 - width) * latent_noiseL +
				width * torch::randn({ 1, missing_dims, latent_trajL.size(2) });

#if DEBUG_PERFORM
			AKPLATFORM::OutputDebugMsg("After width: ");
			for (const auto latentNoiseLSize : latent_noiseL.sizes())
			{
				AKPLATFORM::OutputDebugMsg(std::to_string(latentNoiseLSize).c_str());
				AKPLATFORM::OutputDebugMsg(", ");
			}
			AKPLATFORM::OutputDebugMsg("\n");
#endif

			latent_trajL = torch::cat({ latent_trajL, latent_noiseL }, 1);
			latent_trajR = torch::cat({ latent_trajR, latent_noiseR }, 1);
        

#if DEBUG_PERFORM
		    AKPLATFORM::OutputDebugMsg("Latent processed");
		    AKPLATFORM::OutputDebugMsg("\n");
#endif

            latent_traj = torch::cat({ latent_trajL, latent_trajR }, 0);

#if DEBUG_PERFORM
			AKPLATFORM::OutputDebugMsg("New latent traj shape: ");
			for (const auto latent_traj_size : latent_traj.sizes())
			{
				AKPLATFORM::OutputDebugMsg(std::to_string(latent_traj_size).c_str());
				AKPLATFORM::OutputDebugMsg(", ");
			}
			AKPLATFORM::OutputDebugMsg("\n");
#endif
        }

        // --------------------------------
		// Decode
        at::Tensor out = _rave->decode(latent_traj);

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Out shape: ");
        for (const auto out_size : out.sizes())
        {
			AKPLATFORM::OutputDebugMsg(std::to_string(out_size).c_str());
			AKPLATFORM::OutputDebugMsg(", ");
        }
		AKPLATFORM::OutputDebugMsg("\n");
#endif
        
        // From RAVE VST:
        // """
        // On windows, I don't get why, but the two first dims are swapped (compared
        // to macOS / UNIX) with the same torch version
        // """
        if (out.sizes()[0] == 2) {
            out = out.transpose(0, 1);
        }

        const int out_indexL = 0;
		const int out_indexR = (out.sizes()[1] > 1 ? 1 : 0); // Use right channel only if available
		at::Tensor outL = out.index({ 0, out_indexL, at::indexing::Slice() });
		at::Tensor outR = out.index({ 0, out_indexR, at::indexing::Slice() });

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Latent decoded");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        float* out_dataPtrL, * out_dataPtrR;
        out_dataPtrL = outL.data_ptr<float>();
        out_dataPtrR = outR.data_ptr<float>();

        // Write to output buffers
        assert(input_size >= 0);
        for (size_t i = 0; i < (size_t)input_size; i++) {
            _outModel[0][i] = out_dataPtrL[i];
            _outModel[1][i] = out_dataPtrR[i];
        }

        _modelPerformed.store(true);
    }
    else {

#if DEBUG_PERFORM
        AKPLATFORM::OutputDebugMsg("ModelPerform(): Sorry, model is not ready yet");
        AKPLATFORM::OutputDebugMsg("\n");
#endif

    }
}

void RaveWwiseFX::DetectAvailableModels()
{
	// TODO
}

void RaveWwiseFX::Mute()
{
    _isMuted.store(true);
}

void RaveWwiseFX::Unmute()
{
    _isMuted.store(false);
}

void RaveWwiseFX::UpdateBufferSizes()
{
    auto validBufferSizes = _rave->getValidBufferSizes();
    float a = validBufferSizes.start;
    float b = validBufferSizes.end;

#if DEBUG_PERFORM
	AKPLATFORM::OutputDebugMsg("\n");
	AKPLATFORM::OutputDebugMsg("UpdateBufferSizes(): ");
	AKPLATFORM::OutputDebugMsg("\n");

	AKPLATFORM::OutputDebugMsg("Initial latency mode: ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_latencyMode).c_str());
	AKPLATFORM::OutputDebugMsg(" -- valid buffer sizes range from ");
	AKPLATFORM::OutputDebugMsg(std::to_string(a).c_str());
	AKPLATFORM::OutputDebugMsg(" to ");
	AKPLATFORM::OutputDebugMsg(std::to_string(b).c_str());
	AKPLATFORM::OutputDebugMsg("\n");
#endif

    const float bufferSize = pow(2, _latencyMode);
    if (bufferSize < a)
    {

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Latency mode too low; setting rate to: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(log2(a)).c_str());
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        _latencyMode = static_cast<int>(log2(a));
    }
    else if (bufferSize > b)
    {

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("Latency mode too high; setting rate to: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(log2(b)).c_str());
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        _latencyMode = static_cast<int>(log2(b));
    }
}

void RaveWwiseFX::UpdateEngine(const std::string& modelFile)
{
    if (modelFile == _loadedModelName)
    {
        return;
    }

    _loadedModelName = modelFile;

    std::scoped_lock irCalculationLock(_engineUpdateMutex);

    if (_engineThreadPool)
    {
        _engineThreadPool->purge();
    }

    std::future<void> engineUpdateFuture = _engineThreadPool->submit(RaveWwise::UpdateEngineJob,
                                                                     this,
                                                                     modelFile,
                                                                     1);
}
