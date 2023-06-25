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
	void modelPerform_callback(RaveWwiseFX* ap) { ap->modelPerform(); }
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

RaveWwiseFX::RaveWwiseFX()
    : m_pParams(nullptr)
    , m_pAllocator(nullptr)
    , m_pContext(nullptr)
{
    // TODO: Make sure all dynamic allocations use m_pAllocator

    _loadedModelName = "";
    _computeThread = nullptr;

    _inBuffer = std::make_unique<circular_buffer<float, float>[]>(1);
	_dryBuffer = std::make_unique<circular_buffer<float, float>[]>(1);
    _outBuffer = std::make_unique<circular_buffer<float, float>[]>(2);
	_inModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
	_outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
	_outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
    
    _engineThreadPool = std::make_unique<BS::thread_pool>(1);
    
    _rave.reset(new RAVE()); // TODO: Use m_pAllocator in Init()

    // TODO: Parameter listener equivalents

    _editorReady = false;
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
    m_pParams = (RaveWwiseFXParams*)in_pParams;
    m_pAllocator = in_pAllocator;
    m_pContext = in_pContext;

	// --------
    // Allocation for RAVE members

 //   _ravePtr = AK_PLUGIN_NEW(m_pAllocator, RAVE());
 //   _rave.reset(_ravePtr);

 //   _inModelBuf = (float*)(AK_PLUGIN_ALLOC(m_pAllocator, sizeof(float) * BUFFER_LENGTH));
 //   _inModel.push_back(_inModelBuf);

 //   _outModelBufL = (float*)(AK_PLUGIN_ALLOC(m_pAllocator, sizeof(float) * BUFFER_LENGTH));
 //   _outModelBufR = (float*)(AK_PLUGIN_ALLOC(m_pAllocator, sizeof(float) * BUFFER_LENGTH));
 //   _outModel.push_back(_outModelBufL);
 //   _outModel.push_back(_outModelBufR);

    // --------
    // RaveAP::prepareToPlay()

	//_sampleRate = sampleRate;
	_inBuffer[0].initialize(BUFFER_LENGTH);
	_dryBuffer[0].initialize(DRY_BUFFER_LENGTH);
	_outBuffer[0].initialize(BUFFER_LENGTH);
	_outBuffer[1].initialize(BUFFER_LENGTH);
	//setLatencySamples(pow(2, *_latencyMode));

    // --------
    // Load the model

    // NOTE: We do direct AkOSChar*-to-char* conversion via casting here, since CONVERT_OSCHAR_TO_CHAR() returns incorrect results
    //
    // TODO: - Escape backslashes for file paths ("\" --> "\\")
    //       - Move the char-conversion/string-creation to RaveWwiseFXParams?
	//       - Move entire model loading routine to plugin library registration step, to avoid re-loading unnecessarily with every effect instantiation at runtime? See https://www.audiokinetic.com/en/library/edge/?source=SDK&id=soundengine_plugins.html#fx_global_hooks

    _modelLoaded = false;
    _modelLoadTimeSamples = 0;
    _modelPerformed = false;
    _modelPerformTimeSamples = 0;
    _dryLatencySamplesElapsed = 0;

    AkOSChar* modelFilePathOsStr = m_pParams->NonRTPC.sModelFilePath;
    char* modelFilePathCStr = reinterpret_cast<char*>(modelFilePathOsStr);
    std::string modelFilePath = std::string(modelFilePathCStr);
    updateEngine(modelFilePath);

    return AK_Success;
}

AKRESULT RaveWwiseFX::Term(AK::IAkPluginMemAlloc* in_pAllocator)
{
 //   AK_PLUGIN_DELETE(in_pAllocator, _ravePtr);
 //   AK_PLUGIN_FREE(in_pAllocator, _inModelBuf);
 //   AK_PLUGIN_FREE(in_pAllocator, _outModelBufL);
 //   AK_PLUGIN_FREE(in_pAllocator, _outModelBufR);

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

    // TODO: Rest of params
    _dryWetValue = m_pParams->RTPC.fOutputDryWet;
    _additionalLatencyCompensation = m_pParams->RTPC.iLatencyCompensationSamples;

	// ----------------------------------------------------------------
    // Setup book-keeping

	// RAVE
    const int nSamples = in_pBuffer->uValidFrames;
	const int nChannelsIn = (const int)uNumChannelsIn;
    const int nChannelsOut = (const int)uNumChannelsOut;
    
    if (!_modelLoaded)
    {
        _modelLoadTimeSamples += nSamples;
    }

    if (!_modelPerformed)
    {
        _modelPerformTimeSamples += nSamples;
    }
    
    // Wwise out-of-place effect plugin
	AkUInt16 uFramesConsumed = 0;
	AkUInt16 uFramesProduced = 0;

    //for (AkUInt32 i = 0; i < uNumChannelsIn; ++i)
    //{
    //    AkReal32* AK_RESTRICT pInBuf = (AkReal32* AK_RESTRICT)in_pBuffer->GetChannel(i) + in_ulnOffset;
    //    AkReal32* AK_RESTRICT pOutBuf = (AkReal32* AK_RESTRICT)out_pBuffer->GetChannel(i) +  out_pBuffer->uValidFrames;

    //    uFramesConsumed = 0;
    //    uFramesProduced = 0;
    //    while (uFramesConsumed < in_pBuffer->uValidFrames
    //        && uFramesProduced < out_pBuffer->MaxFrames())
    //    {
    //         // Execute DSP that consumes input and produces output at different rate here
    //        *pOutBuf++ = *pInBuf++;
    //        ++uFramesConsumed;
    //        ++uFramesProduced;
    //    }
    //}


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
			std::cout << "joining..." << std::endl;
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

        //if (_modelLoaded)
        //{ 
		    _outBuffer[0].put(_outModel[0].get(), currentRefreshRate);
		    _outBuffer[1].put(_outModel[1].get(), currentRefreshRate);
        //}

		_computeThread = std::make_unique<std::thread>(RaveWwise::modelPerform_callback, this);

		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg("_inModel: ");
  //      for (size_t i = 0; i < currentRefreshRate; ++i)
  //      {
  //          AKPLATFORM::OutputDebugMsg(std::to_string(_inModel[0][i]).c_str());
		//	AKPLATFORM::OutputDebugMsg(", ");
  //      }
		//AKPLATFORM::OutputDebugMsg("\n");

		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg("_outModel: ");
		//for (size_t i = 0; i < currentRefreshRate; ++i)
		//{
		//	AKPLATFORM::OutputDebugMsg(std::to_string(_outModel[0][i]).c_str());
		//	AKPLATFORM::OutputDebugMsg(", ");
		//}
		//AKPLATFORM::OutputDebugMsg("\n");
	}

	AKPLATFORM::OutputDebugMsg("_inBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_inBuffer[0].len()).c_str());
	AKPLATFORM::OutputDebugMsg(", _dryBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_dryBuffer[0].len()).c_str());
	AKPLATFORM::OutputDebugMsg(", _outBuffer[0].len() = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_outBuffer[0].len()).c_str());
    AKPLATFORM::OutputDebugMsg("\n");

    // ----------------------------------------------------------------
    // Write to final output buffer

	//const int dryWetLatency = _modelLoadTimeSamples + nSamples; // + currentRefreshRate;
    //const int dryWetLatency = _modelPerformTimeSamples + _additionalLatencyCompensation; // - currentRefreshRate;
    const int dryWetLatency = _modelLoadTimeSamples + _additionalLatencyCompensation;
	AKPLATFORM::OutputDebugMsg("dryWetLatency = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(dryWetLatency).c_str());
	AKPLATFORM::OutputDebugMsg(" (currentRefreshRate = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(currentRefreshRate).c_str());
	AKPLATFORM::OutputDebugMsg(", nSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(nSamples).c_str());
	AKPLATFORM::OutputDebugMsg(", _additionalLatencyCompensation = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_additionalLatencyCompensation).c_str());
	AKPLATFORM::OutputDebugMsg(", _modelLoadTimeSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_modelLoadTimeSamples).c_str());
	AKPLATFORM::OutputDebugMsg(", _modelPerformTimeSamples = ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_modelPerformTimeSamples).c_str());
	AKPLATFORM::OutputDebugMsg(")");
	AKPLATFORM::OutputDebugMsg("\n");

	//if (_dryLatencySamplesElapsed < dryWetLatency)
	//{
	//	_dryLatencySamplesElapsed += nSamples;
	//}

	if (_outBuffer[0].len() >= nSamples) {
		AkReal32* AK_RESTRICT pOutBufL = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(0) + out_pBuffer->uValidFrames;
		AkReal32* AK_RESTRICT pOutBufR = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(1) + out_pBuffer->uValidFrames;

		//_outBuffer[0].get((float*)pOutBufL, nSamples);
		//_outBuffer[1].get((float*)pOutBufR, nSamples);

        for (size_t i = 0; i < nSamples; i++)
        {
            const float wetSampleValueL = _outBuffer[0].pop();
            const float wetSampleValueR = _outBuffer[1].pop();

            //const float drySampleValue = _inBuffer[0].peek(i);
            //int dryIndex = i - dryWetLatency;
            //const float drySampleValue = _dryBuffer[0].peek(i, dryWetLatency);

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
                const float wetAmount = _dryWetValue * 0.01f;
                const float dryAmount = 1.f - wetAmount;

				const float wetSampleValue = (wetSampleValueL + wetSampleValueR) * 0.5f;

                // L/R
				//const AkReal32 outputSampleValueL = (dryAmount * drySampleValue) + (wetAmount * wetSampleValueL);
				//const AkReal32 outputSampleValueR = (dryAmount * drySampleValue) + (wetAmount * wetSampleValueR);
				//pOutBufL[i] = outputSampleValueL;
				//pOutBufR[i] = outputSampleValueR;

                // Copy averaged L/R output sample value to all channels of output buffer
                // TODO: Split L/R output amongst the output channels

				const AkReal32 outputSampleValue = (dryAmount * drySampleValue) + (wetAmount * wetSampleValue);

				for (size_t outChannelIdx = 0; outChannelIdx < nChannelsOut; ++outChannelIdx)
				{
					AkReal32* AK_RESTRICT pOutBuf = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(outChannelIdx) + out_pBuffer->uValidFrames;
					pOutBuf[i] = outputSampleValue;
				}
            }
        }

        uFramesProduced = nSamples;
		AKPLATFORM::OutputDebugMsg("Produced ");
		AKPLATFORM::OutputDebugMsg(std::to_string(nSamples).c_str());
		AKPLATFORM::OutputDebugMsg(" frames");
		AKPLATFORM::OutputDebugMsg("\n");
	}
	else {
		//out_buffer.clear();

        // Clear output buffers
        // TODO: Clear all channels, not just L/R

		//AkReal32* AK_RESTRICT pOutBufL = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(0) + out_pBuffer->uValidFrames;
		//AkReal32* AK_RESTRICT pOutBufR = (AkReal32 * AK_RESTRICT)out_pBuffer->GetChannel(1) + out_pBuffer->uValidFrames;

		//for (size_t i = 0; i < nSamples; i++)
		//{
		//	pOutBufL[i] = 0.f;
		//	pOutBufR[i] = 0.f;
		//}

        //uFramesProduced = nSamples;

		AKPLATFORM::OutputDebugMsg("Waiting...");
		AKPLATFORM::OutputDebugMsg("\n");
	}

//#if DEBUG
//	std::cout << "buffer out : " << out_buffer.getMagnitude(0, nSamples)
//		<< std::endl;
//#endif
//
//	buffer.copyFrom(0, 0, out_buffer, 0, 0, nSamples);
//	if (nChannelsIn == 2)
//		buffer.copyFrom(1, 0, out_buffer, 1, 0, nSamples);
//
//
//#if DEBUG_PERFORM
//	std::cout << "sortie : " << buffer.getMagnitude(0, nSamples) << std::endl;
//#endif

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

void RaveWwiseFX::modelPerform()
{
    if (_rave.get() && !_isMuted.load()) {
        c10::InferenceMode guard(true);

        // --------------------------------
        // Setup parameters

        int input_size = static_cast<int>(pow(2, _latencyMode));

        at::Tensor latent_traj;
        at::Tensor latent_traj_mean;

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("\n");
        AKPLATFORM::OutputDebugMsg("modelPerform():");
		AKPLATFORM::OutputDebugMsg("\n");

        // Dump full model to string
		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg(_rave->dump_to_str().c_str());
		//AKPLATFORM::OutputDebugMsg("\n");
		//AKPLATFORM::OutputDebugMsg("\n");

        std::cout << "exp: " << _latencyMode << " value: " << input_size << '\n';
		AKPLATFORM::OutputDebugMsg("Latency mode exponent: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_latencyMode).c_str());
		AKPLATFORM::OutputDebugMsg(", Input size value: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(input_size).c_str());
		AKPLATFORM::OutputDebugMsg("\n");


        std::cout << "has prior : " << _rave->hasPrior()
            << "; use prior : " << _usePrior << std::endl;
		AKPLATFORM::OutputDebugMsg("Has prior: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_rave->hasPrior()).c_str());
		AKPLATFORM::OutputDebugMsg(", Use prior: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_usePrior).c_str());
		AKPLATFORM::OutputDebugMsg("\n");

        std::cout << "temperature : " << _priorTemperature << std::endl;
		AKPLATFORM::OutputDebugMsg("Prior temperature: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(_priorTemperature).c_str());
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        if (_rave->hasPrior() && _usePrior) {
            auto n_trajs = pow(2, _latencyMode) / _rave->getModelRatio();
            latent_traj = _rave->sample_prior((int)n_trajs, _priorTemperature);
            latent_traj_mean = latent_traj;
        }
        else {
            int64_t sizes = { input_size };
            at::Tensor frame = torch::from_blob(_inModel[0].get(), sizes);
            frame = torch::reshape(frame, { 1, 1, input_size });

#if DEBUG_PERFORM
            std::cout << "Current input size : " << frame.sizes() << std::endl;

            AKPLATFORM::OutputDebugMsg("Current input size: ");
            for (const auto inputSize : frame.sizes())
            {
	            AKPLATFORM::OutputDebugMsg(std::to_string(inputSize).c_str());
	            AKPLATFORM::OutputDebugMsg(", ");
            }
            AKPLATFORM::OutputDebugMsg("\n");
#endif DEBUG_PERFORM

			// --------------------------------
            // Encode
            if (_rave->hasMethod("encode_amortized")) {

                // Mean and std from encode_amortized()
                std::vector<torch::Tensor> latent_probs = _rave->encode_amortized(frame);
                latent_traj_mean = latent_probs[0];
                at::Tensor latent_traj_std = latent_probs[1];

#if DEBUG_PERFORM
                std::cout << "mean shape" << latent_traj_mean.sizes() << std::endl;
                std::cout << "std shape" << latent_traj_std.sizes() << std::endl;
#endif

                latent_traj = latent_traj_mean +
                    latent_traj_std * torch::randn_like(latent_traj_mean);
            }
            else {

                // Regular encode()
                latent_traj = _rave->encode(frame);
                latent_traj_mean = latent_traj;
            }
        }

#if DEBUG_PERFORM
        std::cout << "latent traj shape" << latent_traj.sizes() << std::endl;

        AKPLATFORM::OutputDebugMsg("latent traj shape: ");
        for (const auto latentTrajSize : latent_traj.sizes())
        {
	        AKPLATFORM::OutputDebugMsg(std::to_string(latentTrajSize).c_str());
	        AKPLATFORM::OutputDebugMsg(", ");
        }
        AKPLATFORM::OutputDebugMsg("\n");
#endif
        // --------------------------------
        // Latent modifications
        // apply scale and bias
        int64_t n_dimensions =
            std::min((int)latent_traj.size(1), (int)AVAILABLE_DIMS);
        for (int i = 0; i < n_dimensions; i++) {
            
            // From RAVE VST:
            // """
            // The assert and casting here is needed as I got a:
            // warning: conversion to ‘std::array<std::atomic<float>*,
            // 8>::size_type’ {aka ‘long unsigned int’} from ‘int’ may change the
            // sign of the result [-Wsign-conversion]
            // Whatever AVAILABLE_DIMS type I defined
            // """
            assert(i >= 0);
            auto i2 = (long unsigned int)i;
            float scale = _latentScale.at(i2).load();
            float bias = _latentBias.at(i2).load();
            latent_traj.index_put_({ 0, i },
                (latent_traj.index({ 0, i }) * scale + bias));
            latent_traj_mean.index_put_(
                { 0, i }, (latent_traj_mean.index({ 0, i }) * scale + bias));
        }
        _rave->writeLatentBuffer(latent_traj_mean);

#if DEBUG_PERFORM
        std::cout << "scale & bias applied" << std::endl;

		AKPLATFORM::OutputDebugMsg("scale & bias applied");
		AKPLATFORM::OutputDebugMsg("\n");
#endif
		// --------------------------------
        // Add latent jitter on meaningful dimensions
        float jitter_amount = _latentJitterValue.load();
        latent_traj = latent_traj + jitter_amount * torch::randn_like(latent_traj);

#if DEBUG_PERFORM
        std::cout << "jitter applied" << std::endl;

		AKPLATFORM::OutputDebugMsg("jitter applied");
		AKPLATFORM::OutputDebugMsg("\n");
#endif
		// --------------------------------
        // Fill missing dimensions with width parameter

		int missing_dims = _rave->getFullLatentDimensions() - latent_traj.size(1);

		if (_rave->isStereo() && missing_dims > 0)
		{

			torch::Tensor latent_trajL = latent_traj,
				latent_trajR = latent_traj.clone();

#if DEBUG_PERFORM
		    AKPLATFORM::OutputDebugMsg("latent size: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(_rave->getFullLatentDimensions()).c_str());
		    AKPLATFORM::OutputDebugMsg(", latent traj size: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(latent_trajL.size(1)).c_str());
		    AKPLATFORM::OutputDebugMsg(", missing dimensions: ");
		    AKPLATFORM::OutputDebugMsg(std::to_string(missing_dims).c_str());

		    AKPLATFORM::OutputDebugMsg("\n");
#endif


			float width = _widthValue.load() / 100.f;
			at::Tensor latent_noiseL =
				torch::randn({ 1, missing_dims, latent_trajL.size(2) });
			at::Tensor latent_noiseR =
				(1 - width) * latent_noiseL +
				width * torch::randn({ 1, missing_dims, latent_trajL.size(2) });

#if DEBUG_PERFORM
			std::cout << "after width : " << latent_noiseL.sizes() << ";"
				<< latent_trajL.sizes() << std::endl;

			AKPLATFORM::OutputDebugMsg("after width: ");
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
		    std::cout << "latent processed" << std::endl;

		    AKPLATFORM::OutputDebugMsg("latent processed");
		    AKPLATFORM::OutputDebugMsg("\n");
#endif

            latent_traj = torch::cat({ latent_trajL, latent_trajR }, 0);

#if DEBUG_PERFORM
			std::cout << "new latent traj shape" << latent_traj.sizes() << std::endl;

			AKPLATFORM::OutputDebugMsg("new latent traj shape: ");
			for (const auto latentTrajSize : latent_traj.sizes())
			{
				AKPLATFORM::OutputDebugMsg(std::to_string(latentTrajSize).c_str());
				AKPLATFORM::OutputDebugMsg(", ");
			}
			AKPLATFORM::OutputDebugMsg("\n");
#endif
        }

        // --------------------------------
		// Decode
        at::Tensor out = _rave->decode(latent_traj);

#if DEBUG_PERFORM
		AKPLATFORM::OutputDebugMsg("out shape: ");
        for (const auto outSize : out.sizes())
        {
			AKPLATFORM::OutputDebugMsg(std::to_string(outSize).c_str());
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

		const int outIndexR = (out.sizes()[1] > 1 ? 1 : 0); // Use right channel if available
		at::Tensor outL = out.index({ 0, 0, at::indexing::Slice() });
		at::Tensor outR = out.index({ 0, outIndexR, at::indexing::Slice() });

#if DEBUG_PERFORM
        std::cout << "latent decoded" << std::endl;

		AKPLATFORM::OutputDebugMsg("latent decoded");
		AKPLATFORM::OutputDebugMsg("\n");
#endif

        float* outputDataPtrL, * outputDataPtrR;
        outputDataPtrL = outL.data_ptr<float>();
        outputDataPtrR = outR.data_ptr<float>();

        // Write in buffers
        assert(input_size >= 0);
		//AKPLATFORM::OutputDebugMsg("\n");
        for (size_t i = 0; i < (size_t)input_size; i++) {
            _outModel[0][i] = outputDataPtrL[i];
            _outModel[1][i] = outputDataPtrR[i];

            //AKPLATFORM::OutputDebugMsg(std::to_string(outputDataPtrL[i]).c_str());
			//AKPLATFORM::OutputDebugMsg(std::to_string(_outModel[0][i]).c_str());
            //AKPLATFORM::OutputDebugMsg(", ");
        }
		//AKPLATFORM::OutputDebugMsg("\n");

        // TODO
        //if (_smoothedFadeInOut.getCurrentValue() < EPSILON) {
        //    _isMuted.store(true);
        //}

        _modelPerformed = true;
    }
    else {
        AKPLATFORM::OutputDebugMsg("modelPerform(): Sorry, model is not ready yet");
        AKPLATFORM::OutputDebugMsg("\n");
    }
}

void RaveWwiseFX::detectAvailableModels()
{
	// TODO
}

void RaveWwiseFX::mute()
{
    _fadeScheduler.store(muting::mute);
    _isMuted.store(true);
}

void RaveWwiseFX::unmute()
{
    _fadeScheduler.store(muting::unmute);
    _isMuted.store(false);
}

void RaveWwiseFX::updateBufferSizes()
{
    auto validBufferSizes = _rave->getValidBufferSizes();
    float a = validBufferSizes.start;
    float b = validBufferSizes.end;

	AKPLATFORM::OutputDebugMsg("\n");
	AKPLATFORM::OutputDebugMsg("updateBufferSizes(): ");
	AKPLATFORM::OutputDebugMsg("\n");

	AKPLATFORM::OutputDebugMsg("Initial latency mode: ");
	AKPLATFORM::OutputDebugMsg(std::to_string(_latencyMode).c_str());
	AKPLATFORM::OutputDebugMsg(" -- valid buffer sizes range from ");
	AKPLATFORM::OutputDebugMsg(std::to_string(a).c_str());
	AKPLATFORM::OutputDebugMsg(" to ");
	AKPLATFORM::OutputDebugMsg(std::to_string(b).c_str());
	AKPLATFORM::OutputDebugMsg("\n");

    const float bufferSize = pow(2, _latencyMode);
    if (bufferSize < a)
    {
        std::cout << "too low; setting rate to : " << static_cast<int>(log2(a))
            << std::endl;
		AKPLATFORM::OutputDebugMsg("Latency mode too low; setting rate to: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(log2(a)).c_str());
		AKPLATFORM::OutputDebugMsg("\n");

        _latencyMode = static_cast<int>(log2(a));
    }
    else if (bufferSize > b)
    {
        std::cout << "too high; setting rate to : " << static_cast<int>(log2(b))
            << std::endl;
		AKPLATFORM::OutputDebugMsg("Latency mode too high; setting rate to: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(log2(b)).c_str());
		AKPLATFORM::OutputDebugMsg("\n");

        _latencyMode = static_cast<int>(log2(b));
    }
}

void RaveWwiseFX::updateEngine(const std::string& modelFile)
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

    std::future<void> engineUpdateFuture = _engineThreadPool->submit(RaveWwise::UpdateEngineJob, this, modelFile, 1);
}
