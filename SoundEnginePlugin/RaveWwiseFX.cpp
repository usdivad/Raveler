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

#define DEBUG_PERFORM 0

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

    // TODO
    //_dryWetMixerEffect = new juce::dsp::DryWetMixer<float>(BUFFER_LENGTH);
    //_dryWetMixerEffect.setMixingRule(juce::dsp::DryWetMixingRule::balanced);

    _inBuffer = std::make_unique<circular_buffer<float, float>[]>(1);
    _outBuffer = std::make_unique<circular_buffer<float, float>[]>(2);
    _inModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
    _outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));
    _outModel.push_back(std::make_unique<float[]>(BUFFER_LENGTH));

    // TODO: Initialize _inputGainValue through _priorTemperature

    // TODO: Initialize internal values
    _latentScale = new std::array<std::atomic<float>*, AVAILABLE_DIMS>;
    _latentBias = new std::array<std::atomic<float>*, AVAILABLE_DIMS>;
    
    _engineThreadPool = std::make_unique<BS::thread_pool>(1);
    
    _rave.reset(new RAVE());

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
    // Load the model

    // NOTE: We do direct AkOSChar*-to-char* conversion via casting here, since CONVERT_OSCHAR_TO_CHAR() returns incorrect results
    //
    // TODO: - Escape backslashes for file paths ("\" --> "\\")
    //       - Move the char-conversion/string-creation to RaveWwiseFXParams?
	//       - Move entire model loading routine to plugin library registration step, to avoid re-loading unnecessarily with every effect instantiation at runtime? See https://www.audiokinetic.com/en/library/edge/?source=SDK&id=soundengine_plugins.html#fx_global_hooks

    AkOSChar* modelFilePathOsStr = m_pParams->NonRTPC.sModelFilePath;
    char* modelFilePathCStr = reinterpret_cast<char*>(modelFilePathOsStr);
    std::string modelFilePath = std::string(modelFilePathCStr);
    updateEngine(modelFilePath);

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
    const AkUInt32 uNumChannels = in_pBuffer->NumChannels();

    AkUInt16 uFramesConsumed;
    AkUInt16 uFramesProduced;
    for (AkUInt32 i = 0; i < uNumChannels; ++i)
    {
        AkReal32* AK_RESTRICT pInBuf = (AkReal32* AK_RESTRICT)in_pBuffer->GetChannel(i) + in_ulnOffset;
        AkReal32* AK_RESTRICT pOutBuf = (AkReal32* AK_RESTRICT)out_pBuffer->GetChannel(i) +  out_pBuffer->uValidFrames;

        uFramesConsumed = 0;
        uFramesProduced = 0;
        while (uFramesConsumed < in_pBuffer->uValidFrames
            && uFramesProduced < out_pBuffer->MaxFrames())
        {
             // Execute DSP that consumes input and produces output at different rate here
            *pOutBuf++ = *pInBuf++;
            ++uFramesConsumed;
            ++uFramesProduced;
        }
    }

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
        // encode
        int input_size = static_cast<int>(pow(2, *_latencyMode));

        at::Tensor latent_traj;
        at::Tensor latent_traj_mean;

#if DEBUG_PERFORM
        std::cout << "exp: " << *_latencyMode << " value: " << input_size << '\n';
        std::cout << "has prior : " << _rave->hasPrior()
            << "; use prior : " << *_usePrior << std::endl;
        std::cout << "temperature : " << *_priorTemperature << std::endl;
#endif

        if (_rave->hasPrior() && *_usePrior) {
            auto n_trajs = pow(2, *_latencyMode) / _rave->getModelRatio();
            latent_traj = _rave->sample_prior((int)n_trajs, *_priorTemperature);
            latent_traj_mean = latent_traj;
        }
        else {
            int64_t sizes = { input_size };
            at::Tensor frame = torch::from_blob(_inModel[0].get(), sizes);
            frame = torch::reshape(frame, { 1, 1, input_size });
#if DEBUG_PERFORM
            std::cout << "Current input size : " << frame.sizes() << std::endl;
#endif DEBUG_PERFORM
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

#if DEBUG_PERFORM
        std::cout << "latent traj shape" << latent_traj.sizes() << std::endl;
#endif

        // Latent modifications
        // apply scale and bias
        int64_t n_dimensions =
            std::min((int)latent_traj.size(1), (int)AVAILABLE_DIMS);
        for (int i = 0; i < n_dimensions; i++) {
            // The assert and casting here is needed as I got a:
            // warning: conversion to ‘std::array<std::atomic<float>*,
            // 8>::size_type’ {aka ‘long unsigned int’} from ‘int’ may change the
            // sign of the result [-Wsign-conversion]
            // Whatever AVAILABLE_DIMS type I defined
            assert(i >= 0);
            auto i2 = (long unsigned int)i;
            float scale = _latentScale->at(i2)->load();
            float bias = _latentBias->at(i2)->load();
            latent_traj.index_put_({ 0, i },
                (latent_traj.index({ 0, i }) * scale + bias));
            latent_traj_mean.index_put_(
                { 0, i }, (latent_traj_mean.index({ 0, i }) * scale + bias));
        }
        _rave->writeLatentBuffer(latent_traj_mean);

#if DEBUG_PERFORM
        std::cout << "scale & bias applied" << std::endl;
#endif

        // adding latent jitter on meaningful dimensions
        float jitter_amount = _latentJitterValue->load();
        latent_traj = latent_traj + jitter_amount * torch::randn_like(latent_traj);

#if DEBUG_PERFORM
        std::cout << "jitter applied" << std::endl;
#endif

        // filling missing dimensions with width parameter
        torch::Tensor latent_trajL = latent_traj,
            latent_trajR = latent_traj.clone();
        int missing_dims = _rave->getFullLatentDimensions() - latent_trajL.size(1);
        float width = _widthValue->load() / 100.f;
        at::Tensor latent_noiseL =
            torch::randn({ 1, missing_dims, latent_trajL.size(2) });
        at::Tensor latent_noiseR =
            (1 - width) * latent_noiseL +
            width * torch::randn({ 1, missing_dims, latent_trajL.size(2) });

#if DEBUG_PERFORM
        std::cout << "after width : " << latent_noiseL.sizes() << ";"
            << latent_trajL.sizes() << std::endl;
#endif

        latent_trajL = torch::cat({ latent_trajL, latent_noiseL }, 1);
        latent_trajR = torch::cat({ latent_trajR, latent_noiseR }, 1);

#if DEBUG_PERFORM
        std::cout << "latent processed" << std::endl;
#endif

        // Decode
        latent_traj = torch::cat({ latent_trajL, latent_trajR }, 0);
        at::Tensor out = _rave->decode(latent_traj);
        // On windows, I don't get why, but the two first dims are swapped (compared
        // to macOS / UNIX) with the same torch version
        if (out.sizes()[0] == 2) {
            out = out.transpose(0, 1);
        }
        at::Tensor outL = out.index({ 0, 0, at::indexing::Slice() });
        at::Tensor outR = out.index({ 0, 1, at::indexing::Slice() });

#if DEBUG_PERFORM
        std::cout << "latent decoded" << std::endl;
#endif

        float* outputDataPtrL, * outputDataPtrR;
        outputDataPtrL = outL.data_ptr<float>();
        outputDataPtrR = outR.data_ptr<float>();

        // Write in buffers
        assert(input_size >= 0);
        for (size_t i = 0; i < (size_t)input_size; i++) {
            _outModel[0][i] = outputDataPtrL[i];
            _outModel[1][i] = outputDataPtrR[i];
        }
        if (_smoothedFadeInOut.getCurrentValue() < EPSILON) {
            _isMuted.store(true);
        }
    }
}

void modelPerform_callback(RaveWwiseFX* ap) { ap->modelPerform(); }


void RaveWwiseFX::detectAvailableModels()
{
	// TODO
}

void RaveWwiseFX::mute()
{
    _fadeScheduler.store(muting::mute);
}

void RaveWwiseFX::unmute()
{
    _fadeScheduler.store(muting::unmute);
}

void RaveWwiseFX::updateBufferSizes()
{
    auto validBufferSizes = _rave->getValidBufferSizes();
    float a = validBufferSizes.start;
    float b = validBufferSizes.end;

    if (*_latencyMode < a) {
        std::cout << "too low; setting rate to : " << static_cast<int>(log2(a))
            << std::endl;
        *_latencyMode = static_cast<int>(log2(a));
    }
    else if (*_latencyMode > b) {
        std::cout << "too high; setting rate to : " << static_cast<int>(log2(b))
            << std::endl;
        *_latencyMode = static_cast<int>(log2(b));
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

    std::future<void> engineUpdateFuture = _engineThreadPool->submit(RAVEWwise::UpdateEngineJob, this, modelFile, 1);
}
