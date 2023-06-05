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

#include <math.h>

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
    
    // TODO
    //_engineThreadPool = std::make_unique<juce::ThreadPool>(1);
    
    // TODO
    //_rave.reset(new RAVE());

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
