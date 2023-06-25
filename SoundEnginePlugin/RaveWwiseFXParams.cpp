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

#include "RaveWwiseFXParams.h"

#include <AK/Tools/Common/AkBankReadHelpers.h>

RaveWwiseFXParams::RaveWwiseFXParams()
{
}

RaveWwiseFXParams::~RaveWwiseFXParams()
{
}

RaveWwiseFXParams::RaveWwiseFXParams(const RaveWwiseFXParams& in_rParams)
{
    RTPC = in_rParams.RTPC;
    NonRTPC = in_rParams.NonRTPC;
    m_paramChangeHandler.SetAllParamChanges();
}

AK::IAkPluginParam* RaveWwiseFXParams::Clone(AK::IAkPluginMemAlloc* in_pAllocator)
{
    return AK_PLUGIN_NEW(in_pAllocator, RaveWwiseFXParams(*this));
}

AKRESULT RaveWwiseFXParams::Init(AK::IAkPluginMemAlloc* in_pAllocator, const void* in_pParamsBlock, AkUInt32 in_ulBlockSize)
{
    if (in_ulBlockSize == 0)
    {
        // Initialize default parameters here

		RTPC.fInputGain = 0.f;
		RTPC.uChannelMode = static_cast<AkUInt32>(EChannelMode::Left);

		RTPC.fInputThreshold = 0.f;
		RTPC.fInputRatio = 1.f;

		RTPC.fLatentJitter = 0.f;
		RTPC.fOutputWidth = 100.f;
		RTPC.bUsePrior = false; // Originally true in RAVE VST, but results in choppy performance on Windows
		RTPC.fPriorTemperature = 1.f;

		RTPC.fLatent1Bias = 0.f;
		RTPC.fLatent1Scale = 1.f;
		RTPC.fLatent2Bias = 0.f;
		RTPC.fLatent2Scale = 1.f;
		RTPC.fLatent3Bias = 0.f;
		RTPC.fLatent3Scale = 1.f;
		RTPC.fLatent4Bias = 0.f;
		RTPC.fLatent4Scale = 1.f;
		RTPC.fLatent5Bias = 0.f;
		RTPC.fLatent5Scale = 1.f;
		RTPC.fLatent6Bias = 0.f;
		RTPC.fLatent6Scale = 1.f;
		RTPC.fLatent7Bias = 0.f;
		RTPC.fLatent7Scale = 1.f;
		RTPC.fLatent8Bias = 0.f;
		RTPC.fLatent8Scale = 1.f;

		RTPC.fOutputGain = 0.f;
		RTPC.fOutputDryWet = 100.f;

		NonRTPC.uLatencyMode = 13;

		NonRTPC.sModelFilePath = nullptr;

        m_paramChangeHandler.SetAllParamChanges();

        return AK_Success;
    }

    return SetParamsBlock(in_pParamsBlock, in_ulBlockSize);
}

AKRESULT RaveWwiseFXParams::Term(AK::IAkPluginMemAlloc* in_pAllocator)
{
    AK_PLUGIN_DELETE(in_pAllocator, this);
    return AK_Success;
}

AKRESULT RaveWwiseFXParams::SetParamsBlock(const void* in_pParamsBlock, AkUInt32 in_ulBlockSize)
{
    AKRESULT eResult = AK_Success;
    AkUInt8* pParamsBlock = (AkUInt8*)in_pParamsBlock;

    // Read bank data here
	// Needs to be read in same order as RaveWwisePlugin::GetBankParameters()

    RTPC.fInputGain = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.uChannelMode = READBANKDATA(AkUInt32, pParamsBlock, in_ulBlockSize);
	
	RTPC.fInputThreshold = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fInputRatio = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);

	RTPC.fLatentJitter = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fOutputWidth = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.bUsePrior = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize) != 0;
	RTPC.fPriorTemperature = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);

	RTPC.fLatent1Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent1Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent2Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent2Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent3Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent3Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent4Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent4Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent5Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent5Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent6Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent6Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent7Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent7Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent8Bias = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fLatent8Scale = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);

	RTPC.fOutputGain = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);
	RTPC.fOutputDryWet = READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize);

	NonRTPC.uLatencyMode = READBANKDATA(AkUInt32, pParamsBlock, in_ulBlockSize);

	NonRTPC.sModelFilePath = READBANKDATA(AkOSChar*, pParamsBlock, in_ulBlockSize);

	RTPC.iLatencyCompensationSamples = (AkInt32)(READBANKDATA(AkReal32, pParamsBlock, in_ulBlockSize));

    CHECKBANKDATASIZE(in_ulBlockSize, eResult);
    m_paramChangeHandler.SetAllParamChanges();

    return eResult;
}

AKRESULT RaveWwiseFXParams::SetParam(AkPluginParamID in_paramID, const void* in_pValue, AkUInt32 in_ulParamSize)
{
    AKRESULT eResult = AK_Success;

    // Handle parameter change here
	// TODO: Replace C-style casts with C++-style casts
    switch (in_paramID)
    {

    case PARAM_INPUT_GAIN_ID:
        RTPC.fInputGain = *((AkReal32*)in_pValue);
        m_paramChangeHandler.SetParamChange(PARAM_INPUT_GAIN_ID);
        break;
	case PARAM_CHANNEL_MODE_ID:
		RTPC.uChannelMode = *((AkUInt32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_CHANNEL_MODE_ID);
		break;

	case PARAM_LATENT_JITTER_ID:
		RTPC.fLatentJitter = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_JITTER_ID);
		break;
	case PARAM_OUTPUT_WIDTH_ID:
		RTPC.fOutputWidth = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_OUTPUT_WIDTH_ID);
		break;
	case PARAM_USE_PRIOR_ID:
		RTPC.bUsePrior = (*((AkReal32*)in_pValue) != 0);
		m_paramChangeHandler.SetParamChange(PARAM_USE_PRIOR_ID);
		break;
	case PARAM_PRIOR_TEMPERATURE_ID:
		RTPC.fPriorTemperature = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_PRIOR_TEMPERATURE_ID);
		break;

	case PARAM_LATENT_1_BIAS_ID:
		RTPC.fLatent1Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_1_BIAS_ID);
		break;
	case PARAM_LATENT_1_SCALE_ID:
		RTPC.fLatent1Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_1_SCALE_ID);
		break;

	case PARAM_LATENT_2_BIAS_ID:
		RTPC.fLatent2Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_2_BIAS_ID);
		break;
	case PARAM_LATENT_2_SCALE_ID:
		RTPC.fLatent2Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_2_SCALE_ID);
		break;

	case PARAM_LATENT_3_BIAS_ID:
		RTPC.fLatent3Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_3_BIAS_ID);
		break;
	case PARAM_LATENT_3_SCALE_ID:
		RTPC.fLatent3Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_3_SCALE_ID);
		break;

	case PARAM_LATENT_4_BIAS_ID:
		RTPC.fLatent4Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_4_BIAS_ID);
		break;
	case PARAM_LATENT_4_SCALE_ID:
		RTPC.fLatent4Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_4_SCALE_ID);
		break;

	case PARAM_LATENT_5_BIAS_ID:
		RTPC.fLatent5Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_5_BIAS_ID);
		break;
	case PARAM_LATENT_5_SCALE_ID:
		RTPC.fLatent5Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_5_SCALE_ID);
		break;

	case PARAM_LATENT_6_BIAS_ID:
		RTPC.fLatent6Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_6_BIAS_ID);
		break;
	case PARAM_LATENT_6_SCALE_ID:
		RTPC.fLatent6Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_6_SCALE_ID);
		break;

	case PARAM_LATENT_7_BIAS_ID:
		RTPC.fLatent7Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_7_BIAS_ID);
		break;
	case PARAM_LATENT_7_SCALE_ID:
		RTPC.fLatent7Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_7_SCALE_ID);
		break;

	case PARAM_LATENT_8_BIAS_ID:
		RTPC.fLatent8Bias = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_8_BIAS_ID);
		break;
	case PARAM_LATENT_8_SCALE_ID:
		RTPC.fLatent8Scale = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENT_8_SCALE_ID);
		break;

	case PARAM_OUTPUT_GAIN_ID:
		RTPC.fOutputGain = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_OUTPUT_GAIN_ID);
		break;
	case PARAM_OUTPUT_DRY_WET_ID:
		RTPC.fOutputDryWet = *((AkReal32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_OUTPUT_DRY_WET_ID);
		break;
	case PARAM_OUTPUT_LIMIT_ID:
		RTPC.bOutputLimit = *((bool*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_OUTPUT_LIMIT_ID);
		break;

	case PARAM_LATENCY_MODE_ID:
		NonRTPC.uLatencyMode = *((AkUInt32*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_LATENCY_MODE_ID);
		break;

	case PARAM_MODEL_FILE_PATH_ID:
		NonRTPC.sModelFilePath = ((AkOSChar*)in_pValue);
		m_paramChangeHandler.SetParamChange(PARAM_MODEL_FILE_PATH_ID);
		break;

	case PARAM_LATENCY_COMPENSATION_ID:
		RTPC.iLatencyCompensationSamples = (AkInt32)(*((AkReal32*)in_pValue));
		m_paramChangeHandler.SetParamChange(PARAM_LATENCY_COMPENSATION_ID);
		break;

    default:
        eResult = AK_InvalidParameter;
        break;
    }

    return eResult;
}
