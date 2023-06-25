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

#ifndef RaveWwiseFXParams_H
#define RaveWwiseFXParams_H

#include <string>

#include <AK/SoundEngine/Common/IAkPlugin.h>
#include <AK/Plugin/PluginServices/AkFXParameterChangeHandler.h>

// ----------------------------------------------------------------

// Add parameters IDs here, those IDs should map to the AudioEnginePropertyID
// attributes in the xml property definition.

// Input
static const AkPluginParamID PARAM_INPUT_GAIN_ID = 0;
static const AkPluginParamID PARAM_CHANNEL_MODE_ID = 1;

// Compressor
static const AkPluginParamID PARAM_INPUT_THRESHOLD_ID = 2;
static const AkPluginParamID PARAM_INPUT_RATIO_ID = 3;

// Model
static const AkPluginParamID PARAM_LATENT_JITTER_ID = 4;
static const AkPluginParamID PARAM_OUTPUT_WIDTH_ID = 5;
static const AkPluginParamID PARAM_USE_PRIOR_ID = 6;
static const AkPluginParamID PARAM_PRIOR_TEMPERATURE_ID = 7;

// Latent
static const AkPluginParamID PARAM_LATENT_1_BIAS_ID = 8;
static const AkPluginParamID PARAM_LATENT_1_SCALE_ID = 9;
static const AkPluginParamID PARAM_LATENT_2_BIAS_ID = 10;
static const AkPluginParamID PARAM_LATENT_2_SCALE_ID = 11;
static const AkPluginParamID PARAM_LATENT_3_BIAS_ID = 12;
static const AkPluginParamID PARAM_LATENT_3_SCALE_ID = 13;
static const AkPluginParamID PARAM_LATENT_4_BIAS_ID = 14;
static const AkPluginParamID PARAM_LATENT_4_SCALE_ID = 15;
static const AkPluginParamID PARAM_LATENT_5_BIAS_ID = 16;
static const AkPluginParamID PARAM_LATENT_5_SCALE_ID = 17;
static const AkPluginParamID PARAM_LATENT_6_BIAS_ID = 18;
static const AkPluginParamID PARAM_LATENT_6_SCALE_ID = 19;
static const AkPluginParamID PARAM_LATENT_7_BIAS_ID = 20;
static const AkPluginParamID PARAM_LATENT_7_SCALE_ID = 21;
static const AkPluginParamID PARAM_LATENT_8_BIAS_ID = 22;
static const AkPluginParamID PARAM_LATENT_8_SCALE_ID = 23;

// Output
static const AkPluginParamID PARAM_OUTPUT_GAIN_ID = 24;
static const AkPluginParamID PARAM_OUTPUT_DRY_WET_ID = 25;
static const AkPluginParamID PARAM_OUTPUT_LIMIT_ID = 26;

// Buffer
static const AkPluginParamID PARAM_LATENCY_MODE_ID = 27;

// Custom
static const AkPluginParamID PARAM_MODEL_FILE_PATH_ID = 28;
static const AkPluginParamID PARAM_LATENCY_COMPENSATION_ID = 29;

// Num params
static const AkUInt32 NUM_PARAMS = 30;

// ----------------------------------------------------------------

// Channel mode
enum class EChannelMode : AkUInt32
{
	Left = 0,
	Right,
	LeftAndRight
};

// ----------------------------------------------------------------

// TODO: Move some of these to non-RTPC params struct
struct RaveWwiseRTPCParams
{
    // Input
    AkReal32 fInputGain;
    AkUInt32 uChannelMode;

    // Compressor
    AkReal32 fInputThreshold;
    AkReal32 fInputRatio;

    // Model
    AkReal32 fLatentJitter;
    AkReal32 fOutputWidth;
    bool bUsePrior;
    AkReal32 fPriorTemperature;

    // Latent
    AkReal32 fLatent1Bias;
    AkReal32 fLatent1Scale;

	AkReal32 fLatent2Bias;
	AkReal32 fLatent2Scale;
	
    AkReal32 fLatent3Bias;
	AkReal32 fLatent3Scale;
	
    AkReal32 fLatent4Bias;
	AkReal32 fLatent4Scale;
	
    AkReal32 fLatent5Bias;
	AkReal32 fLatent5Scale;
	
    AkReal32 fLatent6Bias;
	AkReal32 fLatent6Scale;
	
	AkReal32 fLatent7Bias;
    AkReal32 fLatent7Scale;
	
    AkReal32 fLatent8Bias;
	AkReal32 fLatent8Scale;

    // Output
    AkReal32 fOutputGain;
    AkReal32 fOutputDryWet;
    bool bOutputLimit;

    // Custom
    AkInt32 iLatencyCompensationSamples;
};

struct RaveWwiseNonRTPCParams
{
	// Buffer
	AkUInt32 uLatencyMode;

	// Custom
	AkOSChar* sModelFilePath;
};

struct RaveWwiseFXParams
    : public AK::IAkPluginParam
{
    RaveWwiseFXParams();
    RaveWwiseFXParams(const RaveWwiseFXParams& in_rParams);

    ~RaveWwiseFXParams();

    /// Create a duplicate of the parameter node instance in its current state.
    IAkPluginParam* Clone(AK::IAkPluginMemAlloc* in_pAllocator) override;

    /// Initialize the plug-in parameter node interface.
    /// Initializes the internal parameter structure to default values or with the provided parameter block if it is valid.
    AKRESULT Init(AK::IAkPluginMemAlloc* in_pAllocator, const void* in_pParamsBlock, AkUInt32 in_ulBlockSize) override;

    /// Called by the sound engine when a parameter node is terminated.
    AKRESULT Term(AK::IAkPluginMemAlloc* in_pAllocator) override;

    /// Set all plug-in parameters at once using a parameter block.
    AKRESULT SetParamsBlock(const void* in_pParamsBlock, AkUInt32 in_ulBlockSize) override;

    /// Update a single parameter at a time and perform the necessary actions on the parameter changes.
    AKRESULT SetParam(AkPluginParamID in_paramID, const void* in_pValue, AkUInt32 in_ulParamSize) override;

    AK::AkFXParameterChangeHandler<NUM_PARAMS> m_paramChangeHandler;

    RaveWwiseRTPCParams RTPC;
    RaveWwiseNonRTPCParams NonRTPC;
};

#endif // RaveWwiseFXParams_H
