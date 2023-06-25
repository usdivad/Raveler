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

#include "RaveWwisePlugin.h"
#include "../SoundEnginePlugin/RaveWwiseFXFactory.h"

RaveWwisePlugin::RaveWwisePlugin()
{
}

RaveWwisePlugin::~RaveWwisePlugin()
{
}

bool RaveWwisePlugin::GetBankParameters(const GUID & in_guidPlatform, AK::Wwise::Plugin::DataWriter& in_dataWriter) const
{
    // Write bank data here
	// Needs to be written in same order as RaveWwiseFXParams::SetParamsBlock()

    in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "InputGain"));
	in_dataWriter.WriteUInt32(m_propertySet.GetUInt32(in_guidPlatform, "ChannelMode"));
	
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "InputThreshold"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "InputRatio"));

	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "LatentJitter"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "OutputWidth"));
	in_dataWriter.WriteBool(m_propertySet.GetBool(in_guidPlatform, "UsePrior"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "PriorTemperature"));

	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent1Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent1Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent2Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent2Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent3Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent3Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent4Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent4Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent5Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent5Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent6Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent6Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent7Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent7Scale"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent8Bias"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "Latent8Scale"));

	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "OutputGain"));
	in_dataWriter.WriteReal32(m_propertySet.GetReal32(in_guidPlatform, "OutputDryWet"));

	in_dataWriter.WriteUInt32(m_propertySet.GetUInt32(in_guidPlatform, "LatencyMode"));

	in_dataWriter.WriteString(m_propertySet.GetString(in_guidPlatform, "ModelFilePath"));

	in_dataWriter.WriteInt32(m_propertySet.GetInt32(in_guidPlatform, "LatencyCompensationSamples"));

    return true;
}

DEFINE_AUDIOPLUGIN_CONTAINER(RaveWwise);											// Create a PluginContainer structure that contains the info for our plugin
EXPORT_AUDIOPLUGIN_CONTAINER(RaveWwise);											// This is a DLL, we want to have a standardized name
ADD_AUDIOPLUGIN_CLASS_TO_CONTAINER(                                             // Add our CLI class to the PluginContainer
    RaveWwise,        // Name of the plug-in container for this shared library
    RaveWwisePlugin,  // Authoring plug-in class to add to the plug-in container
    RaveWwiseFX       // Corresponding Sound Engine plug-in class
);
DEFINE_PLUGIN_REGISTER_HOOK

DEFINEDUMMYASSERTHOOK;							// Placeholder assert hook for Wwise plug-ins using AKASSERT (cassert used by default)
