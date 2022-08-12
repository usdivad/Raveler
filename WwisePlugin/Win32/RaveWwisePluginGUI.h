#pragma once

#include "../RaveWwisePlugin.h"

class RaveWwisePluginGUI final
	: public AK::Wwise::Plugin::PluginMFCWindows<>
	, public AK::Wwise::Plugin::GUIWindows
{
public:
	RaveWwisePluginGUI();

};
