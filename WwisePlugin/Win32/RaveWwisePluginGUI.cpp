
#include "RaveWwisePluginGUI.h"

RaveWwisePluginGUI::RaveWwisePluginGUI()
{
}

ADD_AUDIOPLUGIN_CLASS_TO_CONTAINER(
    RaveWwise,            // Name of the plug-in container for this shared library
    RaveWwisePluginGUI,   // Authoring plug-in class to add to the plug-in container
    RaveWwiseFX           // Corresponding Sound Engine plug-in class
);
