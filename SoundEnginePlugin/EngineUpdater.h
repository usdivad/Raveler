#pragma once

#include <string>

class RaveWwiseFX;

//----------------------------------------------------------------------------------------------------------------------
// RaveWwise::UpdateEngineJob
//----------------------------------------------------------------------------------------------------------------------

/// Method that handles internal engine (RAVE instance) updating.
/// Ported from RAVE VST's UpdateEngineJob -- see https://github.com/acids-ircam/rave_vst/blob/main/source/EngineUpdater.h
/// Note that UpdateEngineJob now exists as a method and not a class.

namespace RaveWwise
{
    /// The job that gets submitted to the thread pool -- loads a model and updates the FX processor accordingly
    void UpdateEngineJob(RaveWwiseFX* processor, const std::string& modelFile, size_t fadeOutWaitTimeMs = 0);
}
