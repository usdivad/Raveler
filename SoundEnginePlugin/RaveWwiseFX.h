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

#ifndef RaveWwiseFX_H
#define RaveWwiseFX_H

#include "RaveWwiseFXParams.h"

#include "CircularBuffer.h"
#include "EngineUpdater.h"
#include "Rave.h"

#include <algorithm>
#include <cassert>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <torch/script.h>
#include <torch/torch.h>
#include <BS_thread_pool.hpp>

//----------------------------------------------------------------------------------------------------------------------
// TODO: Move these to their own files

namespace RAVEWwise
{
    // Replacement for juce::NormalisableRange.

    //==============================================================================
    /**
        Represents a mapping between an arbitrary range of values and a
        normalised 0->1 range.

        The properties of the mapping also include an optional snapping interval
        and skew-factor.

        @see Range

        @tags{Core}
    */
    template <typename ValueType>
    class NormalisableRange
    {
    public:
        /** Creates a continuous range that performs a dummy mapping. */
        NormalisableRange() = default;

        NormalisableRange(const NormalisableRange&) = default;
        NormalisableRange& operator= (const NormalisableRange&) = default;
        NormalisableRange(NormalisableRange&&) = default;
        NormalisableRange& operator= (NormalisableRange&&) = default;

        /** Creates a NormalisableRange with a given range, interval and skew factor. */
        NormalisableRange(ValueType rangeStart,
            ValueType rangeEnd,
            ValueType intervalValue,
            ValueType skewFactor,
            bool useSymmetricSkew = false) noexcept
            : start(rangeStart), end(rangeEnd), interval(intervalValue),
            skew(skewFactor), symmetricSkew(useSymmetricSkew)
        {
            checkInvariants();
        }

        /** Creates a NormalisableRange with a given range, continuous interval, but a dummy skew-factor. */
        NormalisableRange(ValueType rangeStart,
            ValueType rangeEnd) noexcept
            : start(rangeStart), end(rangeEnd)
        {
            checkInvariants();
        }

        /** Creates a NormalisableRange with a given range and interval, but a dummy skew-factor. */
        NormalisableRange(ValueType rangeStart,
            ValueType rangeEnd,
            ValueType intervalValue) noexcept
            : start(rangeStart), end(rangeEnd), interval(intervalValue)
        {
            checkInvariants();
        }

        /** Creates a NormalisableRange with a given range, continuous interval, but a dummy skew-factor. */
        NormalisableRange(Range<ValueType> range) noexcept
            : NormalisableRange(range.getStart(), range.getEnd())
        {
        }

        /** Creates a NormalisableRange with a given range and interval, but a dummy skew-factor. */
        NormalisableRange(Range<ValueType> range, ValueType intervalValue) noexcept
            : NormalisableRange(range.getStart(), range.getEnd(), intervalValue)
        {
        }

        /** A function object which can remap a value in some way based on the start and end of a range. */
        using ValueRemapFunction = std::function<ValueType(ValueType rangeStart,
            ValueType rangeEnd,
            ValueType valueToRemap)>;

        /** Creates a NormalisableRange with a given range and an injective mapping function.

            @param rangeStart           The minimum value in the range.
            @param rangeEnd             The maximum value in the range.
            @param convertFrom0To1Func  A function which uses the current start and end of this NormalisableRange
                                        and produces a mapped value from a normalised value.
            @param convertTo0To1Func    A function which uses the current start and end of this NormalisableRange
                                        and produces a normalised value from a mapped value.
            @param snapToLegalValueFunc A function which uses the current start and end of this NormalisableRange
                                        to take a mapped value and snap it to the nearest legal value.
        */
        NormalisableRange(ValueType rangeStart,
            ValueType rangeEnd,
            ValueRemapFunction convertFrom0To1Func,
            ValueRemapFunction convertTo0To1Func,
            ValueRemapFunction snapToLegalValueFunc = {}) noexcept
            : start(rangeStart),
            end(rangeEnd),
            convertFrom0To1Function(std::move(convertFrom0To1Func)),
            convertTo0To1Function(std::move(convertTo0To1Func)),
            snapToLegalValueFunction(std::move(snapToLegalValueFunc))
        {
            checkInvariants();
        }

        /** Uses the properties of this mapping to convert a non-normalised value to
            its 0->1 representation.
        */
        ValueType convertTo0to1(ValueType v) const noexcept
        {
            if (convertTo0To1Function != nullptr)
                return clampTo0To1(convertTo0To1Function(start, end, v));

            auto proportion = clampTo0To1((v - start) / (end - start));

            if (skew == static_cast<ValueType> (1))
                return proportion;

            if (!symmetricSkew)
                return std::pow(proportion, skew);

            auto distanceFromMiddle = static_cast<ValueType> (2) * proportion - static_cast<ValueType> (1);

            return (static_cast<ValueType> (1) + std::pow(std::abs(distanceFromMiddle), skew)
                * (distanceFromMiddle < ValueType() ? static_cast<ValueType> (-1)
                    : static_cast<ValueType> (1)))
                / static_cast<ValueType> (2);
        }

        /** Uses the properties of this mapping to convert a normalised 0->1 value to
            its full-range representation.
        */
        ValueType convertFrom0to1(ValueType proportion) const noexcept
        {
            proportion = clampTo0To1(proportion);

            if (convertFrom0To1Function != nullptr)
                return convertFrom0To1Function(start, end, proportion);

            if (!symmetricSkew)
            {
                if (skew != static_cast<ValueType> (1) && proportion > ValueType())
                    proportion = std::exp(std::log(proportion) / skew);

                return start + (end - start) * proportion;
            }

            auto distanceFromMiddle = static_cast<ValueType> (2) * proportion - static_cast<ValueType> (1);

            if (skew != static_cast<ValueType> (1) && distanceFromMiddle != static_cast<ValueType> (0))
                distanceFromMiddle = std::exp(std::log(std::abs(distanceFromMiddle)) / skew)
                * (distanceFromMiddle < ValueType() ? static_cast<ValueType> (-1)
                    : static_cast<ValueType> (1));

            return start + (end - start) / static_cast<ValueType> (2) * (static_cast<ValueType> (1) + distanceFromMiddle);
        }

        /** Takes a non-normalised value and snaps it based on either the interval property of
            this NormalisableRange or the lambda function supplied to the constructor.
        */
        ValueType snapToLegalValue(ValueType v) const noexcept
        {
            if (snapToLegalValueFunction != nullptr)
                return snapToLegalValueFunction(start, end, v);

            if (interval > ValueType())
                v = start + interval * std::floor((v - start) / interval + static_cast<ValueType> (0.5));

            return (v <= start || end <= start) ? start : (v >= end ? end : v);
        }

        /** Returns the extent of the normalisable range. */
        Range<ValueType> getRange() const noexcept { return { start, end }; }

        /** Given a value which is between the start and end points, this sets the skew
            such that convertFrom0to1 (0.5) will return this value.

            If you have used lambda functions for convertFrom0to1Func and convertFrom0to1Func in the
            constructor of this class then the skew value is ignored.

            @param centrePointValue  this must be greater than the start of the range and less than the end.
        */
        void setSkewForCentre(ValueType centrePointValue) noexcept
        {
            assert(centrePointValue > start);
            assert(centrePointValue < end);

            symmetricSkew = false;
            skew = std::log(static_cast<ValueType> (0.5)) / std::log((centrePointValue - start) / (end - start));
            checkInvariants();
        }

        /** The minimum value of the non-normalised range. */
        ValueType start = 0;

        /** The maximum value of the non-normalised range. */
        ValueType end = 1;

        /** The snapping interval that should be used (for a non-normalised value). Use 0 for a
            continuous range.

            If you have used a lambda function for snapToLegalValueFunction in the constructor of
            this class then the interval is ignored.
        */
        ValueType interval = 0;

        /** An optional skew factor that alters the way values are distribute across the range.

            The skew factor lets you skew the mapping logarithmically so that larger or smaller
            values are given a larger proportion of the available space.

            A factor of 1.0 has no skewing effect at all. If the factor is < 1.0, the lower end
            of the range will fill more of the slider's length; if the factor is > 1.0, the upper
            end of the range will be expanded.

            If you have used lambda functions for convertFrom0to1Func and convertFrom0to1Func in the
            constructor of this class then the skew value is ignored.
        */
        ValueType skew = 1;

        /** If true, the skew factor applies from the middle of the slider to each of its ends. */
        bool symmetricSkew = false;

    private:
        void checkInvariants() const
        {
            assert(end > start);
            assert(interval >= ValueType());
            assert(skew > ValueType());
        }

        static ValueType clampTo0To1(ValueType value)
        {
            auto clampedValue = jlimit(static_cast<ValueType> (0), static_cast<ValueType> (1), value);

            // If you hit this assertion then either your normalisation function is not working
            // correctly or your input is out of the expected bounds.
            assert(clampedValue == value);

            return clampedValue;
        }

        ValueRemapFunction convertFrom0To1Function, convertTo0To1Function, snapToLegalValueFunction;
    };

} // namespace RAVEWwise

//----------------------------------------------------------------------------------------------------------------------

namespace RAVEWwise
{
	// Replacement for juce::SmoothedValue.

    //==============================================================================
    /**
        A base class for the smoothed value classes.

        This class is used to provide common functionality to the SmoothedValue and
        dsp::LogRampedValue classes.

        @tags{Audio}
    */
    template <typename SmoothedValueType>
    class SmoothedValueBase
    {
    private:
        //==============================================================================
        template <typename T> struct FloatTypeHelper;

        template <template <typename> class SmoothedValueClass, typename FloatType>
        struct FloatTypeHelper <SmoothedValueClass <FloatType>>
        {
            using Type = FloatType;
        };

        template <template <typename, typename> class SmoothedValueClass, typename FloatType, typename SmoothingType>
        struct FloatTypeHelper <SmoothedValueClass <FloatType, SmoothingType>>
        {
            using Type = FloatType;
        };

    public:
        using FloatType = typename FloatTypeHelper<SmoothedValueType>::Type;

        //==============================================================================
        /** Constructor. */
        SmoothedValueBase() = default;

        //==============================================================================
        /** Returns true if the current value is currently being interpolated. */
        bool isSmoothing() const noexcept { return countdown > 0; }

        /** Returns the current value of the ramp. */
        FloatType getCurrentValue() const noexcept { return currentValue; }

        //==============================================================================
        /** Returns the target value towards which the smoothed value is currently moving. */
        FloatType getTargetValue() const noexcept { return target; }

        /** Sets the current value and the target value.
            @param newValue    the new value to take
        */
        void setCurrentAndTargetValue(FloatType newValue)
        {
            target = currentValue = newValue;
            countdown = 0;
        }

        //==============================================================================
        /** Applies a smoothed gain to a stream of samples
            S[i] *= gain
            @param samples Pointer to a raw array of samples
            @param numSamples Length of array of samples
        */
        void applyGain(FloatType* samples, int numSamples) noexcept
        {
            assert(numSamples >= 0);

            if (isSmoothing())
            {
                for (int i = 0; i < numSamples; ++i)
                    samples[i] *= getNextSmoothedValue();
            }
            else
            {
                FloatVectorOperations::multiply(samples, target, numSamples);
            }
        }

        /** Computes output as a smoothed gain applied to a stream of samples.
            Sout[i] = Sin[i] * gain
            @param samplesOut A pointer to a raw array of output samples
            @param samplesIn  A pointer to a raw array of input samples
            @param numSamples The length of the array of samples
        */
        void applyGain(FloatType* samplesOut, const FloatType* samplesIn, int numSamples) noexcept
        {
            assert(numSamples >= 0);

            if (isSmoothing())
            {
                for (int i = 0; i < numSamples; ++i)
                    samplesOut[i] = samplesIn[i] * getNextSmoothedValue();
            }
            else
            {
                FloatVectorOperations::multiply(samplesOut, samplesIn, target, numSamples);
            }
        }

        /** Applies a smoothed gain to a buffer */
        //void applyGain(AudioBuffer<FloatType>& buffer, int numSamples) noexcept
        //{
        //    assert(numSamples >= 0);

        //    if (isSmoothing())
        //    {
        //        if (buffer.getNumChannels() == 1)
        //        {
        //            auto* samples = buffer.getWritePointer(0);

        //            for (int i = 0; i < numSamples; ++i)
        //                samples[i] *= getNextSmoothedValue();
        //        }
        //        else
        //        {
        //            for (auto i = 0; i < numSamples; ++i)
        //            {
        //                auto gain = getNextSmoothedValue();

        //                for (int channel = 0; channel < buffer.getNumChannels(); channel++)
        //                    buffer.setSample(channel, i, buffer.getSample(channel, i) * gain);
        //            }
        //        }
        //    }
        //    else
        //    {
        //        buffer.applyGain(0, numSamples, target);
        //    }
        //}

    private:
        //==============================================================================
        FloatType getNextSmoothedValue() noexcept
        {
            return static_cast <SmoothedValueType*> (this)->getNextValue();
        }

    protected:
        //==============================================================================
        FloatType currentValue = 0;
        FloatType target = currentValue;
        int countdown = 0;
    };

    //==============================================================================
    /**
        A namespace containing a set of types used for specifying the smoothing
        behaviour of the SmoothedValue class.

        For example:
        @code
        SmoothedValue<float, ValueSmoothingTypes::Multiplicative> frequency (1.0f);
        @endcode
    */
    namespace ValueSmoothingTypes
    {
        /**
            Used to indicate a linear smoothing between values.

            @tags{Audio}
        */
        struct Linear {};

        /**
            Used to indicate a smoothing between multiplicative values.

            @tags{Audio}
        */
        struct Multiplicative {};
    }

    //==============================================================================
    /**
        A utility class for values that need smoothing to avoid audio glitches.

        A ValueSmoothingTypes::Linear template parameter selects linear smoothing,
        which increments the SmoothedValue linearly towards its target value.

        @code
        SmoothedValue<float, ValueSmoothingTypes::Linear> yourSmoothedValue;
        @endcode

        A ValueSmoothingTypes::Multiplicative template parameter selects
        multiplicative smoothing increments towards the target value.

        @code
        SmoothedValue<float, ValueSmoothingTypes::Multiplicative> yourSmoothedValue;
        @endcode

        Multiplicative smoothing is useful when you are dealing with
        exponential/logarithmic values like volume in dB or frequency in Hz. For
        example a 12 step ramp from 440.0 Hz (A4) to 880.0 Hz (A5) will increase the
        frequency with an equal temperament tuning across the octave. A 10 step
        smoothing from 1.0 (0 dB) to 3.16228 (10 dB) will increase the value in
        increments of 1 dB.

        Note that when you are using multiplicative smoothing you cannot ever reach a
        target value of zero!

        @tags{Audio}
    */
    template <typename FloatType, typename SmoothingType = ValueSmoothingTypes::Linear>
    class SmoothedValue : public SmoothedValueBase <SmoothedValue <FloatType, SmoothingType>>
    {
    public:
        //==============================================================================
        /** Constructor. */
        SmoothedValue() noexcept
            : SmoothedValue((FloatType)(std::is_same<SmoothingType, ValueSmoothingTypes::Linear>::value ? 0 : 1))
        {
        }

        /** Constructor. */
        SmoothedValue(FloatType initialValue) noexcept
        {
            // Multiplicative smoothed values cannot ever reach 0!
            assert(!(std::is_same<SmoothingType, ValueSmoothingTypes::Multiplicative>::value && initialValue == 0));

            // Visual Studio can't handle base class initialisation with CRTP
            this->currentValue = initialValue;
            this->target = this->currentValue;
        }

        //==============================================================================
        /** Reset to a new sample rate and ramp length.
            @param sampleRate           The sample rate
            @param rampLengthInSeconds  The duration of the ramp in seconds
        */
        void reset(double sampleRate, double rampLengthInSeconds) noexcept
        {
            assert(sampleRate > 0 && rampLengthInSeconds >= 0);
            reset((int)std::floor(rampLengthInSeconds * sampleRate));
        }

        /** Set a new ramp length directly in samples.
            @param numSteps     The number of samples over which the ramp should be active
        */
        void reset(int numSteps) noexcept
        {
            stepsToTarget = numSteps;
            this->setCurrentAndTargetValue(this->target);
        }

        //==============================================================================
        /** Set the next value to ramp towards.
            @param newValue     The new target value
        */
        void setTargetValue(FloatType newValue) noexcept
        {
            if (newValue == this->target)
                return;

            if (stepsToTarget <= 0)
            {
                this->setCurrentAndTargetValue(newValue);
                return;
            }

            // Multiplicative smoothed values cannot ever reach 0!
            assert(!(std::is_same<SmoothingType, ValueSmoothingTypes::Multiplicative>::value && newValue == 0));

            this->target = newValue;
            this->countdown = stepsToTarget;

            setStepSize();
        }

        //==============================================================================
        /** Compute the next value.
            @returns Smoothed value
        */
        FloatType getNextValue() noexcept
        {
            if (!this->isSmoothing())
                return this->target;

            --(this->countdown);

            if (this->isSmoothing())
                setNextValue();
            else
                this->currentValue = this->target;

            return this->currentValue;
        }

        //==============================================================================
        /** Skip the next numSamples samples.
            This is identical to calling getNextValue numSamples times. It returns
            the new current value.
            @see getNextValue
        */
        FloatType skip(int numSamples) noexcept
        {
            if (numSamples >= this->countdown)
            {
                this->setCurrentAndTargetValue(this->target);
                return this->target;
            }

            skipCurrentValue(numSamples);

            this->countdown -= numSamples;
            return this->currentValue;
        }

        //==============================================================================
#ifndef DOXYGEN
 /** Using the new methods:

     lsv.setValue (x, false); -> lsv.setTargetValue (x);
     lsv.setValue (x, true);  -> lsv.setCurrentAndTargetValue (x);

     @param newValue     The new target value
     @param force        If true, the value will be set immediately, bypassing the ramp
 */
        [[deprecated("Use setTargetValue and setCurrentAndTargetValue instead.")]]
        void setValue(FloatType newValue, bool force = false) noexcept
        {
            if (force)
            {
                this->setCurrentAndTargetValue(newValue);
                return;
            }

            setTargetValue(newValue);
        }
#endif

    private:
        //==============================================================================
        template <typename T>
        using LinearVoid = typename std::enable_if <std::is_same <T, ValueSmoothingTypes::Linear>::value, void>::type;

        template <typename T>
        using MultiplicativeVoid = typename std::enable_if <std::is_same <T, ValueSmoothingTypes::Multiplicative>::value, void>::type;

        //==============================================================================
        template <typename T = SmoothingType>
        LinearVoid<T> setStepSize() noexcept
        {
            step = (this->target - this->currentValue) / (FloatType)this->countdown;
        }

        template <typename T = SmoothingType>
        MultiplicativeVoid<T> setStepSize()
        {
            step = std::exp((std::log(std::abs(this->target)) - std::log(std::abs(this->currentValue))) / (FloatType)this->countdown);
        }

        //==============================================================================
        template <typename T = SmoothingType>
        LinearVoid<T> setNextValue() noexcept
        {
            this->currentValue += step;
        }

        template <typename T = SmoothingType>
        MultiplicativeVoid<T> setNextValue() noexcept
        {
            this->currentValue *= step;
        }

        //==============================================================================
        template <typename T = SmoothingType>
        LinearVoid<T> skipCurrentValue(int numSamples) noexcept
        {
            this->currentValue += step * (FloatType)numSamples;
        }

        template <typename T = SmoothingType>
        MultiplicativeVoid<T> skipCurrentValue(int numSamples)
        {
            this->currentValue *= (FloatType)std::pow(step, numSamples);
        }

        //==============================================================================
        FloatType step = FloatType();
        int stepsToTarget = 0;
    };

    template <typename FloatType>
    using LinearSmoothedValue = SmoothedValue <FloatType, ValueSmoothingTypes::Linear>;

} // namespace RAVEWwise

//----------------------------------------------------------------------------------------------------------------------

#define EPSILON 0.0000001
#define DEBUG 0

const size_t AVAILABLE_DIMS = 8;
const std::vector<std::string> channel_modes = { "L", "R", "L + R" };

namespace rave_parameters {
	const std::string model_selection{ "model_selection" };
	const std::string input_gain{ "input_gain" };
	const std::string channel_mode{ "channel_mode" };
	const std::string input_thresh{ "input_threshold" };
	const std::string input_ratio{ "input_ratio" };
	const std::string latent_jitter{ "latent_jitter" };
	const std::string output_width{ "output_width" };
	const std::string output_gain{ "output_gain" };
	const std::string output_limit{ "output_limit" };
	const std::string output_drywet{ "output_drywet" };
	const std::string latent_scale{ "latent_scale" };
	const std::string latent_bias{ "latent_bias" };
	const std::string latency_mode{ "latency_mode" };
	const std::string use_prior{ "use_prior" };
	const std::string prior_temperature{ "prior_temperature" };
} // namespace rave_parameters

namespace rave_ranges {
	const RAVEWwise::NormalisableRange<float> gainRange(-70.f, 12.f);
	const RAVEWwise::NormalisableRange<float> latentScaleRange(0.0f, 5.0f);
	const RAVEWwise::NormalisableRange<float> latentBiasRange(-3.0f, 3.0f);
} // namespace rave_ranges


//----------------------------------------------------------------------------------------------------------------------
// RaveWwiseFX
//----------------------------------------------------------------------------------------------------------------------

/// Wwise plugin that runs RAVE.

/// See https://www.audiokinetic.com/library/edge/?source=SDK&id=soundengine__plugins__effects.html
/// for the documentation about effect plug-ins

// TODO: Capitalize ported function and variable names

class RaveWwiseFX
    : public AK::IAkOutOfPlaceEffectPlugin
{
public:
    //------------------------------------------------------------------------------------------------------------------

    RaveWwiseFX();
    ~RaveWwiseFX();

	//------------------------------------------------------------------------------------------------------------------
    // IAkEffectPlugin::

    /// Plug-in initialization.
    /// Prepares the plug-in for data processing, allocates memory and sets up the initial conditions.
    AKRESULT Init(AK::IAkPluginMemAlloc* in_pAllocator, AK::IAkEffectPluginContext* in_pContext, AK::IAkPluginParam* in_pParams, AkAudioFormat& in_rFormat) override;

    /// Release the resources upon termination of the plug-in.
    AKRESULT Term(AK::IAkPluginMemAlloc* in_pAllocator) override;

    /// The reset action should perform any actions required to reinitialize the
    /// state of the plug-in to its original state (e.g. after Init() or on effect bypass).
    AKRESULT Reset() override;

    /// Plug-in information query mechanism used when the sound engine requires
    /// information about the plug-in to determine its behavior.
    AKRESULT GetPluginInfo(AkPluginInfo& out_rPluginInfo) override;

    /// Effect plug-in DSP execution.
    void Execute(AkAudioBuffer* in_pBuffer, AkUInt32 in_ulnOffset, AkAudioBuffer* out_pBuffer) override;

    /// Skips execution of some frames, when the voice is virtual playing from elapsed time.
    /// This can be used to simulate processing that would have taken place (e.g. update internal state).
    /// Return AK_DataReady or AK_NoMoreData, depending if there would be audio output or not at that point.
    AKRESULT TimeSkip(AkUInt32 &io_uFrames) override;

	//------------------------------------------------------------------------------------------------------------------
    // RaveAP functions

	void modelPerform();
	void detectAvailableModels();

	void mute();
	void unmute();
    bool getIsMuted() const { return _isMuted.load(); }
	
    void updateBufferSizes();

	void updateEngine(const std::string& modelFile);

	double getSampleRate() const { return _sampleRate; }


    //------------------------------------------------------------------------------------------------------------------
    // RaveAP variables

    std::unique_ptr<RAVE> _rave { nullptr };
    float _inputAmplitudeL { 0.f };
    float _inputAmplitudeR { 0.f };
    float _outputAmplitudeL { 0.f };
    float _outputAmplitudeR { 0.f };
    bool _plays { false };

private:
    //------------------------------------------------------------------------------------------------------------------

    RaveWwiseFXParams* m_pParams { nullptr };
    AK::IAkPluginMemAlloc* m_pAllocator { nullptr };
    AK::IAkEffectPluginContext* m_pContext { nullptr };

    //------------------------------------------------------------------------------------------------------------------
	// RaveAP variables

    std::mutex _engineUpdateMutex { };
    std::unique_ptr<BS::thread_pool> _engineThreadPool { nullptr };
    std::string _loadedModelName { };

    // Allocate some memory to use as the circular_buffer storage for each of the circular_buffer types to be created
    double _sampleRate { 0.0 };
    std::unique_ptr<circular_buffer<float, float>[]> _inBuffer { nullptr };
    std::unique_ptr<circular_buffer<float, float>[]> _outBuffer { nullptr };
    std::vector<std::unique_ptr<float[]>> _inModel { }, _outModel { };
    std::unique_ptr<std::thread> _computeThread { nullptr };

    bool _editorReady { false };

    float* _inFifoBuffer { nullptr };
    float* _outFifoBuffer { nullptr };

    std::atomic<float> _inputGainValue { 0.f }; // range = rave_ranges::gainRange, default = 0
    std::atomic<float> _thresholdValue { 0.f }; // min = -60, max = 0, default = 0
    std::atomic<float> _ratioValue { 1.f }; // min = 1, max = 10, default = 1
    std::atomic<float> _latentJitterValue { 0.f }; // min = 0, max = 3, default = 0
    std::atomic<float> _widthValue { 100.f }; // min = 0, max = 200, default = 100
    std::atomic<float> _outputGainValue { 0.f }; // range = rave_ranges::gainRange, default = 0
    std::atomic<float> _dryWetValue { 100.f }; // min = 0, max = 100, default = 100
    std::atomic<float> _limitValue { 1.f }; // bool behavior, default = true
    std::atomic<float> _channelMode { 1.f }; // min = 1, max = 3, default = 1

    // Latency mode contains the power of 2 of the current refresh rate.
    std::atomic<float> _latencyMode { 13 }; // min = 9, max = 15, default = 13
    
    std::atomic<float> _usePrior { 0.f }; // bool behavior, default = false
    std::atomic<float> _priorTemperature { 1.f }; // min = 0, max = 5, default = 1

    std::array<std::atomic<float>, AVAILABLE_DIMS> _latentScale { 0.f };
    std::array<std::atomic<float>, AVAILABLE_DIMS> _latentBias { 0.f };
    std::atomic<bool> _isMuted { true };

    enum class muting : int { ignore = 0, mute, unmute };

    std::atomic<muting> _fadeScheduler{ muting::mute };
    RAVEWwise::LinearSmoothedValue<float> _smoothedFadeInOut { };
    RAVEWwise::LinearSmoothedValue<float> _smoothedWetGain { };
    RAVEWwise::LinearSmoothedValue<float> _smoothedDryGain { };

    // TODO: DSP effects
    // - Compressor
    // - Input gain
    // - Output gain
    // - Limiter
    // - Dry/wet mixer
};

#endif // RaveWwiseFX_H
