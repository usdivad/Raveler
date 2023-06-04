# RAVE Wwise

Wwise port of the [RAVE VST](https://github.com/acids-ircam/rave_vst) plugin.


## Setup

**(TODO: Set this up in CMake or something similar)**

1. Download and add library dependencies:
	1. Download LibTorch 1.11.0 from  https://pytorch.org/ and place the `libtorch` directory in `RaveWwwise/Libraries/torch` (so that the full path to the library is `RaveWwise/Libraries/torch/libtorch`)
		- CPU: https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-1.11.0%2Bcpu.zip
		- CUDA 11.3:  https://download.pytorch.org/libtorch/cu113/libtorch-win-shared-with-deps-1.11.0%2Bcu113.zip
	2. Similarly download and add ONNX to `RaveWwise/Libraries/onnx` (so the full path to the library is `RaveWwise/Libraries/onnx/onnxruntime`)
		- https://github.com/microsoft/onnxruntime/releases/download/v1.14.1/onnxruntime-win-x64-1.14.1.zip
	3. Similarly download and add JUCE 6.1.6 modules to `RaveWwise/Libraries/juce` (so that the full path to the library is `RaveWwise/Libraries/juce/modules`)
		- https://github.com/juce-framework/JUCE/tree/6.1.6/modules
		- Eventually we'll replace our JUCE dependencies with custom/equivalent classes

2. Run Wwise premake step: `python "C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Scripts\Build\Plugins\wp.py" premake Authoring`

3. Apply general settings to RaveWwise_Authoring_Windows_vc160 solution:
	1. Retarget both RaveWwise and RaveWwiseFX projects to latest 10.0
	2. (optional) Properties > C/C++ > Optimization > `Disabled (/Od)`

4. Apply library-specific settings to the RaveWwiseFX project:
	1. C/C++ > General > Additional Include Directories: `..\Libraries\torch\libtorch\include;..\Libraries\torch\libtorch\include\torch\csrc\api\include;..\Libraries\onnx\onnxruntime\include;..\Libraries\juce\modules;..\Libraries\juce\JuceLibraryCode;C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\SDK\include;%(AdditionalIncludeDirectories)`
		- This adds the LibTorch, ONNX, and JUCE directories to the include paths
	2. C/C++ > Preprocessor > Preprocessor Definitions: `NDEBUG;WIN64;WIN32_LEAN_AND_MEAN;NOMINMAX;%(PreprocessorDefinitions)`
		- `NOMINMAX` prevents conflicts with `std`
	3. C/C++ > All Options > Additional Options: `/utf-8 /d2FH4- /GR %(AdditionalOptions)`
		- `/GR` enables run-time type information (used for `std::dynamic_pointer_cast()` in `torch/nn/cloneable.h`)
	4. Librarian > Additional Dependencies: `..\Libraries\torch\libtorch\lib\c10.lib;..\Libraries\torch\libtorch\lib\torch.lib;..\Libraries\torch\libtorch\lib\torch_cpu.lib;..\Libraries\onnx\onnxruntime\lib\onnxruntime.lib;..\Libraries\onnx\onnxruntime\lib\onnxruntime_providers_shared.lib;;;;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib`
		- Adds dependencies needed for LibTorch and ONNX
	5. Librarian > Additional Library Directories: `..\Libraries\torch\libtorch\lib;..\Libraries\onnx\onnxruntime\lib;%(AdditionalLibraryDirectories)`
		- Adds library directories for LibTorch and ONNX

5. Build, either using Visual Studio or `python "C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Scripts\Build\Plugins\wp.py" build -c Release -x x64 -t vc160 Authoring`

6. Copy library files (in `RaveWwise/Libraries/torch/libtorch/lib` and `RaveWwise/Libraries/onnx/onnxruntime/lib`) to `C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Authoring\x64\Release\bin`
	- Make sure *all* the files are in the top-level of `bin`, e.g. `bin/onnxruntime.dll`, not `bin/onnx/onnxruntime.dll`)