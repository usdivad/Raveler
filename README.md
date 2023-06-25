# RaveWwise

RaveWwise is a plugin that allows you to run [RAVE](https://github.com/acids-ircam/RAVE) models directly within Wwise, enabling real-time timbre transfer via neural audio synthesis in a game audio setting.

Much of the plugin implementation is based on the [RAVE VST](https://github.com/acids-ircam/rave_vst) project.


## Setup

**(TODO: Set this up in CMake or something similar)**

1. Download and add library dependencies:
	1. Download LibTorch 1.11.0 from  https://pytorch.org/ and place the `libtorch` directory in `RaveWwwise/Libraries/torch` (so that the full path to the library is `RaveWwise/Libraries/torch/libtorch`)
		- CPU: https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-1.11.0%2Bcpu.zip
		- CUDA 11.3:  https://download.pytorch.org/libtorch/cu113/libtorch-win-shared-with-deps-1.11.0%2Bcu113.zip
	2. Similarly download and add ONNX 1.14.1 to `RaveWwise/Libraries/onnx` (so the full path to the library is `RaveWwise/Libraries/onnx/onnxruntime`)
		- https://github.com/microsoft/onnxruntime/releases/download/v1.14.1/onnxruntime-win-x64-1.14.1.zip
	4. Download BS::thread_pool 3.5.0 and add the BS_thread_pool.hpp header to `RaveWwise/Libraries/bs_thread_pool` (so that the full path to the header is `RaveWwise/Libraries/bs_thread_pool/BS_thread_pool.hpp`)
		- https://github.com/bshoshany/thread-pool/releases/tag/v3.5.0

2. Run Wwise premake step, substituting 2021.1.9.7847 with your version of Wwise: `python "C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Scripts\Build\Plugins\wp.py" premake Authoring`

3. Apply general settings to RaveWwise_Authoring_Windows_vc160 solution:
	1. Retarget both RaveWwise and RaveWwiseFX projects to latest 10.0

4. Apply library-specific settings to the RaveWwiseFX project:
	1. General > C++ Language Standard: `/std:c++17` (C++ 17 standard)
		- This is required for BS::thread_pool

5. Go to View > Property Manager > Add Existing Property Sheet and add `SoundEnginePlugin/RaveWwiseFXProperties.props`, **OR** apply the following settings manually, substituting 2021.1.9.7847 with your version of Wwise:
	1. (optional, for debugging) Properties > C/C++ > Optimization > `Disabled (/Od)`
	1. C/C++ > General > Additional Include Directories: `..\Libraries\torch\libtorch\include;..\Libraries\torch\libtorch\include\torch\csrc\api\include;..\Libraries\onnx\onnxruntime\include;C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\SDK\include;%(AdditionalIncludeDirectories)`
		- This adds the LibTorch and ONNX directories to the include paths
	1. C/C++ > Preprocessor > Preprocessor Definitions: `NDEBUG;WIN64;WIN32_LEAN_AND_MEAN;NOMINMAX;%(PreprocessorDefinitions)`
		- `NOMINMAX` prevents conflicts with `std`
	1. C/C++ > All Options > Additional Options: `/utf-8 /d2FH4- /GR %(AdditionalOptions)`
		- `/GR` enables run-time type information (used for `std::dynamic_pointer_cast()` in `torch/nn/cloneable.h`)
	1. Librarian > Additional Dependencies: `..\Libraries\torch\libtorch\lib\c10.lib;..\Libraries\torch\libtorch\lib\torch.lib;..\Libraries\torch\libtorch\lib\torch_cpu.lib;..\Libraries\onnx\onnxruntime\lib\onnxruntime.lib;..\Libraries\onnx\onnxruntime\lib\onnxruntime_providers_shared.lib;;;;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib`
		- Adds dependencies needed for LibTorch and ONNX
	1. Librarian > Additional Library Directories: `..\Libraries\torch\libtorch\lib;..\Libraries\onnx\onnxruntime\lib;%(AdditionalLibraryDirectories)`
		- Adds library directories for LibTorch and ONNX

6. Build for Wwise authoring, either using Visual Studio or `python "C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Scripts\Build\Plugins\wp.py" build -c Release -x x64 -t vc160 Authoring`

7. Copy library files (in `RaveWwise/Libraries/torch/libtorch/lib` and `RaveWwise/Libraries/onnx/onnxruntime/lib`) to `C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Authoring\x64\Release\bin` (substituting 2021.1.9.7847 with your version of Wwise)
	- Make sure *all* the files are in the top-level of `bin`, e.g. `bin/onnxruntime.dll`, not `bin/onnx/onnxruntime.dll`)

8. Build and package for desired target platforms (**TODO: Windows example**)
	- https://www.audiokinetic.com/en/library/edge/?source=SDK&id=effectplugin_tools_building.html
	- Make sure to copy library files to their appropriate destinations as well