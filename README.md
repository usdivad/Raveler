# RAVE Wwise

Wwise port of the [RAVE VST](https://github.com/acids-ircam/rave_vst) plugin.


## Setup

**(TODO: Set this up in CMake or something similar)**

1. Download LibTorch 1.11.0 from  https://pytorch.org/ and place the `libtorch` directory in `RaveWwwise/Libraries/torch` (so that the full path to the library is `RaveWwise/Libraries/torch/libtorch`)
	- CPU: https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-1.11.0%2Bcpu.zip
	- CUDA 11.3:  https://download.pytorch.org/libtorch/cu113/libtorch-win-shared-with-deps-1.11.0%2Bcu113.zip

2. Run Wwise premake step: `python "C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\Scripts\Build\Plugins\wp.py" premake Authoring`

3. Apply settings to the RaveWwiseFX project:
	1. C/C++ > Additional Include Directories: `..\Libraries\torch\libtorch\include;..\Libraries\torch\libtorch\include\torch\csrc\api\include;C:\Program Files (x86)\Audiokinetic\Wwise 2021.1.9.7847\SDK\include;%(AdditionalIncludeDirectories)`
		- This adds the LibTorch directories to the include paths
	2. C/C++ > Preprocessor: `NDEBUG;WIN64;WIN32_LEAN_AND_MEAN;NOMINMAX;%(PreprocessorDefinitions)`
		- `NOMINMAX` prevents conflicts with `std`
	3. C/C++ > All Options > Additional Options: `/utf-8 /d2FH4- /GR %(AdditionalOptions)`
		- `/GR` enables run-time type information (used for `std::dynamic_pointer_cast()` in `torch/nn/cloneable.h`)
	4. Librarian > Additional Dependencies: `..\Libraries\torch\libtorch\lib\c10.lib;C:\Users\DavidSu\Documents\DDS\RAVE_Wwise\RaveWwise\Libraries\torch\libtorch\lib\torch.lib;..\Libraries\torch\libtorch\lib\torch_cpu.lib;;;;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib`
		- Adds dependencies needed for LibTorch
	5. Librarian > Additional Library Directories: `..\Libraries\torch\libtorch\lib;%(AdditionalLibraryDirectories)`
		- Adds library directories for LibTorch