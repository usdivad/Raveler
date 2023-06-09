#pragma once
#include <string>
#include <JuceHeader.h>
#include <torch/script.h>
#include <torch/torch.h>

// For debug printing
#include <AK/Tools/Common/AkPlatformFuncs.h>

#define MAX_LATENT_BUFFER_SIZE 32
#define BUFFER_LENGTH 32768
using namespace torch::indexing;

// TODO: Move this to its own file
namespace RAVEWwise {
    
    // Replacement for juce::Range
    template <typename T>
    struct Range {

        Range() = default;

        Range(T inStart, T inEnd) {
            start = inStart;
            end = inEnd;
        }

        T start;
        T end;
    };
}

// TODO: Inherit from custom implementation of juce::ChangeBroadcaster?
//       (May be unnecessary since in RAVE VST the ChangeBroadcaster functionality is used for the plugin editor)
class RAVE {

public:
  RAVE() {
    torch::jit::getProfilingMode() = false;
    c10::InferenceMode guard;
    torch::jit::setGraphExecutorOptimize(true);
  }

  void load_model(const std::string &rave_model_file) {
    try {
      c10::InferenceMode guard;
      this->model = torch::jit::load(rave_model_file);
    } catch (const c10::Error &e) {
      std::cerr << e.what();
      std::cerr << e.msg();
      std::cerr << "error loading the model\n";
      return;
    }

    this->model_path = std::string(rave_model_file);
    auto named_buffers = this->model.named_buffers();
    this->has_prior = false;
    this->prior_params = torch::zeros({0});

    std::cout << "[ ] RAVE - Model successfully loaded: " << rave_model_file
              << std::endl;

    AKPLATFORM::OutputDebugMsg("\n[ ] RAVE - Model successfully loaded: ");
    AKPLATFORM::OutputDebugMsg(rave_model_file.c_str());

    for (auto const &i : named_buffers) {
      if (i.name == "_rave.sampling_rate") {
        this->sr = i.value.item<int>();
        std::cout << "\tSampling rate: " << this->sr << std::endl;

		AKPLATFORM::OutputDebugMsg("\tSampling rate: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(this->sr).c_str());
      }
      if (i.name == "_rave.latent_size") {
        this->latent_size = i.value.item<int>();
        std::cout << "\tLatent size: " << this->latent_size << std::endl;

		AKPLATFORM::OutputDebugMsg("\tLatent size: ");
		AKPLATFORM::OutputDebugMsg(std::to_string(this->latent_size).c_str());
      }
      if (i.name == "encode_params") {
        this->encode_params = i.value;
        std::cout << "\tEncode parameters: " << this->encode_params
                  << std::endl;

		AKPLATFORM::OutputDebugMsg("\tEncode parameters: ");
		AKPLATFORM::OutputDebugMsg(this->encode_params.toString().c_str());
      }
      if (i.name == "decode_params") {
        this->decode_params = i.value;
        std::cout << "\tDecode parameters: " << this->decode_params
                  << std::endl;

		AKPLATFORM::OutputDebugMsg("\tDecode parameters: ");
		AKPLATFORM::OutputDebugMsg(this->decode_params.toString().c_str());
      }
      if (i.name == "prior_params") {
        this->prior_params = i.value;
        this->has_prior = true;
        std::cout << "\tPrior parameters: " << this->prior_params << std::endl;

		AKPLATFORM::OutputDebugMsg("\tPrior parameters: ");
		AKPLATFORM::OutputDebugMsg(this->prior_params.toString().c_str());
      }
    }
    
    std::cout << "\tFull latent size: " << getFullLatentDimensions()
              << std::endl;
	AKPLATFORM::OutputDebugMsg("\tFull latent size: ");
	AKPLATFORM::OutputDebugMsg(std::to_string(getFullLatentDimensions()).c_str());

    std::cout << "\tRatio: " << getModelRatio() << std::endl;
	AKPLATFORM::OutputDebugMsg("\tRatio: ");
	AKPLATFORM::OutputDebugMsg(std::to_string(getModelRatio()).c_str());

	AKPLATFORM::OutputDebugMsg("\n");


    c10::InferenceMode guard;
    inputs_rave.clear();
    inputs_rave.push_back(torch::ones({1, 1, getModelRatio()}));
    resetLatentBuffer();
    //sendChangeMessage(); // TODO: Implement or remove
  }

  torch::Tensor sample_prior(const int n_steps, const float temperature) {
    c10::InferenceMode guard;
    inputs_rave[0] = torch::ones({1, 1, n_steps}) * temperature;
    torch::Tensor prior =
        this->model.get_method("prior")(inputs_rave).toTensor();
    return prior;
  }

  torch::Tensor encode(const torch::Tensor input) {
    c10::InferenceMode guard;
    inputs_rave[0] = input;
    auto y = this->model.get_method("encode")(inputs_rave).toTensor();
    return y;
  }

  std::vector<torch::Tensor> encode_amortized(const torch::Tensor input) {
    c10::InferenceMode guard;
    inputs_rave[0] = input;
    auto stats = this->model.get_method("encode_amortized")(inputs_rave)
                     .toTuple()
                     .get()
                     ->elements();
    torch::Tensor mean = stats[0].toTensor();
    torch::Tensor std = stats[1].toTensor();
    std::vector<torch::Tensor> mean_std = {mean, std};
    return mean_std;
  }

  torch::Tensor decode(const torch::Tensor input) {
    c10::InferenceMode guard;
    inputs_rave[0] = input;
    auto y = this->model.get_method("decode")(inputs_rave).toTensor();
    return y;
  }

  RAVEWwise::Range<float> getValidBufferSizes() {
    return RAVEWwise::Range<float>(getModelRatio(), BUFFER_LENGTH);
  }

  unsigned int getLatentDimensions() {
    int tmp = decode_params.index({0}).item<int>();
    assert(tmp >= 0);
    return (unsigned int)tmp;
  }

  unsigned int getEncodeChannels() {
    int tmp = encode_params.index({0}).item<int>();
    assert(tmp >= 0);
    return (unsigned int)tmp;
  }

  unsigned int getDecodeChannels() {
    int tmp = decode_params.index({3}).item<int>();
    assert(tmp >= 0);
    return (unsigned int)tmp;
  }

  int getModelRatio() { return encode_params.index({3}).item<int>(); }

  float zPerSeconds() { return encode_params.index({3}).item<float>() / sr; }

  int getFullLatentDimensions() { return latent_size; }

  int getInputBatches() { return encode_params.index({1}).item<int>(); }

  int getOutputBatches() { return decode_params.index({3}).item<int>(); }

  void resetLatentBuffer() { latent_buffer = torch::zeros({0}); }

  void writeLatentBuffer(at::Tensor latent) {
    if (latent_buffer.size(0) == 0) {
      latent_buffer = latent;
    } else {
      latent_buffer = torch::cat({latent_buffer, latent}, 2);
    }
    if (latent_buffer.size(2) > MAX_LATENT_BUFFER_SIZE) {
      latent_buffer = latent_buffer.index(
          {"...", Slice(-MAX_LATENT_BUFFER_SIZE, None, None)});
    }
  }

  bool hasPrior() { return has_prior; }

  at::Tensor getLatentBuffer() { return latent_buffer; }

private:
  torch::jit::Module model;
  int sr;
  int latent_size;
  bool has_prior = false;
  std::string model_path;
  at::Tensor encode_params;
  at::Tensor decode_params;
  at::Tensor prior_params;
  at::Tensor latent_buffer;
  std::vector<torch::jit::IValue> inputs_rave;
  RAVEWwise::Range<float> validBufferSizeRange;
};
