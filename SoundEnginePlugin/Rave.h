#pragma once
#include <string>
#include <torch/script.h>
#include <torch/torch.h>

#include <AK/Tools/Common/AkPlatformFuncs.h>

//----------------------------------------------------------------------------------------------------------------------

#define MAX_LATENT_BUFFER_SIZE 32

#define BUFFER_LENGTH 32768
#define DRY_BUFFER_LENGTH BUFFER_LENGTH * 4 // Increase dry buffer size to account for latency compensation settings

#define DEBUG_MODEL 0

using namespace torch::indexing;

//----------------------------------------------------------------------------------------------------------------------

namespace RaveWwise {
    
    // Range with a start and end value
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

//----------------------------------------------------------------------------------------------------------------------

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

#if DEBUG_MODEL
    AKPLATFORM::OutputDebugMsg("\n[ ] RAVE - Error loading model: ");
    AKPLATFORM::OutputDebugMsg(rave_model_file.c_str());
    AKPLATFORM::OutputDebugMsg("\t // ");
    AKPLATFORM::OutputDebugMsg(e.what());
    AKPLATFORM::OutputDebugMsg("\t // ");
    AKPLATFORM::OutputDebugMsg(e.msg().c_str());
#endif

      return;
    }

    this->model_path = std::string(rave_model_file);
    auto named_buffers = this->model.named_buffers();
    auto named_attributes = this->model.named_attributes();
    this->has_prior = false;
    this->prior_params = torch::zeros({0});

#if DEBUG_MODEL
    AKPLATFORM::OutputDebugMsg("\n[ ] RAVE - Model successfully loaded: ");
    AKPLATFORM::OutputDebugMsg(rave_model_file.c_str());
  AKPLATFORM::OutputDebugMsg("\n");
#endif

    bool found_model_as_attribute = false; // TODO: Check by module type instead (e.g. Combined vs. ScriptedRAVE)?
  for (auto const& attr : named_attributes) {
    if (attr.name == "_rave") {
      found_model_as_attribute = true;

#if DEBUG_MODEL
            AKPLATFORM::OutputDebugMsg("Found _rave model as named attribute");
      AKPLATFORM::OutputDebugMsg("\n");
#endif

    }
    else if (attr.name == "stereo" || attr.name == "_rave.stereo") {
      stereo = attr.value.toBool();

#if DEBUG_MODEL
      AKPLATFORM::OutputDebugMsg("Stereo? ");
      AKPLATFORM::OutputDebugMsg(stereo ? "true" : "false");
      AKPLATFORM::OutputDebugMsg("\n");
#endif

    }
  }

  if (found_model_as_attribute)
  {
    // Named buffers (for RAVE v1)
#if DEBUG_MODEL
    AKPLATFORM::OutputDebugMsg("\n");
    AKPLATFORM::OutputDebugMsg("Using named buffers");
    AKPLATFORM::OutputDebugMsg("\n");
#endif

    for (auto const& buf : named_buffers) {
      if (buf.name == "_rave.sampling_rate") {
        this->sr = buf.value.item<int>();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tSampling rate: ");
        AKPLATFORM::OutputDebugMsg(std::to_string(this->sr).c_str());
#endif
      }
      if (buf.name == "_rave.latent_size") {
        this->latent_size = buf.value.item<int>();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tLatent size: ");
        AKPLATFORM::OutputDebugMsg(std::to_string(this->latent_size).c_str());
#endif

      }
      if (buf.name == "encode_params") {
        this->encode_params = buf.value;

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tEncode parameters: ");
        AKPLATFORM::OutputDebugMsg(this->encode_params.toString().c_str());
#endif

      }
      if (buf.name == "decode_params") {
        this->decode_params = buf.value;

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tDecode parameters: ");
        AKPLATFORM::OutputDebugMsg(this->decode_params.toString().c_str());
#endif

      }
      if (buf.name == "prior_params") {
        this->prior_params = buf.value;
        this->has_prior = true;

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tPrior parameters: ");
        AKPLATFORM::OutputDebugMsg(this->prior_params.toString().c_str());
#endif

      }
    }
  }
  else
  {
    // Named attributes (for ONNX / RAVE v2)

#if DEBUG_MODEL
    AKPLATFORM::OutputDebugMsg("\n");
    AKPLATFORM::OutputDebugMsg("Using named attributes");
    AKPLATFORM::OutputDebugMsg("\n");
#endif

    for (auto const& attr : named_attributes) {
      if (attr.name == "sampling_rate") {
        this->sr = attr.value.toInt();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tSampling rate: ");
        AKPLATFORM::OutputDebugMsg(std::to_string(this->sr).c_str());
#endif

      }
      if (attr.name == "full_latent_size") { // TODO: Keep track of "latent_size" as well?
        this->latent_size = attr.value.toInt();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tLatent size: ");
        AKPLATFORM::OutputDebugMsg(std::to_string(this->latent_size).c_str());
#endif

      }
      if (attr.name == "encode_params") {
        this->encode_params = attr.value.toTensor();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tEncode parameters: ");
        AKPLATFORM::OutputDebugMsg(this->encode_params.toString().c_str());
#endif

      }
      if (attr.name == "decode_params") {
        this->decode_params = attr.value.toTensor();

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tDecode parameters: ");
        AKPLATFORM::OutputDebugMsg(this->decode_params.toString().c_str());
#endif

      }
      if (attr.name == "prior_params") {
        this->prior_params = attr.value.toTensor();
        this->has_prior = true;

#if DEBUG_MODEL
        AKPLATFORM::OutputDebugMsg("\tPrior parameters: ");
        AKPLATFORM::OutputDebugMsg(this->prior_params.toString().c_str());
#endif
      }
    }
  }
    
#if DEBUG_MODEL
  AKPLATFORM::OutputDebugMsg("\tFull latent size: ");
  AKPLATFORM::OutputDebugMsg(std::to_string(getFullLatentDimensions()).c_str());

  AKPLATFORM::OutputDebugMsg("\tRatio: ");
  AKPLATFORM::OutputDebugMsg(std::to_string(getModelRatio()).c_str());

  AKPLATFORM::OutputDebugMsg("\n");
#endif

    c10::InferenceMode guard;
    inputs_rave.clear();
    inputs_rave.push_back(torch::ones({1, 1, getModelRatio()}));
    resetLatentBuffer();
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

  std::string dump_to_str() {
      return this->model.dump_to_str(true, false, false);
  }

  torch::Tensor decode(const torch::Tensor input) {
    c10::InferenceMode guard;
    inputs_rave[0] = input;
    auto y = this->model.get_method("decode")(inputs_rave).toTensor();
    return y;
  }

  RaveWwise::Range<float> getValidBufferSizes() {
    return RaveWwise::Range<float>(getModelRatio(), BUFFER_LENGTH);
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

  bool isStereo() const { return stereo; }

  at::Tensor getLatentBuffer() { return latent_buffer; }

  bool hasMethod(const std::string& method_name) const {
  return this->model.find_method(method_name).has_value();
  }

private:
  torch::jit::Module model;
  int sr;
  int latent_size;
  bool has_prior = false;
  bool stereo = false;
  std::string model_path;
  at::Tensor encode_params;
  at::Tensor decode_params;
  at::Tensor prior_params;
  at::Tensor latent_buffer;
  std::vector<torch::jit::IValue> inputs_rave;
  RaveWwise::Range<float> validBufferSizeRange;
};
