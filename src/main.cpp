#include <DaisyDuino.h>
#include <cmath>

static float sample_rate_hz = 96000.0f;

static constexpr int kInputChannel = 0;
static constexpr float kCarrierHz = 39500.0f;
static float carrier_phase = 0.0f;
static constexpr float kModDepth = 1.0f;
static constexpr float kCarrierLevel = 0.5f;
static constexpr float kBasebandGain = 1.0f;

struct SimpleBiquad
{
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float z1 = 0.0f, z2 = 0.0f;

  inline float Process(float x)
  {
    const float y = (b0 * x) + z1;
    z1 = (b1 * x) - (a1 * y) + z2;
    z2 = (b2 * x) - (a2 * y);
    return y;
  }
};

static SimpleBiquad base_hpf_l;
static SimpleBiquad base_hpf_r;
static SimpleBiquad base_lpf_l;
static SimpleBiquad base_lpf_r;
static SimpleBiquad low_shelf_l;
static SimpleBiquad low_shelf_r;
static SimpleBiquad pre_emph_l;
static SimpleBiquad pre_emph_r;
static SimpleBiquad bandpass_hpf_l;
static SimpleBiquad bandpass_hpf_r;
static SimpleBiquad bandpass_lpf_l;
static SimpleBiquad bandpass_lpf_r;
static SimpleBiquad bandpass_lpf2_l;
static SimpleBiquad bandpass_lpf2_r;
static SimpleBiquad post_hpf_l;
static SimpleBiquad post_hpf_r;
static SimpleBiquad post_hpf2_l;
static SimpleBiquad post_hpf2_r;

static constexpr size_t kHilbertTaps = 256;
static constexpr size_t kHilbertCenter = (kHilbertTaps - 1) / 2;
static float hilbert_coeffs[kHilbertTaps] = {};
static float hilbert_state_l[kHilbertTaps] = {};
static float hilbert_state_r[kHilbertTaps] = {};
static size_t hilbert_index = 0;

static void ConfigurePeaking(SimpleBiquad& bq, float fs, float f0, float q, float gain_db)
{
  const float A = powf(10.0f, gain_db / 40.0f);
  const float w0 = 2.0f * 3.14159265358979323846f * (f0 / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);

  const float b0 = 1.0f + alpha * A;
  const float b1 = -2.0f * cosw0;
  const float b2 = 1.0f - alpha * A;
  const float a0 = 1.0f + alpha / A;
  const float a1 = -2.0f * cosw0;
  const float a2 = 1.0f - alpha / A;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static void ConfigureHighShelf(SimpleBiquad& bq, float fs, float f0, float q, float gain_db)
{
  const float A = powf(10.0f, gain_db / 40.0f);
  const float w0 = 2.0f * 3.14159265358979323846f * (f0 / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);
  const float sqrtA = sqrtf(A);

  const float b0 =    A * ((A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha);
  const float b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosw0);
  const float b2 =    A * ((A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha);
  const float a0 =         (A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha;
  const float a1 =  2.0f * ((A - 1.0f) - (A + 1.0f) * cosw0);
  const float a2 =         (A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static void ConfigureLowShelf(SimpleBiquad& bq, float fs, float f0, float q, float gain_db)
{
  const float A = powf(10.0f, gain_db / 40.0f);
  const float w0 = 2.0f * 3.14159265358979323846f * (f0 / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);
  const float sqrtA = sqrtf(A);

  const float b0 =    A * ((A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha);
  const float b1 =  2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosw0);
  const float b2 =    A * ((A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha);
  const float a0 =         (A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha;
  const float a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosw0);
  const float a2 =         (A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static float Compress(float x, float& env, float threshold, float ratio, float attack, float release)
{
  const float ax = fabsf(x);
  if (ax > env)
    env += (ax - env) * attack;
  else
    env += (ax - env) * release;

  float gain = 1.0f;
  if (env > threshold)
  {
    const float over = env / threshold;
    gain = powf(over, (1.0f / ratio) - 1.0f);
  }
  return x * gain;
}

static void ConfigureHighpass(SimpleBiquad& bq, float fs, float fc, float q)
{
  const float w0 = 2.0f * 3.14159265358979323846f * (fc / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);

  const float b0 = (1.0f + cosw0) * 0.5f;
  const float b1 = -(1.0f + cosw0);
  const float b2 = (1.0f + cosw0) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * cosw0;
  const float a2 = 1.0f - alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static void ConfigureLowpass(SimpleBiquad& bq, float fs, float fc, float q)
{
  const float w0 = 2.0f * 3.14159265358979323846f * (fc / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);

  const float b0 = (1.0f - cosw0) * 0.5f;
  const float b1 = 1.0f - cosw0;
  const float b2 = (1.0f - cosw0) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * cosw0;
  const float a2 = 1.0f - alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static void InitHilbertCoeffs()
{
  for (size_t i = 0; i < kHilbertTaps; ++i)
  {
    const int n = static_cast<int>(i) - static_cast<int>(kHilbertCenter);
    if (n == 0)
    {
      hilbert_coeffs[i] = 0.0f;
      continue;
    }

    const bool is_odd = (n & 1) != 0;
    const float ideal = is_odd ? (2.0f / (3.14159265358979323846f * static_cast<float>(n))) : 0.0f;
    const float t = static_cast<float>(i) / static_cast<float>(kHilbertTaps - 1);
    const float w = 0.35875f
            - 0.48829f * cosf(2.0f * 3.14159265358979323846f * t)
            + 0.14128f * cosf(4.0f * 3.14159265358979323846f * t)
            - 0.01168f * cosf(6.0f * 3.14159265358979323846f * t);
    hilbert_coeffs[i] = ideal * w;
  }

  for (size_t i = 0; i < kHilbertTaps; ++i)
  {
    hilbert_state_l[i] = 0.0f;
    hilbert_state_r[i] = 0.0f;
  }
  hilbert_index = 0;
}

static float ProcessHilbert(float x, float* state)
{
  state[hilbert_index] = x;
  float y = 0.0f;

  size_t idx = hilbert_index;
  for (size_t i = 0; i < kHilbertTaps; ++i)
  {
    y += hilbert_coeffs[i] * state[idx];
    if (idx == 0)
      idx = kHilbertTaps - 1;
    else
      --idx;
  }

  return y;
}

// Output:
// - out[0]: input passthrough (96 kHz)
// - out[1]: input passthrough (96 kHz)

void AudioCallback(float** in, float** out, size_t size)
{
  const bool have_in1 = (in != nullptr) && (in[kInputChannel] != nullptr);
  const bool have_in2 = (in != nullptr) && (in[1] != nullptr);
  const float phase_inc = (2.0f * 3.14159265358979323846f * kCarrierHz) / sample_rate_hz;
  const float attack = 1.0f - expf(-1.0f / (0.005f * sample_rate_hz));
  const float release = 1.0f - expf(-1.0f / (0.050f * sample_rate_hz));
  const float threshold = 0.6f;
  const float ratio = 3.0f;
  static float env_l = 0.0f;
  static float env_r = 0.0f;

  for (size_t i = 0; i < size; i++)
  {
    float x = have_in1 ? in[kInputChannel][i] : 0.0f;
    float y = have_in2 ? in[1][i] : 0.0f;

    x = base_lpf_l.Process(base_hpf_l.Process(x));
    y = base_lpf_r.Process(base_hpf_r.Process(y));

    x = low_shelf_l.Process(x);
    y = low_shelf_r.Process(y);

    // x = pre_emph_l.Process(x);
    // y = pre_emph_r.Process(y);

    // x = Compress(x, env_l, threshold, ratio, attack, release);
    // y = Compress(y, env_r, threshold, ratio, attack, release);

    const float carrier = sinf(carrier_phase);
    carrier_phase += phase_inc;
    if (carrier_phase > 2.0f * 3.14159265358979323846f)
      carrier_phase -= 2.0f * 3.14159265358979323846f;

    const float mod_l = (kCarrierLevel + (kModDepth * kBasebandGain * x)) * carrier;
    const float mod_r = (kCarrierLevel + (kModDepth * kBasebandGain * y)) * carrier;

    const float bp_l = bandpass_lpf2_l.Process(bandpass_lpf_l.Process(bandpass_hpf_l.Process(mod_l)));
    const float bp_r = bandpass_lpf2_r.Process(bandpass_lpf_r.Process(bandpass_hpf_r.Process(mod_r)));

    out[0][i] = post_hpf2_l.Process(post_hpf_l.Process(bp_l));
    out[1][i] = post_hpf2_r.Process(post_hpf_r.Process(bp_r));
  }
}

void setup()
{
  // ADC supports up to 96 kHz on this hardware.
  DAISY.init(DAISY_SEED, AUDIO_SR_96K);
  DAISY.SetAudioBlockSize(48);
  sample_rate_hz = DAISY.get_samplerate();

  ConfigureHighpass(base_hpf_l, sample_rate_hz, 200.0f, 0.70710678f);
  ConfigureHighpass(base_hpf_r, sample_rate_hz, 200.0f, 0.70710678f);
  ConfigureLowpass(base_lpf_l, sample_rate_hz, 5000.0f, 0.70710678f);
  ConfigureLowpass(base_lpf_r, sample_rate_hz, 5000.0f, 0.70710678f);
  ConfigureLowShelf(low_shelf_l, sample_rate_hz, 200.0f, 1.5f, -3.0f);
  ConfigureLowShelf(low_shelf_r, sample_rate_hz, 200.0f, 1.5f, -3.0f);
  // ConfigureHighShelf(pre_emph_l, sample_rate_hz, 3000.0f, 0.7f, 6.0f);
  // ConfigureHighShelf(pre_emph_r, sample_rate_hz, 3000.0f, 0.7f, 6.0f);
  ConfigureHighpass(bandpass_hpf_l, sample_rate_hz, 24000.0f, 0.70710678f);
  ConfigureHighpass(bandpass_hpf_r, sample_rate_hz, 24000.0f, 0.70710678f);
  ConfigureLowpass(bandpass_lpf_l, sample_rate_hz, 45000.0f, 0.70710678f);
  ConfigureLowpass(bandpass_lpf_r, sample_rate_hz, 45000.0f, 0.70710678f);
  ConfigureLowpass(bandpass_lpf2_l, sample_rate_hz, 45000.0f, 0.70710678f);
  ConfigureLowpass(bandpass_lpf2_r, sample_rate_hz, 45000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf_l, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf_r, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf2_l, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf2_r, sample_rate_hz, 19000.0f, 0.70710678f);

  InitHilbertCoeffs();

  DAISY.begin(AudioCallback);
}

void loop() {}