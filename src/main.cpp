#include <DaisyDuino.h>
#include <cmath>

static float sample_rate_hz = 96000.0f;

static constexpr int kInputChannel = 0;
static constexpr float kCarrierHz = 39500.0f;
static float carrier_phase = 0.0f;
static constexpr bool kEnableEq = false;

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

static SimpleBiquad bass_eq;
static SimpleBiquad mid_eq;
static SimpleBiquad post_hpf_l;
static SimpleBiquad post_hpf_r;
static SimpleBiquad post_hpf2_l;
static SimpleBiquad post_hpf2_r;

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

// Output:
// - out[0]: input passthrough (96 kHz)
// - out[1]: input passthrough (96 kHz)

void AudioCallback(float** in, float** out, size_t size)
{
  const bool have_in1 = (in != nullptr) && (in[kInputChannel] != nullptr);
  const bool have_in2 = (in != nullptr) && (in[1] != nullptr);
  const float phase_inc = (2.0f * 3.14159265358979323846f * kCarrierHz) / sample_rate_hz;

  for (size_t i = 0; i < size; i++)
  {
    float x = have_in1 ? in[kInputChannel][i] * 2 : 0.0f;
    float y = have_in2 ? in[1][i] * 2 : 0.0f;

    if (kEnableEq)
    {
      x = mid_eq.Process(bass_eq.Process(x));
      y = mid_eq.Process(bass_eq.Process(y));
    }
    const float carrier = (1.0f + sinf(carrier_phase)) / 2.0f;
    carrier_phase += phase_inc;
    if (carrier_phase > 2.0f * 3.14159265358979323846f)
      carrier_phase -= 2.0f * 3.14159265358979323846f;

    const float mod_l = y * carrier;
    const float mod_r = x * carrier;

    out[0][i] = post_hpf2_l.Process(post_hpf_l.Process(mod_l));
    out[1][i] = post_hpf2_r.Process(post_hpf_r.Process(mod_r));
  }
}

void setup()
{
  // ADC supports up to 96 kHz on this hardware.
  DAISY.init(DAISY_SEED, AUDIO_SR_96K);
  DAISY.SetAudioBlockSize(48);
  sample_rate_hz = DAISY.get_samplerate();

  if (kEnableEq)
  {
    ConfigurePeaking(bass_eq, sample_rate_hz, 120.0f, 0.7f, 6.0f);
    ConfigurePeaking(mid_eq, sample_rate_hz, 1500.0f, 1.0f, 4.0f);
  }
  ConfigureHighpass(post_hpf_l, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf_r, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf2_l, sample_rate_hz, 19000.0f, 0.70710678f);
  ConfigureHighpass(post_hpf2_r, sample_rate_hz, 19000.0f, 0.70710678f);

  DAISY.begin(AudioCallback);
}

void loop() {}