#include <DaisyDuino.h>
#include <cmath>

static float tri_phase       = 0.0f;
static float carrier_phase   = 0.0f;
static float sample_rate_hz  = 48000.0f;

static constexpr float kTriHz       = 1000.0f;
static constexpr float kCarrierHz   = 80000.0f;

// Apply 0.8 gain reduction globally (per your earlier request).
static constexpr float kTriGain     = 0.4f * 0.8f;
static constexpr float kOutGain     = 0.6f * 0.5f; // extra headroom: input(±1) * sin(±1)

static constexpr int   kInputChannel = 0; // only keep one input active

// Input (ADC) is capped at 96 kHz; we run output at 192 kHz for an 80 kHz carrier.
static constexpr float kInputMaxSrHz = 96000.0f;
static float           input_sr_hz   = 48000.0f;
static int             hold_factor   = 1;
static int             in_hold_count = 0;
static float           filtered_held = 0.0f;

struct SimpleBiquad
{
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float z1 = 0.0f, z2 = 0.0f;

  inline float Process(float x)
  {
    const float y = (b0 * x) + z1;
    z1            = (b1 * x) - (a1 * y) + z2;
    z2            = (b2 * x) - (a2 * y);
    return y;
  }
};

static SimpleBiquad hp10;
static SimpleBiquad lp20k;

static void ConfigureBiquadLowpass(SimpleBiquad& bq, float fs, float fc, float q)
{
  const float w0    = 2.0f * 3.14159265358979323846f * (fc / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);

  float b0 = (1.0f - cosw0) * 0.5f;
  float b1 = 1.0f - cosw0;
  float b2 = (1.0f - cosw0) * 0.5f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosw0;
  float a2 = 1.0f - alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

static void ConfigureBiquadHighpass(SimpleBiquad& bq, float fs, float fc, float q)
{
  const float w0    = 2.0f * 3.14159265358979323846f * (fc / fs);
  const float cosw0 = cosf(w0);
  const float sinw0 = sinf(w0);
  const float alpha = sinw0 / (2.0f * q);

  float b0 = (1.0f + cosw0) * 0.5f;
  float b1 = -(1.0f + cosw0);
  float b2 = (1.0f + cosw0) * 0.5f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosw0;
  float a2 = 1.0f - alpha;

  bq.b0 = b0 / a0;
  bq.b1 = b1 / a0;
  bq.b2 = b2 / a0;
  bq.a1 = a1 / a0;
  bq.a2 = a2 / a0;
  bq.z1 = 0.0f;
  bq.z2 = 0.0f;
}

// Output:
// - out[0]: 1 kHz sine oscillator
// - out[1]: (HP@10Hz -> LP@20kHz) input, multiplied by 80 kHz sine carrier

void AudioCallback(float** in, float** out, size_t size)
{
  const bool have_in = (in != nullptr) && (in[kInputChannel] != nullptr);

  const float tri_phase_inc     = kTriHz / sample_rate_hz;
  const float carrier_phase_inc = (2.0f * 3.14159265358979323846f * kCarrierHz) / sample_rate_hz;

  for (size_t i = 0; i < size; i++)
  {
    // Update the held input sample at the (assumed) input rate.
    if (have_in)
    {
      if (in_hold_count <= 0)
      {
        in_hold_count = hold_factor;
        float x       = in[kInputChannel][i];
        x             = hp10.Process(x);
        x             = lp20k.Process(x);
        filtered_held = x;
      }
      in_hold_count--;
    }
    else
    {
      filtered_held = 0.0f;
    }

    // Sine in [-1, 1]
    const float test_sine = sinf(2.0f * 3.14159265358979323846f * tri_phase);

    tri_phase += tri_phase_inc;
    if (tri_phase >= 1.0f)
      tri_phase -= 1.0f;

    const float carrier = sinf(carrier_phase);
    carrier_phase += carrier_phase_inc;
    if (carrier_phase > 2.0f * 3.14159265358979323846f)
      carrier_phase -= 2.0f * 3.14159265358979323846f;

    out[0][i] = kTriGain * test_sine;
    out[1][i] = kOutGain * (filtered_held * carrier);
  }
}

void setup()
{
  // Output needs 192 kHz to generate an 80 kHz carrier cleanly.
  DAISY.init(DAISY_SEED, AUDIO_SR_192K);
  DAISY.SetAudioBlockSize(48);
  sample_rate_hz = DAISY.get_samplerate();

  input_sr_hz = (sample_rate_hz > kInputMaxSrHz) ? kInputMaxSrHz : sample_rate_hz;
  const float ratio = sample_rate_hz / input_sr_hz;
  const int   hf    = (int)lroundf(ratio);
  hold_factor       = (fabsf(ratio - (float)hf) < 0.01f && hf >= 1) ? hf : 1;
  in_hold_count     = 0;
  filtered_held     = 0.0f;

  // 2nd-order Butterworth-ish filters (Q ~= 0.707) at the *input* rate.
  ConfigureBiquadHighpass(hp10, input_sr_hz, 10.0f, 0.70710678f);
  ConfigureBiquadLowpass(lp20k, input_sr_hz, 20000.0f, 0.70710678f);

  DAISY.begin(AudioCallback);
}

void loop() {}