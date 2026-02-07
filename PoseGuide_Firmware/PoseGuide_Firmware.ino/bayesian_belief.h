// ============================================================
// bayesian_belief.h — Bayesian Belief Updater
// Denise's module
//
// Computes a continuous slouch confidence score P(slouch | obs)
// using Bayes' rule, then applies exponential smoothing.
//
// The smoothing creates "implicit patience" — the belief can't
// spike instantly from a single noisy reading. Combined with the
// HMM's temporal filtering, this makes the dual-gated system
// very resistant to false positives.
//
// Input features match Albert's updated EMA output:
//   deviation  = ema_pitch - baseline_angle
//   abs_ema_gyro = fabs(ema_gyro)
// ============================================================

#ifndef BAYESIAN_BELIEF_H
#define BAYESIAN_BELIEF_H

#include <Arduino.h>
#include <math.h>

class BayesianBelief {
private:
  float belief;

  // Exponential smoothing factor.
  // belief_t = SMOOTH_ALPHA × belief_{t-1} + (1 - SMOOTH_ALPHA) × instant
  // At α=0.8 and 10Hz, effective time constant ≈ 0.5 seconds.
  // Takes ~5 consecutive high readings to ramp belief to confident range.
  static constexpr float SMOOTH_ALPHA = 0.8;

  // ====== LIKELIHOOD PARAMETERS ======
  // Derived from the same training data as the HMM.
  // These define P(feature | class) for binary Bayes (slouch vs upright).
  //
  // Angle likelihoods (Gaussian):
  //   Slouching: user tilted forward → large negative deviation
  //   Upright:   deviation near zero
  static constexpr float SLOUCH_ANGLE_MEAN  = -19.66;
  static constexpr float SLOUCH_ANGLE_VAR   =  40.63;
  static constexpr float UPRIGHT_ANGLE_MEAN =  -0.99;
  static constexpr float UPRIGHT_ANGLE_VAR  =  11.16;

  // Gyro likelihoods (Gaussian on abs EMA-filtered gyro):
  //   Both states have low gyro (stable posture), but upright
  //   is tighter. Transient states (high gyro) dilute both
  //   likelihoods roughly equally, which naturally suppresses
  //   Bayesian belief during movement — a nice property.
  static constexpr float SLOUCH_GYRO_MEAN   = 0.26;
  static constexpr float SLOUCH_GYRO_VAR    = 13.44;
  static constexpr float UPRIGHT_GYRO_MEAN  = 0.33;
  static constexpr float UPRIGHT_GYRO_VAR   = 6.00;

  // Prior: P(slouch) = 0.5 (uninformative)
  static constexpr float PRIOR_SLOUCH = 0.5;

  float gaussPDF(float x, float mean, float variance) {
    float diff = x - mean;
    float exponent = -(diff * diff) / (2.0 * variance);
    if (exponent < -30.0) exponent = -30.0;
    return (1.0 / sqrt(2.0 * PI * variance)) * exp(exponent);
  }

public:
  void begin() {
    belief = 0.0;
  }

  // ====== BAYESIAN UPDATE + EXPONENTIAL SMOOTHING ======
  //
  // Step 1 — Compute instantaneous posterior via Bayes' rule:
  //
  //   P(slouch | obs) = P(obs | slouch) × P(slouch)
  //                     ─────────────────────────────
  //                              P(obs)
  //
  //   Where P(obs) = P(obs|slouch)×P(slouch) + P(obs|upright)×P(upright)
  //
  //   Conditional independence assumption (Naive Bayes):
  //     P(obs | slouch) = P(angle | slouch) × P(gyro | slouch)
  //
  // Step 2 — Exponential smoothing for temporal patience:
  //
  //   belief_t = α × belief_{t-1} + (1-α) × instant_posterior
  //
  //   This means the belief "remembers" recent history. A brief
  //   slouch-like reading can't immediately spike belief to 1.0,
  //   and belief naturally decays when posture improves.
  void update(float deviation, float abs_ema_gyro) {
    float like_slouch  = gaussPDF(deviation,    SLOUCH_ANGLE_MEAN, SLOUCH_ANGLE_VAR)
                       * gaussPDF(abs_ema_gyro, SLOUCH_GYRO_MEAN,  SLOUCH_GYRO_VAR);

    float like_upright = gaussPDF(deviation,    UPRIGHT_ANGLE_MEAN, UPRIGHT_ANGLE_VAR)
                       * gaussPDF(abs_ema_gyro, UPRIGHT_GYRO_MEAN,  UPRIGHT_GYRO_VAR);

    float numerator   = like_slouch * PRIOR_SLOUCH;
    float denominator = numerator + like_upright * (1.0 - PRIOR_SLOUCH);

    float instant_belief = 0.0;
    if (denominator > 1e-30) {
      instant_belief = numerator / denominator;
    }

    belief = SMOOTH_ALPHA * belief + (1.0 - SMOOTH_ALPHA) * instant_belief;
  }

  float getBelief() {
    return belief;
  }
};

#endif