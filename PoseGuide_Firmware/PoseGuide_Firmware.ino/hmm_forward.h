// ============================================================
// hmm_forward.h — HMM 3-State Forward Algorithm (Online)
// Denise's module
//
// Runs the forward algorithm at each 10Hz tick using pre-trained
// parameters. Input features match Albert's updated EMA output:
//   obs[0] = deviation from baseline (EMA-filtered angle - baseline)
//   obs[1] = abs(EMA-filtered gyro rate)
//
// Trained on ~146 minutes of labeled data from 9 participants.
// ============================================================

#ifndef HMM_FORWARD_H
#define HMM_FORWARD_H

#include <Arduino.h>
#include <math.h>

#define HMM_UPRIGHT   0
#define HMM_SLOUCHING 1
#define HMM_TRANSIENT 2
#define HMM_N_STATES  3
#define HMM_N_OBS     2   // [deviation, abs_ema_gyro]

class HMMForward {
private:
  // ====== TRAINED PARAMETERS (from your JSON model) ======

  // Emission means: μ[state][feature]
  // These define the "center" of each state's observation cluster.
  //   State 0 (Upright):   small deviation, low gyro
  //   State 1 (Slouching):  large negative deviation, low gyro  
  //   State 2 (Transient):  large deviation, HIGH gyro (movement)
  float means[HMM_N_STATES][HMM_N_OBS] = {
    { -0.9884,  0.3333 },   // Upright
    { -19.6600, 0.2582 },   // Slouching
    { -18.5924, 5.4577 }    // Transient
  };

  // Emission variances: σ²[state][feature]
  // How "spread out" observations are within each state.
  // Note: Transient has massive variance — catches all kinds of movements.
  float vars[HMM_N_STATES][HMM_N_OBS] = {
    {  11.1647,   5.9958 },   // Upright — tight cluster
    {  40.6328,  13.4401 },   // Slouching — moderate spread
    { 294.0740, 340.4464 }    // Transient — very wide (diverse movements)
  };

  // Transition matrix: A[from][to]
  // Each row sums to 1.0. Strongly diagonal-dominant (states are "sticky").
  // Key insight: trans[2][1] = 0.035 — transient often resolves to slouching,
  //   not upright. This matches real behavior: people shift, then settle back
  //   into their slouch.
  float trans[HMM_N_STATES][HMM_N_STATES] = {
    { 0.99904, 0.00088, 0.00008 },   // Upright → almost always stays upright
    { 0.00049, 0.99786, 0.00166 },   // Slouching → very sticky
    { 0.00188, 0.03511, 0.96301 }    // Transient → often becomes slouching
  };

  // Forward variable: α[state]
  // After each update, α[j] = P(state_t = j | all observations so far)
  float alpha[HMM_N_STATES];

  bool initialized;

  // ====== GAUSSIAN EMISSION PROBABILITY ======
  // Computes P(observation | state) assuming diagonal covariance
  // (features conditionally independent given state).
  //
  // For each feature dimension d:
  //   p_d = (1 / √(2π σ²)) × exp(-(obs_d - μ_d)² / (2σ²_d))
  //
  // Joint probability (diagonal = independent):
  //   P(obs | state) = ∏_d p_d
  //
  // Exponent clamped to [-30, 0] to prevent underflow on 32-bit float.
  float emission(int state, float obs[HMM_N_OBS]) {
    float prob = 1.0;
    for (int d = 0; d < HMM_N_OBS; d++) {
      float diff = obs[d] - means[state][d];
      float variance = vars[state][d];
      float exponent = -(diff * diff) / (2.0 * variance);
      if (exponent < -30.0) exponent = -30.0;
      float p = (1.0 / sqrt(2.0 * PI * variance)) * exp(exponent);
      prob *= p;
    }
    if (prob < 1e-30) prob = 1e-30;
    return prob;
  }

  // Normalize α so probabilities sum to 1.
  // Without this, α values shrink exponentially and underflow
  // to 0 within ~50 steps (~5 seconds at 10Hz) on 32-bit float.
  void normalize() {
    float sum = 0;
    for (int i = 0; i < HMM_N_STATES; i++) sum += alpha[i];
    if (sum > 0) {
      for (int i = 0; i < HMM_N_STATES; i++) alpha[i] /= sum;
    } else {
      for (int i = 0; i < HMM_N_STATES; i++) alpha[i] = 1.0 / HMM_N_STATES;
    }
  }

public:
  void begin() {
    for (int i = 0; i < HMM_N_STATES; i++) alpha[i] = 1.0 / HMM_N_STATES;
    initialized = false;
  }

  // ====== FORWARD ALGORITHM — SINGLE STEP ======
  // Called once per sample (every 100ms at 10Hz).
  //
  // Standard HMM forward recursion:
  //
  //   First observation (t=1):
  //     α₁(j) = π(j) × B(j, o₁)
  //     where π(j) = 1/N (uniform prior)
  //
  //   Subsequent observations (t>1):
  //     α_t(j) = [ Σᵢ α_{t-1}(i) × A(i,j) ] × B(j, o_t)
  //
  //   Then normalize: α_t(j) ← α_t(j) / Σⱼ α_t(j)
  //
  // After normalization, α_t(j) directly gives us:
  //   P(state_t = j | o₁, o₂, ..., o_t)
  //
  // This is "filtering" — the probability of the current state given
  // all past observations. Unlike Viterbi (which finds the best sequence),
  // filtering gives us a real-time answer suitable for embedded control.
  //
  // Parameters:
  //   deviation: ema_pitch - baseline_angle (degrees from upright)
  //   abs_ema_gyro: fabs(ema_gyro), the EMA-filtered absolute angular velocity
  void update(float deviation, float abs_ema_gyro) {
    float obs[HMM_N_OBS] = { deviation, abs_ema_gyro };

    if (!initialized) {
      for (int j = 0; j < HMM_N_STATES; j++) {
        alpha[j] = (1.0 / HMM_N_STATES) * emission(j, obs);
      }
      normalize();
      initialized = true;
      return;
    }

    float new_alpha[HMM_N_STATES];
    for (int j = 0; j < HMM_N_STATES; j++) {
      float sum = 0;
      for (int i = 0; i < HMM_N_STATES; i++) {
        sum += alpha[i] * trans[i][j];
      }
      new_alpha[j] = sum * emission(j, obs);
    }

    for (int j = 0; j < HMM_N_STATES; j++) alpha[j] = new_alpha[j];
    normalize();
  }

  float getProb(int state) {
    if (state >= 0 && state < HMM_N_STATES) return alpha[state];
    return 0.0;
  }
};

#endif