// ============================================================
// dual_gate_controller.h — Dual-Gating Adaptive Feedback Controller
//
// Core design: BOTH the Bayesian belief AND the HMM must
// independently agree before any vibration alert fires.
//
// New in this version: POSTURE HISTORY — a long-term memory
// that tracks the user's overall posture behavior over ~100
// seconds and modulates the belief activation threshold.
// Good posture history → higher threshold (more lenient).
// Poor posture history → lower threshold (more aggressive).
//
// Three timescales of adaptiveness:
//   1. EMA filter:      ~0.6s memory (sensor noise)
//   2. Bayesian belief:  ~0.5s memory (episode detection)
//   3. Posture history: ~100s memory (behavioral pattern)
//
// Includes:
//   - Transient suppression (no alerts during active movement)
//   - Escalating feedback (single → double pulse)
//   - Hysteretic reset (must sustain upright 3s to clear alert)
// ============================================================

#ifndef DUAL_GATE_CONTROLLER_H
#define DUAL_GATE_CONTROLLER_H

#include <Arduino.h>

class DualGateController {
private:
  // ====== GATE 2 & 3 THRESHOLDS (fixed) ======
  static constexpr float SLOUCH_THRESHOLD  = 0.70;  // Gate 2: HMM P(slouch)
  static constexpr float TRANSIENT_CEILING = 0.50;  // Gate 3: HMM P(transient)

  // ====== GATE 1: ADAPTIVE BELIEF THRESHOLD ======
  // Instead of a fixed 0.65, the belief threshold is modulated by
  // the posture history score. Range: [BASE, BASE + LENIENCY]
  //
  //   threshold_t = BASE + LENIENCY × (1 − h_t)
  //
  //   h_t ≈ 0 (excellent history): threshold = 0.55 + 0.25 = 0.80
  //   h_t ≈ 0.5 (mixed history):   threshold = 0.55 + 0.125 = 0.675
  //   h_t ≈ 1 (terrible history):   threshold = 0.55 + 0.00 = 0.55
  static constexpr float BASE_THRESHOLD    = 0.55;
  static constexpr float LENIENCY_RANGE    = 0.25;

  // ====== POSTURE HISTORY — Long-Term Behavioral Memory ======
  // A slow EMA that tracks the proportion of time spent slouching
  // over approximately the last 100 seconds.
  //
  //   h_t = α_h × h_{t-1} + (1 − α_h) × s_t
  //
  //   where s_t = 1 if belief > 0.50 (currently slouching), else 0
  //
  // α_h = 0.999: N_eff = 1/(1-0.999) = 1000 samples = 100s at 10Hz
  //
  // This timescale spans multiple slouch-correction cycles, capturing
  // the user's behavioral pattern rather than any single episode.
  // A single 3-second slouch shifts history by only ~0.03 — negligible.
  // Sustained poor behavior over minutes shifts it significantly.
  static constexpr float HISTORY_ALPHA           = 0.999;
  static constexpr float HISTORY_SLOUCH_INDICATOR = 0.50;

  float posture_history;            // Long-term posture score [0, 1]
  float adaptive_belief_threshold;  // Computed each tick from history

  // ====== RESET THRESHOLDS ======
  static constexpr float UPRIGHT_RESET_THRESH    = 0.70;
  static constexpr unsigned long UPRIGHT_RESET_DURATION_MS = 3000;
  static constexpr float TRANSIENT_RESET_THRESH  = 0.80;

  // ====== ESCALATION ======
  static constexpr unsigned long ESCALATION_DELAY_MS  = 15000;
  static constexpr unsigned long MIN_BUZZ_INTERVAL_MS = 5000;

  // ====== STATE ======
  bool   alert_active;
  bool   escalated;
  unsigned long alert_start_ms;
  unsigned long last_buzz_ms;

  unsigned long upright_start_ms;
  bool   upright_counting;

  int buzz_command;

  void resetAlert() {
    alert_active   = false;
    escalated      = false;
    alert_start_ms = 0;
    buzz_command   = 0;
  }

public:
  void begin() {
    alert_active     = false;
    escalated        = false;
    alert_start_ms   = 0;
    last_buzz_ms     = 0;
    upright_start_ms = 0;
    upright_counting = false;
    buzz_command     = 0;

    // Initialize history to neutral — no assumption about behavior
    posture_history          = 0.5;
    adaptive_belief_threshold = BASE_THRESHOLD + LENIENCY_RANGE * (1.0 - posture_history);
  }

  void update(float belief, float p_slouch, float p_transient,
              float p_upright, unsigned long now) {

    // ===== POSTURE HISTORY UPDATE (runs every tick, unconditionally) =====
    float slouch_indicator = (belief > HISTORY_SLOUCH_INDICATOR) ? 1.0 : 0.0;
    posture_history = HISTORY_ALPHA * posture_history
                    + (1.0 - HISTORY_ALPHA) * slouch_indicator;

    // Recompute adaptive threshold
    adaptive_belief_threshold = BASE_THRESHOLD + LENIENCY_RANGE * (1.0 - posture_history);

    // ===== PHASE 1: RESET =====
    if (p_transient > TRANSIENT_RESET_THRESH) {
      resetAlert();
      upright_counting = false;
      return;
    }

    if (p_upright > UPRIGHT_RESET_THRESH) {
      if (!upright_counting) {
        upright_counting = true;
        upright_start_ms = now;
      } else if (now - upright_start_ms >= UPRIGHT_RESET_DURATION_MS) {
        resetAlert();
        upright_counting = false;
        return;
      }
    } else {
      upright_counting = false;
    }

    // ===== PHASE 2: ACTIVATION (adaptive belief threshold) =====
    bool gates_pass = (belief      > adaptive_belief_threshold) &&
                      (p_slouch    > SLOUCH_THRESHOLD) &&
                      (p_transient < TRANSIENT_CEILING);

    if (gates_pass && !alert_active) {
      alert_active   = true;
      escalated      = false;
      alert_start_ms = now;
      last_buzz_ms   = now;
      buzz_command   = 1;
      return;
    }

    if (alert_active && gates_pass) {
      // ===== PHASE 3: ESCALATION =====
      if (!escalated && (now - alert_start_ms >= ESCALATION_DELAY_MS)) {
        escalated = true;
      }

      if (now - last_buzz_ms >= MIN_BUZZ_INTERVAL_MS) {
        last_buzz_ms = now;
        buzz_command = escalated ? 2 : 1;
      } else {
        buzz_command = 0;
      }
      return;
    }

    buzz_command = 0;
  }

  int getBuzzCommand() {
    return buzz_command;
  }

  float getPostureHistory() {
    return posture_history;
  }

  float getAdaptiveThreshold() {
    return adaptive_belief_threshold;
  }
};

#endif
