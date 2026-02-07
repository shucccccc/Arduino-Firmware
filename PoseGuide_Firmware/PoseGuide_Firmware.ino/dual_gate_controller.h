// ============================================================
// dual_gate_controller.h — Dual-Gating Adaptive Feedback Controller
//
// BOTH the Bayesian belief AND the HMM must
// independently agree that the user is slouching before any
// vibration alert fires. This dramatically reduces false alerts
// compared to single-classifier systems, because each system
// has different failure modes:
//   - Bayesian: can spike from a single noisy reading
//     (mitigated by smoothing, but still possible)
//   - HMM: can get "stuck" in a state due to transition stickiness
//     (mitigated by reset logic)
//   - Requiring agreement means both must fail simultaneously
//     for a false alert to occur
//
// Also includes:
//   - Transient suppression (no alerts during active movement)
//   - Escalating feedback (single → double pulse)
//   - Hysteretic reset (must sustain upright 3s to clear alert)
// ============================================================

#ifndef DUAL_GATE_CONTROLLER_H
#define DUAL_GATE_CONTROLLER_H

#include <Arduino.h>

class DualGateController {
private:
  // ====== ACTIVATION THRESHOLDS (all must pass) ======
  static constexpr float BELIEF_THRESHOLD  = 0.65;  // Gate 1: Bayesian
  static constexpr float SLOUCH_THRESHOLD  = 0.70;  // Gate 2: HMM P(slouch)
  static constexpr float TRANSIENT_CEILING = 0.50;  // Gate 3: HMM P(transient)

  // ====== RESET THRESHOLDS ======
  static constexpr float UPRIGHT_RESET_THRESH    = 0.70;
  static constexpr unsigned long UPRIGHT_RESET_DURATION_MS = 3000;  // 3s sustained
  static constexpr float TRANSIENT_RESET_THRESH  = 0.80;

  // ====== ESCALATION ======
  static constexpr unsigned long ESCALATION_DELAY_MS  = 15000;  // 15s
  static constexpr unsigned long MIN_BUZZ_INTERVAL_MS = 5000;   // 5s between buzzes

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
  }

  // ====== DUAL-GATING DECISION LOGIC ======
  //
  // Evaluated every 100ms. Three phases checked in order:
  //
  // PHASE 1 — RESET CHECK
  //   If P(TRANSIENT) > 0.80 → immediate reset.
  //     User is actively moving; any alert would be false.
  //   If P(UPRIGHT) > 0.70 for 3 continuous seconds → reset.
  //     User has corrected posture. The 3s requirement prevents
  //     brief upright moments during fidgeting from clearing
  //     a legitimate alert.
  //
  // PHASE 2 — ACTIVATION CHECK (dual gate)
  //   ALL three conditions must hold simultaneously:
  //     Gate 1: belief > 0.65 (Bayesian says "likely slouching")
  //     Gate 2: P(SLOUCHING) > 0.70 (HMM says "in slouch state")
  //     Gate 3: P(TRANSIENT) < 0.50 (not likely moving)
  //
  // PHASE 3 — ESCALATION CHECK
  //   If alert has been active > 15 seconds → escalate to double pulse.
  //   Buzzes are spaced at least 5 seconds apart to prevent habituation.
  void update(float belief, float p_slouch, float p_transient,
              float p_upright, unsigned long now) {

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

    // ===== PHASE 2: ACTIVATION =====
    bool gates_pass = (belief    > BELIEF_THRESHOLD) &&
                      (p_slouch  > SLOUCH_THRESHOLD) &&
                      (p_transient < TRANSIENT_CEILING);

    if (gates_pass && !alert_active) {
      alert_active   = true;
      escalated      = false;
      alert_start_ms = now;
      last_buzz_ms   = now;
      buzz_command   = 1;   // Single pulse
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
        buzz_command = 0;   // Waiting between buzzes
      }
      return;
    }

    // Gates not passing but alert still active — hold alert state
    // (don't reset until explicit reset conditions met)
    // This prevents alert flickering on the boundary
    buzz_command = 0;
  }

  int getBuzzCommand() {
    return buzz_command;
  }
};

#endif