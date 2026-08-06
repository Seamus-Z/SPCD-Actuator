"""Snapshot diagnostics for the xtellar GUI /api/snap payload.

Computes the quantities that distinguish current-sense imbalance from
commutation-frame problems, and detects pole-pair / speed mismatches:

  - fundamental frequency of every channel (id/iq/i1/i2/i3), from the
    median positive-going zero-crossing spacing
  - amplitude of every channel and the i1:i2:i3 balance ratio
  - i1+i2+i3 residual (zero for a balanced real motor; a sinusoid at
    f_elec or 2*f_elec proves a sensing imbalance)
  - Id/Iq ripple frequency vs the phase-current frequency (ripple at
    1x/2x/3x f_elec identifies the mechanism)

Usage:
  python3 snap_analysis.py snap.json [pole_pairs=14] [omega_mech_rad_s=200]
"""

import json
import math
import sys

CH = ["id_a", "iq_a", "i1_a", "i2_a", "i3_a", "theta_mech_rad", "theta_elec_rad"]


def zero_cross_freq(x, hz, min_hz=5.0):
    """Fundamental frequency via median positive-going zero-crossing spacing."""
    n = len(x)
    if n < 8:
        return 0.0
    crossings = []
    for i in range(1, n):
        if x[i - 1] < 0.0 <= x[i]:
            crossings.append(i)
    if len(crossings) < 2:
        return 0.0
    gaps = [crossings[i + 1] - crossings[i] for i in range(len(crossings) - 1)]
    gaps.sort()
    med = gaps[len(gaps) // 2]
    if med <= 0:
        return 0.0
    f = hz / med
    return f if f >= min_hz else 0.0


def _wrap_pm(x):
    return ((x + math.pi) % (2.0 * math.pi)) - math.pi


def _ramp_freq(xs, hz):
    """Frequency of a wrapped-angle ramp: unwrap, least-squares slope of
    theta(t), divide by 2π. Robust to noise, no zero-crossing heuristics."""
    n = len(xs)
    if n < 8:
        return 0.0
    u = _unwrap_series(xs)
    # least squares slope of u vs index i
    sx = sum(range(n))
    sy = sum(u)
    sxx = sum(i * i for i in range(n))
    sxy = sum(i * u[i] for i in range(n))
    denom = n * sxx - sx * sx
    if denom == 0:
        return 0.0
    slope = (n * sxy - sx * sy) / denom  # rad / sample
    return slope * hz / (2.0 * math.pi)


def _unwrap_series(xs):
    out = []
    carry = 0.0
    prev = None
    for x in xs:
        if prev is None:
            out.append(x)
        else:
            d = x - prev
            if d > math.pi:
                carry -= 2.0 * math.pi
            elif d < -math.pi:
                carry += 2.0 * math.pi
            out.append(x + carry)
        prev = x
    return out


def pkpk(x):
    return (max(x) - min(x)) / 2.0 if x else 0.0


def analyze(snap, pole_pairs=14.0, omega_mech_rad_s=0.0):
    hz = float(snap.get("sample_hz") or 1)
    series = snap.get("series") or {}
    out = {"sample_hz": hz, "n": len(series.get("i1_a", []))}

    freq = {}
    amp = {}
    for k in CH:
        v = list(series.get(k, []))
        freq[k] = zero_cross_freq(v, hz)
        amp[k] = pkpk(v)

    f_phase = max(freq.get("i1_a", 0.0), freq.get("i2_a", 0.0),
                  freq.get("i3_a", 0.0))
    f_dq = max(freq.get("id_a", 0.0), freq.get("iq_a", 0.0))

    out["freq_hz"] = {k: round(freq[k], 1) for k in CH}
    out["amp_A"] = {k: round(amp[k], 3) for k in CH}

    # Encoder-derived mechanical/electrical frequency and the true ratio of
    # electrical periods per mechanical revolution (= effective pole pairs).
    # theta_* are mrad int16 that wrap; unwrap by accumulating wrapped deltas.
    tm = series.get("theta_mech_rad", [])
    te = series.get("theta_elec_rad", [])
    if len(tm) > 4 and len(te) > 4:
        dm = de = 0.0
        for i in range(1, len(tm)):
            dm += _wrap_pm(tm[i] - tm[i - 1])
            de += _wrap_pm(te[i] - te[i - 1])
        # Theta is a sawtooth that wraps every 2π; unwrap it, then the zero
        # crossings of (theta - linear_fit) count full rotations.
        tm_u = _unwrap_series(tm)
        te_u = _unwrap_series(te)
        f_mech = _ramp_freq(tm_u, hz)
        f_elec_snap = _ramp_freq(te_u, hz)
        out["f_elec_snap_hz"] = round(f_elec_snap, 2)
        out["f_mech_enc_hz"] = round(f_mech, 2)
        out["theta_dbg"] = {
            "n": len(tm),
            "d_mech_rad": round(dm, 3),
            "d_elec_rad": round(de, 3),
            "mech_range": [round(min(tm[:512]), 3), round(max(tm[:512]), 3)],
            "elec_range": [round(min(te[:512]), 3), round(max(te[:512]), 3)],
            "mech_first16": [round(x, 3) for x in tm[:16]],
            "elec_first16": [round(x, 3) for x in te[:16]],
        }
        if abs(dm) > 1e-6:
            out["pp_measured"] = round(de / dm, 2)

    # Phase-current amplitude balance.
    amps = [amp["i1_a"], amp["i2_a"], amp["i3_a"]]
    if amps and max(amps) > 1e-4:
        out["balance_minmax_ratio"] = round(min(amps) / max(amps), 3)
    else:
        out["balance_minmax_ratio"] = 0.0

    # i1+i2+i3 residual: 0 for a balanced 3-phase set.
    n = out["n"]
    resid = [series["i1_a"][i] + series["i2_a"][i] + series["i3_a"][i]
             for i in range(n)]
    out["sum_pkpk_A"] = round(pkpk(resid), 3)
    out["sum_freq_hz"] = round(zero_cross_freq(resid, hz), 1)

    # Ratios that identify the mechanism.
    if f_phase > 0.0:
        out["dq_over_phase_ratio"] = round(f_dq / f_phase, 2)
        if omega_mech_rad_s > 0.0 and pole_pairs > 0.0:
            f_elec_expected = omega_mech_rad_s * pole_pairs / (2.0 * math.pi)
            out["f_elec_expected_hz"] = round(f_elec_expected, 1)
            out["f_phase_over_f_expected"] = round(f_phase / f_elec_expected, 3)
    return out


def _fmt(out):
    lines = [f"sample_hz={out['sample_hz']:.0f} n={out['n']}"]
    lines.append("freq_hz: " + ", ".join(
        f"{k}={out['freq_hz'][k]}" for k in CH))
    lines.append("amp_A:   " + ", ".join(
        f"{k}={out['amp_A'][k]}" for k in CH))
    lines.append(f"balance(min/max)={out['balance_minmax_ratio']}")
    lines.append(f"i1+i2+i3 pkpk={out['sum_pkpk_A']}A "
                 f"f={out['sum_freq_hz']}Hz")
    if "dq_over_phase_ratio" in out:
        lines.append(f"f(id/iq)/f(phase)={out['dq_over_phase_ratio']}")
    if "f_elec_expected_hz" in out:
        lines.append(
            f"f_phase={out['freq_hz']['i1_a']}Hz vs "
            f"expected f_elec={out['f_elec_expected_hz']}Hz "
            f"(ratio {out['f_phase_over_f_expected']})")
    if "f_mech_enc_hz" in out:
        lines.append(f"f_mech(encoder)={out['f_mech_enc_hz']}Hz")
    if "theta_dbg" in out:
        d = out["theta_dbg"]
        lines.append(f"Δθmech={d['d_mech_rad']}rad Δθelec={d['d_elec_rad']}rad")
        lines.append(f"θmech[0..512]range={d['mech_range']} θelec[0..512]range={d['elec_range']}")
        lines.append(f"θmech[0:16]={d['mech_first16']}")
        lines.append(f"θelec[0:16]={d['elec_first16']}")
    if "pp_measured" in out:
        lines.append(f"pp_measured(Δθelec/Δθmech)={out['pp_measured']} "
                     f"(expect 14)")
    lines.append("")
    lines.append("read as:")
    lines.append("  dq_over_phase≈1  -> offset/1x;  ≈2 -> gain imbalance/2x")
    lines.append("  f_phase/f_expected≈1 -> pole_pairs ok;  >1.3 -> pole count "
                 "or encoder scaling wrong")
    lines.append("  i1+i2+i3 pkpk >> 0 -> sensing imbalance (not a real motor)")
    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        print("usage: snap_analysis.py snap.json [pole_pairs] [omega_mech]")
        return 1
    with open(sys.argv[1]) as f:
        snap = json.load(f)
    pp = float(sys.argv[2]) if len(sys.argv) > 2 else 14.0
    w = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    out = analyze(snap, pole_pairs=pp, omega_mech_rad_s=w)
    print(json.dumps(out, indent=2))
    print(_fmt(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
