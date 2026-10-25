#!/usr/bin/env python3
"""Phase 10 / D4: two-factor calibration of the Hessian information (definition in docs/phase10_d4_realism.md,
recorded before this script was run). Uses the step 1 per-KF outputs (runs/phase10_d4/realism_kf.csv).

information_cal = S H S, S = diag(1/k_p I3, 1/k_r I3) ([pos, rot]) -> Sigma_cal = S^-1 Sigma S^-1:
  position block x k_p^2 (NEES / k_p^2), rotation std x k_r.
k_p^2 = mean Hessian position NEES on V1_01 / 3; k_r = empirical per-KF yaw noise on V1_01 / median implied yaw std
on V1_01 (empirical yaw noise: median over runs of std(diff(correction yaw)) / sqrt 2). Per method, V1_01 ONLY.
Sensitivity only (not an arm): k_p^2 from the median NEES (/ 2.366, the chi^2_3 median).
Usage (venv): python analysis/phase10_d4_calibrate.py  ->  runs/phase10_d4/calibration.json
"""
import csv
import json
import os
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.path.join(REPO, "runs/phase10_d4")
SEQS = ["V1_01_easy", "V1_02_medium", "V2_02_medium"]
CHI2_3_MEDIAN = 2.365973884375338
CHI2_3_95 = 7.814727903251178


def main():
    kf = list(csv.DictReader(open(os.path.join(D, "realism_kf.csv"))))
    res = json.load(open(os.path.join(D, "realism.json")))["results"]
    out = dict(definition="docs/phase10_d4_realism.md, 'Calibrated Hessian: definition' (recorded 2026-10-25T17:45:23+05:30)")
    for m in ("P2P", "DF"):
        def sel(seq):
            return [k for k in kf if k["method"] == m and k["seq"] == seq and k.get("hess_nees") not in ("", None)]
        v1 = sel("V1_01_easy")
        nees_v1 = np.array([float(k["hess_nees"]) for k in v1])
        kp2 = float(nees_v1.mean() / 3)
        emp_yaw_v1 = res[f"{m}/V1_01_easy"]["empirical_yaw_noise_deg"]
        med_sig_yaw_v1 = float(np.median([float(k["hess_sig_yaw_deg"]) for k in v1]))
        kr = emp_yaw_v1 / med_sig_yaw_v1
        e = dict(k_p=float(np.sqrt(kp2)), k_p_squared=kp2, k_r=float(kr), k_r_squared=float(kr ** 2),
                 fit=dict(mean_nees_v1_01=float(nees_v1.mean()), empirical_yaw_noise_v1_01_deg=emp_yaw_v1,
                          median_implied_yaw_v1_01_deg=med_sig_yaw_v1))
        for seq in SEQS:
            s = sel(seq)
            nees = np.array([float(k["hess_nees"]) for k in s]) / kp2
            sig_pos = float(np.sqrt(np.mean(np.array([float(k["hess_sig_pos_rms"]) for k in s]) ** 2))) * np.sqrt(kp2)
            sig_yaw = float(np.median([float(k["hess_sig_yaw_deg"]) for k in s])) * kr
            r = res[f"{m}/{seq}"]
            e[seq] = dict(in_sample=seq == "V1_01_easy", n=len(s), nees_mean_over_3=float(nees.mean() / 3),
                          nees_median=float(np.median(nees)), frac_nees_below_chi2_95=float(np.mean(nees < CHI2_3_95)),
                          implied_pos_rms_per_axis_m=sig_pos, ratio_pos=sig_pos / r["empirical_pos_rms_per_axis_m"],
                          implied_yaw_median_deg=sig_yaw, ratio_yaw=sig_yaw / r["empirical_yaw_noise_deg"])
        kp2_med = float(np.median(nees_v1) / CHI2_3_MEDIAN)
        e["sensitivity_median_based_k_p"] = dict(k_p_squared=kp2_med, k_p=float(np.sqrt(kp2_med)),
                                                 nees_mean_over_3={seq: float(np.mean([float(k["hess_nees"]) for k in sel(seq)]) / 3 / kp2_med) for seq in SEQS},
                                                 note="offline sensitivity only; not an online arm")
        out[m] = e
    json.dump(out, open(os.path.join(D, "calibration.json"), "w"), indent=2)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
