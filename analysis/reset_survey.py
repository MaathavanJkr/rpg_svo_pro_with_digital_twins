#!/usr/bin/env python3
"""Survey SVO full resets per run from frame_events.csv (log_frame_events:=true).

A full reset = stage kRelocalization (3) -> kPaused (0) (relocalisation failed, resetAll/resetBackend),
followed by re-initialisation (0 -> 1 -> 2). Times are relative to the first image of the run.
Classification (decision rules, 2026-10-24):
  (i)   no resets                       -> include
  (ii)  all resets within the first EARLY_S seconds -> include with the sequence start shifted to the
        final re-initialisation (first return to tracking after the last reset); D1(B) applied there
  (iii) any reset after EARLY_S         -> exclude from the main table (limitation)
Usage: python analysis/reset_survey.py runs/<id> ...   (prints a table, writes runs/reset_survey.json)
"""
import json
import os
import sys
import numpy as np

EARLY_S = 15.0


def survey(run):
    d = np.genfromtxt(os.path.join(run, "frame_events.csv"), delimiter=",", names=True, dtype=np.int64)
    t = (d["stamp_ns"] - d["stamp_ns"][0]) * 1e-9
    st = d["stage"]
    first_track = float(t[np.argmax(st == 2)]) if (st == 2).any() else None
    resets = t[1:][(st[:-1] == 3) & (st[1:] == 0)]
    lost = int(np.sum((st[:-1] == 2) & (st[1:] == 3)))
    reloc_ok = int(np.sum((st[:-1] == 3) & (st[1:] == 2)))
    final_reinit = None
    if len(resets):
        k = np.flatnonzero((t > resets[-1]) & (st == 2))
        final_reinit = float(t[k[0]]) if len(k) else None
    if len(resets) == 0:
        cls = "(i) include"
    elif resets.max() <= EARLY_S:
        cls = f"(ii) include, start shifted to {final_reinit:.1f} s"
    else:
        cls = "(iii) exclude (mid-flight reset)"
    return dict(run=os.path.basename(run.rstrip("/")), duration_s=float(t[-1]), first_tracking_s=first_track,
                tracking_lost=lost, relocalised=reloc_ok, n_resets=int(len(resets)),
                reset_times_s=[round(float(x), 1) for x in resets], all_resets_early=bool(len(resets) == 0 or resets.max() <= EARLY_S),
                final_reinit_tracking_s=final_reinit, classification=cls)


if __name__ == "__main__":
    out = [survey(r) for r in sys.argv[1:]]
    for s in out:
        print(f"{s['run']:42s} resets {s['n_resets']} at {s['reset_times_s']}  lost {s['tracking_lost']}, reloc ok {s['relocalised']}  -> {s['classification']}")
    json.dump(out, open("runs/reset_survey.json", "w"), indent=2)
