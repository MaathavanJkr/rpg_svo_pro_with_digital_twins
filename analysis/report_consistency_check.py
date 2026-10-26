#!/usr/bin/env python3
"""Consistency check of the report drafts (docs/report/*.md) against the aggregate / JSON files they cite.

For every line (a table row inherits the sources cited in its table's header line, caption or the line before the table),
the sources are: JSON/CSV files named in the line (paths in backticks or brackets) plus, for every cited project document
(docs/<name>.md), the aggregate files listed in DOC_SOURCES below. Every number in the line is then checked for PRESENCE among
the numeric values of those files (all numeric leaves of the JSON; numeric cells of CSV), at the printed precision
(|round(value, d) - printed| <= 0.5 * 10^-d, plus 1e-9), also accepting the value x 1000 (m -> mm), x 100 (fraction -> %),
its absolute value, and |value| for "+/-" ranges. Numbers that are structural (years, section numbers, sequence ids, counts
written in the method names, voxel sizes in file names) are skipped by simple rules.
Limitations (stated): presence, not identity: a printed number that matches some value in the cited files is not proven to
be the right field; numbers derived in the text (ratios, sums, rounded "about" values) appear as unmatched and need a manual look.
Usage (venv): python analysis/report_consistency_check.py [docs/report/results.md ...]
Output: runs/report_consistency.json and a printed list of unmatched numbers per line.
"""
import csv
import glob
import json
import math
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC_SOURCES = {
    "phase9_online.md": ["runs/phase9/aggregate.json"],
    "phase10_throughput.md": ["runs/phase10_tc/aggregate.json", "runs/phase10_tc/exploratory.json", "runs/phase10_tc/exploratory_heading.json", "runs/phase10_tc/m2fast_equivalence.json"],
    "phase10_replication.md": ["runs/phase10_rep/aggregate.json", "runs/phase9/aggregate.json", "runs/phase10_tc/aggregate.json", "runs/phase10_d4/aggregate.json"],
    "phase10_d4_realism.md": ["runs/phase10_d4/realism.json", "runs/phase10_d4/calibration.json", "runs/phase10_d4/calibration_mapframe_superseded.json"],
    "phase10_d4_ablation.md": ["runs/phase10_d4/aggregate.json", "runs/phase10_d4/outliers.json", "runs/phase10_d4/mechanism.json"],
    "phase8_cpp.md": ["runs/phase8/equivalence.json", "runs/phase8/timing.json", "runs/phase8/node_check.json"],
    "phase7_offline.md": ["runs/phase7/study/report.json"],
    "phase10_align_results.md": ["runs/phase10_align/aggregate.json", "runs/phase10_align/threshold.json"],
    "phase10_h4_memory.md": ["runs/phase10_h4/memory.json"],
    "PLAN.md": ["runs/phase10_align/threshold.json"],
}
NUM = re.compile(r"(?<![\w.])[−\-+]?\d+(?:\.\d+)?(?:e[−\-+]?\d+)?(?![\w])")


def leaves(x, out):
    if isinstance(x, dict):
        for v in x.values():
            leaves(v, out)
    elif isinstance(x, list):
        for v in x:
            leaves(v, out)
    elif isinstance(x, bool):
        return
    elif isinstance(x, (int, float)) and math.isfinite(x):
        out.append(float(x))


def load_values(path, cache={}):
    if path in cache:
        return cache[path]
    vals = []
    full = os.path.join(REPO, path)
    if os.path.exists(full):
        if path.endswith(".json"):
            leaves(json.load(open(full)), vals)
        elif path.endswith(".csv"):
            for row in csv.reader(open(full)):
                for c in row:
                    try:
                        vals.append(float(c))
                    except ValueError:
                        pass
    cache[path] = vals
    return vals


def sources_in(text):
    src = set(re.findall(r"(runs/[\w./*-]+\.(?:json|csv))", text))
    for d in re.findall(r"(?:docs/)?([\w]+\.md)", text):
        src.update(DOC_SOURCES.get(d, []))
    return {s for s in src if "*" not in s}


def matches(printed, vals):
    s = printed.replace("−", "-")
    x = float(s)
    d = len(s.split(".")[1].split("e")[0]) if "." in s and "e" not in s.split(".")[0] else 0
    if "e" in s.lower():
        return any(abs(v - x) <= abs(x) * 0.051 for v in vals) or any(abs(abs(v) - abs(x)) <= abs(x) * 0.051 for v in vals)
    tol = 0.5 * 10 ** (-d) + 1e-9
    for v in vals:
        for c in (v, v * 1000, v * 100, abs(v), abs(v) * 1000, abs(v) * 100):
            if abs(c - x) <= tol or abs(c - abs(x)) <= tol:
                return True
    return False


def skip(tok, line):
    t = tok.replace("−", "-").lstrip("+-")
    if re.fullmatch(r"20\d\d", t):
        return True
    if re.search(r"V\d_0?" + re.escape(t) + r"\b", line) or re.search(r"\bH" + re.escape(t) + r"\b", line) or re.search(r"\bM" + re.escape(t) + r"\b", line):
        return True
    return False


def check(md):
    lines = open(md).read().splitlines()
    report = []
    table_src = set()
    for i, line in enumerate(lines):
        if not line.strip().startswith("|"):
            table_src = sources_in(line) | sources_in(lines[i - 1] if i else "")
        src = sources_in(line) | (table_src if line.strip().startswith("|") else set())
        if not src:
            continue
        vals = [v for s in src for v in load_values(s)]
        clean = re.sub(r"`[^`]*`", "", line)            # ignore paths / code
        clean = re.sub(r"\([^)]*\.md[^)]*\)", "", clean)  # ignore source brackets
        clean = re.sub(r"(\d)[ \u202f\u2009](\d{3})\b", r"\1\2", clean)  # thousands separators: "6 658" -> "6658"
        clean = re.sub(r"(?:Phases?|§|Chapter|Table|Fig\.?|Figure)\s*[\d.–,\s]+", " ", clean)  # structural references
        toks = [t for t in NUM.findall(clean) if not skip(t, clean)]
        bad = [t for t in toks if not matches(t, vals)]
        report.append(dict(line=i + 1, n_numbers=len(toks), unmatched=bad, sources=sorted(src), missing_sources=[s for s in src if not os.path.exists(os.path.join(REPO, s))]))
    return report


def main():
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(REPO, "docs/report/*.md")))
    out = {}
    for f in files:
        r = check(f)
        out[os.path.relpath(f, REPO)] = r
        n = sum(x["n_numbers"] for x in r)
        u = sum(len(x["unmatched"]) for x in r)
        print(f"== {os.path.relpath(f, REPO)}: {len(r)} lines with sources, {n} numbers checked, {u} unmatched")
        for x in r:
            if x["unmatched"] or x["missing_sources"]:
                print(f"  line {x['line']}: unmatched {x['unmatched']}" + (f"  MISSING SOURCES {x['missing_sources']}" if x["missing_sources"] else ""))
    json.dump(out, open(os.path.join(REPO, "runs/report_consistency.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
