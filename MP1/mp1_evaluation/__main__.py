"""Evaluation script for MP1a"""


import argparse
import csv
import logging
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Iterable, List, Mapping, NamedTuple, Tuple

import rtamt
from rtamt import STLDenseTimeSpecification

logger = logging.getLogger("EVAL")
# handler = logging.StreamHandler()
# formatter = logging.Formatter("%(asctime)s %(name)-12s %(levelname)-8s %(message)s")
# handler.setFormatter(formatter)
# logger.addHandler(handler)
logger.setLevel(logging.INFO)


class TraceRow(NamedTuple):
    ego_velocity: float
    target_speed: float
    distance_to_lead: float
    ado_velocity: float


Signal = Iterable[Tuple[float, float]]
Trace = Mapping[str, Signal]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="""Evaluation script for Mini-Project 1a.

        This script expects a set of CSV trace files (saved from running the
        experiment), and computes the robustness of each trace.
        """,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "tracefiles",
        metavar="tracefile.csv",
        nargs="+",
        help="CSV files containing the tracees for experiments.",
        type=lambda p: Path(p).absolute(),
    )

    return parser.parse_args()


def extract_trace(tracefile: Path) -> Trace:
    signals = ["ego_velocity", "target_speed", "distance_to_lead", "lead_speed"]
    trace = defaultdict(deque)  # type: Mapping[str, deque[Tuple[float, float]]]
    with open(tracefile, "r") as f:
        csv_file = csv.DictReader(f)
        for row in csv_file:
            for signal in signals:
                trace[signal].append((float(row["time_elapsed"]), float(row[signal])))
    return trace


def _prepare_spec() -> STLDenseTimeSpecification:
    spec = STLDenseTimeSpecification()
    spec.set_sampling_period(500, "ms", 0.1)
    spec.declare_const("dsafe", "float", "4")
    spec.declare_const("T", "float", "20.0")

    spec.declare_var("ego_velocity", "float")
    spec.declare_var("target_speed", "float")
    spec.declare_var("distance_to_lead", "float")
    spec.declare_var("lead_speed", "float")

    return spec


def _parse_and_eval_spec(spec: STLDenseTimeSpecification, trace: Trace) -> float:
    try:
        spec.parse()
    except rtamt.STLParseException as e:
        logger.critical("STL Parse Exception: {}".format(e))
        sys.exit(1)

    return spec.evaluate(
        ["ego_velocity", list(trace["ego_velocity"])],
        ["target_speed", list(trace["target_speed"])],
        ["distance_to_lead", list(trace["distance_to_lead"])],
        ["lead_speed", list(trace["lead_speed"])],
    )


def checkSafeFollowing(trace: Trace) -> float:
    spec = _prepare_spec()
    spec.name = "Check if the ego maintains safe following distance"

    spec.spec = "always (distance_to_lead >= dsafe)"

    return _parse_and_eval_spec(spec, trace)


def checkForwardProgress(trace: Trace) -> float:
    spec = _prepare_spec()
    spec.name = "Check if ego car is never moving backwards"

    spec.spec = "always (ego_velocity >= 0)"

    return _parse_and_eval_spec(spec, trace)


def checkDontStopUnlessLeadStops(trace: Trace) -> float:
    spec = _prepare_spec()
    spec.name = "Check if ego car stopped without lead stopping"

    spec.declare_const("reallySmallSpeed", "float", "0.1")
    spec.spec = "not((lead_speed > reallySmallSpeed) until[0:T] (ego_velocity < reallySmallSpeed))"

    return _parse_and_eval_spec(spec, trace)


def evaluate_tracefile(tracefile: Path):
    trace = extract_trace(tracefile)

    safeFollowing = checkSafeFollowing(trace)
    print(
        "Robustness for `safeFollowing`           = {}".format(
            (safeFollowing[0], safeFollowing[-1])
        )
    )

    forwardProgress = checkForwardProgress(trace)
    print(
        "Robustness for `forwardProgress`         = {}".format(
            (forwardProgress[0], forwardProgress[-1])
        )
    )

    dontStopUnlessLeadStops = checkDontStopUnlessLeadStops(trace)
    print(
        "Robustness for `dontStopUnlessLeadStops` = {}".format(
            (dontStopUnlessLeadStops[0], dontStopUnlessLeadStops[-1])
        )
    )


def main():
    args = parse_args()

    tracefiles = args.tracefiles  # type: List[Path]
    for tracefile in tracefiles:
        print("===================================================")
        print("Evaluating trace file: %s", str(tracefile.relative_to(Path.cwd())))
        evaluate_tracefile(tracefile)
        print("===================================================")
        print()


if __name__ == "__main__":
    main()
