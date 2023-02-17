#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

import argparse
from dataclasses import dataclass
from dataclasses import field
import sys
from typing import List

import builtin_interfaces.msg
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
import yaml

from tilde_msg.msg import MessageTrackingTag

from .tm_common import DP
from .tm_common import LOC
from .tm_common import debug_ctrl
from .tm_common import location
from .tm_common import stamp_to_sec

VER = "v0.20"


@dataclass(frozen=False)
class MttInfo1line:
    line: int = 1
    org_topic: str = ""
    org_stamp: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time(sec=0, nanosec=0)
    pub_topic: str = ""  # mtt topic
    pub_stamp: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time(sec=0, nanosec=0)


whole_mtt = []


@dataclass(frozen=False)
class MttTimingAnalyzer:
    ok: int = 0
    p_ok: int = 0
    ng: int = 0
    p_ng: int = 0
    # current line
    line: int = 1
    rel_time: float = 0.0  # r_{i, j-1}
    org_interval: float = 0.0
    prev_rel_time: float = 0.0
    pub_time: float = 0.0
    pub_interval: float = 0.0
    prev_pub_time: float = 0.0
    next_rel_time: float = 0.0  # r_{i, j} = r_{i, j-1} + p_i
    proc_time: float = 0.0  # pub_time - rel_time
    # next line
    l2_line: int = 0
    l2_rel_time: float = 0.0
    l2_org_interval: float = 0.0
    l2_prev_rel_time: float = 0.0
    l2_pub_time: float = 0.0
    l2_pub_interval: float = 0.0
    l2_prev_pub_time: float = 0.0
    l2_proc_time: float = 0.0
    # calc
    deadline_timer: List[float] = field(default_factory=list)
    stime: List[float] = field(default_factory=list)


def analyze(p_i, d_i):
    print_title()
    ana = MttTimingAnalyzer()
    skip = 0
    for idx, ln in enumerate(whole_mtt):
        if skip > 0:
            skip -= 1
            continue
        # prepare
        ana.cur_ok = ana.cur_ng = ana.cur_p_ok = ana.cur_p_ng = 0
        ana.line = ln.line
        ana.rel_time = stamp_to_sec(ln.org_stamp)
        if ln.line == 1:
            ana.org_interval = 0
        else:
            ana.org_interval = ana.rel_time - ana.prev_rel_time
        ana.prev_rel_time = ana.rel_time
        ana.pub_time = stamp_to_sec(ln.pub_stamp)
        if ln.line == 1:
            ana.pub_interval = 0
        else:
            ana.pub_interval = ana.pub_time - ana.prev_pub_time
        ana.prev_pub_time = ana.pub_time
        ana.next_rel_time = ana.rel_time + p_i
        ana.proc_time = ana.pub_time - ana.rel_time
        if ana.proc_time > d_i or ana.org_interval > d_i or ana.pub_interval > d_i:
            print_line(ana, p_i, d_i, "fail: deadline miss")
            continue
        print_line(ana, p_i, d_i, "")
        ana.l2_prev_rel_time = ana.rel_time
        ana.l2_prev_pub_time = ana.pub_time
        for n in range(len(whole_mtt) - (idx + 1)):
            i = n + idx + 1
            ana.l2_line = whole_mtt[i].line
            ana.l2_rel_time = stamp_to_sec(whole_mtt[i].org_stamp)
            ana.l2_org_interval = ana.l2_rel_time - ana.l2_prev_rel_time
            ana.l2_pub_time = stamp_to_sec(whole_mtt[i].pub_stamp)
            ana.l2_pub_interval = ana.l2_pub_time - ana.l2_prev_pub_time
            ana.l2_proc_time = ana.l2_pub_time - ana.l2_rel_time
            # print(f"--- [{ana.l2_line}]{ana.l2_org_interval=} {ana.l2_pub_interval=} {ana.l2_proc_time=} ---")
            if ana.l2_org_interval > d_i or ana.l2_pub_interval > d_i or ana.l2_proc_time > d_i:
                print_next_line(ana, p_i, d_i, "skip: deadline miss")
                ana.l2_prev_rel_time = ana.l2_rel_time
                ana.l2_prev_pub_time = ana.l2_pub_time
                ana.prev_rel_time = ana.l2_rel_time
                ana.prev_pub_time = ana.l2_pub_time
                skip += 1
                continue
            break
        else:
            print_line(ana, p_i, d_i, "one before last line")
        # analyze
        # check the prev deadline timers
        if skip == 0:
            first = False
            for t in ana.deadline_timer:
                rel = ana.rel_time
                if first is False and t <= rel:
                    ana.cur_ok += 1
                    ana.ok += 1
                    first = True
                    print_desc(ana, t, "OK")
                else:
                    print_desc(ana, t, "-- IGNORE --")

            ana.deadline_timer.clear()
        #
        # set current deadline timers
        # print_analyze(ana, p_i, d_i, "")
        for t in np.arange(ana.rel_time + p_i, ana.l2_rel_time, p_i):
            if t + d_i < ana.l2_rel_time + ana.l2_proc_time:
                ana.cur_ng += 1
                ana.ng += 1
                print_periodic(ana, t, p_i, d_i, "periodic+deadline TIMEOUT")
            else:
                print_periodic(ana, t, p_i, d_i, "periodic+deadline START")
                ana.deadline_timer.append(t)
    cnt = 0
    for i, w in enumerate(ana.stime):
        if i + i >= len(ana.stime):
            continue
        if w == ana.stime[i + 1]:
            continue
        cnt += 1
    fm = stamp_to_sec(whole_mtt[0].org_stamp)
    to = stamp_to_sec(whole_mtt[len(whole_mtt) - 1].org_stamp)
    print(f"--- p_i={p_i * 1000:4.2f}(ms) d_i={d_i * 1000:4.2f}(ms) ---")
    print(f"--- p_i count={cnt:4} p_i in time={(to - fm) / p_i:4.3f} ---")
    print(
        f"--- ({len(whole_mtt):4}) OK:{ana.ok:3} NG:{ana.ng:3} completed_j={ana.ok+ana.p_ok+ana.ng+ana.p_ng:3} ---"
    )


def print_line(ana, p_i, d_i, desc):
    print(
        f"{LOC()}|{ana.line:4}|{ana.rel_time:6f}|{ana.org_interval:6f}|{ana.pub_time:6f}|{ana.pub_interval:6f}|{ana.proc_time:6f}|{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}||{desc}|"
    )
    ana.stime.append(ana.rel_time)


def print_next_line(ana, p_i, d_i, desc):
    print(
        f"{LOC()}|{ana.l2_line:4}|{ana.l2_rel_time:6f}|{ana.l2_org_interval:6f}|{ana.l2_pub_time:6f}|{ana.l2_pub_interval:6f}|{ana.l2_proc_time:6f}|{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}|{desc}|"
    )
    ana.stime.append(ana.rel_time)


def print_analyze(ana, p_i, d_i, desc):
    print(
        f"{LOC()}|p_i |{ana.next_rel_time:6f}|{ana.next_rel_time - ana.rel_time:6f}|{ana.next_rel_time + d_i:6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}|{desc}|"
    )
    ana.stime.append(ana.next_rel_time)


def print_periodic(ana, t, p_i, d_i, desc):
    print(
        f"{LOC()}|-p_i|{t:6f}|{t - ana.rel_time:6f}|{t + d_i:6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}|{desc}|"
    )
    ana.stime.append(t)


def print_deadline_to(ana, t, desc):
    print(
        f"{LOC()}|-d_i|{t:.6f}|        |                 |        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}|{desc}|"
    )
    ana.stime.append(t)


def print_desc(ana, dead, desc):
    print(
        f"{LOC()}|    |                 |        |{dead:.6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.ng=:3}|{desc}|"
    )


def print_title():
    print(f"MTT: {whole_mtt[0].pub_topic}")
    print(f"ORG: {whole_mtt[0].org_topic}")
    #       |    |1585897255.632508|1585897257.086864|0.681261|1.454356|
    print("release_time: SRC topic stamp")
    print("dur-1(s): SRC(n+1)-SRC(n) topic stamp interval")
    print("pub_time: MTT topic publish time")
    print("dur-2(s): MTT(n+1)-MTT(n)) topic stamp interval")
    print("proc(s): pub_time - release_time (response_time)")
    print(
        "| No |  release_time   | dur-1  |     pub_time    |  dur-2 |  proc  |   |   OK     |   NG     |    remarks            |",
        flush=True,
    )


prev_pub_time = 0.0
prev_rel_time = 0.0
ok = 0
ng = 0


def simple_analyze(mtt, count, p_i, d_i):
    global prev_pub_time, prev_rel_time, ok, ng
    try:
        if not mtt:
            return
        pub_topic = mtt["output_info"]["topic_name"]
        pub_stamp = builtin_interfaces.msg.Time(sec=0, nanosec=0)
        pub_stamp = mtt["header"]["stamp"]
        pub_time = stamp_to_sec(pub_stamp)
        sub_topic = mtt["input_infos"][0]["topic_name"]
        org_topic_stamp = builtin_interfaces.msg.Time(sec=0, nanosec=0)
        org_topic_stamp = mtt["input_infos"][0]["header_stamp"]
        org_topic_stamp_sec = stamp_to_sec(org_topic_stamp)
    except Exception as e:
        print(f"{location()}[Exception] {e}", file=sys.stderr)  # discard error
        sys.exit(-1)

    if prev_pub_time == 0.0:
        print(f"MTT: {pub_topic}")
        print(f"SRC: {sub_topic}")
        #       |    |1585897255.632508|1585897257.086864|0.681261|1.454356|
        print("release_time: SRC topic stamp")
        print("interval1(s): SRC(n+1)-SRC(n) topic stamp interval")
        print("pub_time: MTT topic publish time")
        print("interval2(s): MTT(n+1)-MTT(n)) topic stamp interval")
        print("proc_time(s): pub_time - release_time(EKF+NDT process time)")
        print(
            "interval(s): SRC(n+1)-SRC(n) topic stamp interval\nproc_time(s): pub_time - release_time(EKF+NDT process time)"
        )
        print(
            "| No |  release_time   |interval1|     pub_time    |interval2|proc_time|deadline decision             |",
            flush=True,
        )
        prev_rel_time = org_topic_stamp_sec
        prev_pub_time = pub_time
    release_time = org_topic_stamp_sec
    proc_time = pub_time - release_time
    interval_time1 = release_time - prev_rel_time
    interval_time2 = pub_time - prev_pub_time
    ng_count = 0
    if proc_time >= d_i:
        ng_count += int(proc_time / d_i)
    # print(f"## {prev_rel_time=} {release_time=}")
    for t in np.arange(prev_rel_time, release_time, p_i):
        if t + d_i < release_time:
            # print(f"## {t}+{d_i} : {release_time}")
            ng_count += 1
    if ng_count > 0:
        m = f"Deadline miss.(Over {d_i} ({ng_count:3}))"
        ng += ng_count
    else:
        ok += 1
        m = f"OK.(Under {d_i})"
    prev_rel_time = release_time
    prev_pub_time = pub_time
    print(
        f"|{count:4}|{release_time:06f}| {interval_time1:06f}|{pub_time:06f}| {interval_time2:06f}| {proc_time:06f}|{m}|"
    )


#
def simple_analyze_header_only(mtt, count, p_i, d_i, pub_topic):
    global prev_pub_time, prev_rel_time, ok, ng
    try:
        if not mtt:
            return
        org_topic_stamp = builtin_interfaces.msg.Time(sec=0, nanosec=0)
        org_topic_stamp = mtt["header"]["stamp"]
        org_topic_stamp_sec = stamp_to_sec(org_topic_stamp)
    except Exception as e:
        print(f"{location()}[Exception] {e}", file=sys.stderr)  # discard error
        sys.exit(-1)

    if prev_rel_time == 0.0:
        print(f"header: {pub_topic}")
        #       |    |1585897255.632508|1585897257.086864|0.681261|1.454356|
        print("release_time: topic stamp")
        print("interval(s): stamp(n+1)-stamp(n) topic stamp interval")
        print("| No |  release_time   |interval|deadline decision             |", flush=True)
        prev_rel_time = org_topic_stamp_sec
    release_time = org_topic_stamp_sec
    interval_time1 = release_time - prev_rel_time
    ng_count = 0
    # print(f"## {prev_rel_time=} {release_time=}")
    for t in np.arange(prev_rel_time, release_time, p_i):
        if t + d_i < release_time:
            # print(f"## {t}+{d_i} : {release_time}")
            ng_count += 1
    if ng_count > 0:
        m = f"Deadline miss.(Over {d_i} ({ng_count:3}))"
        ng += ng_count
    else:
        ok += 1
        m = f"OK.(Under {d_i})"
    prev_rel_time = release_time
    print(f"|{count:4}|{release_time:06f}| {interval_time1:06f}|{m}|")


# ros2 treat rosbag utility
def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def mtt_check(mode, in_file, periodic_time, deadline_time, mtt_topic):
    if in_file.endswith(".yaml"):
        mtt = MessageTrackingTag()
        try:
            count = 0
            with open(in_file, "r") as f:
                ydata = yaml.load_all(f, Loader=yaml.FullLoader)
                for mtt in ydata:
                    count += 1
                    DP(f"count={count}=====================================================")
                    if not mtt:
                        break
                    if mode == "normal":
                        ln = MttInfo1line()
                        ln.line = count
                        ln.pub_topic = mtt["output_info"]["topic_name"]
                        ln.pub_stamp = mtt["output_info"]["header_stamp"]
                        ln.org_topic = mtt["input_infos"][0]["topic_name"]
                        ln.org_stamp = mtt["input_infos"][0]["header_stamp"]
                        whole_mtt.append(ln)
                    else:
                        # simple_analyze(mtt, count, periodic_time, deadline_time)
                        simple_analyze_header_only(
                            mtt, count, periodic_time, deadline_time, mtt_topic
                        )
        except Exception as e:
            print(f"{location()}[Exception] {e}", file=sys.stderr)  # discard error
            sys.exit(-1)
    else:
        try:
            storage_options, converter_options = get_rosbag_options(in_file)
            # print('storage_options=%s, converter_options=%s' % (storage_options, converter_options))
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            topic_types = reader.get_all_topics_and_types()
            type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        except Exception as e:
            print(f"{location()}[Exception] {e} input file not rosbag {in_file}", file=sys.stderr)
            sys.exit(-1)
        count = 0
        while reader.has_next():
            try:
                (topic, data, t) = reader.read_next()  # これら3つの要素をタプルで返す
                # print('topic=%s t=%s' % (topic, t))
                if mtt_topic not in topic:
                    continue
                if "mtt" in topic or "message_tracking_tag" in topic:
                    # print(f"OLD {t}")
                    pass
                else:
                    continue
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                # print(type(msg))
                mtt = message_to_ordereddict(msg)
            except Exception as e:
                print(f"{location()}[Exception] {e}", file=sys.stderr)
                continue
            count += 1
            DP(f"count={count}=====================================================")
            if not mtt:
                break
            if mode == "normal":
                ln = MttInfo1line()
                ln.line = count
                ln.pub_topic = mtt["output_info"]["topic_name"]
                ln.pub_stamp = mtt["output_info"]["header_stamp"]
                ln.org_topic = mtt["input_infos"][0]["topic_name"]
                ln.org_stamp = mtt["input_infos"][0]["header_stamp"]
                whole_mtt.append(ln)
            else:
                # simple_analyze(mtt, count, periodic_time, deadline_time)
                simple_analyze_header_only(mtt, count, periodic_time, deadline_time, mtt_topic)

    if mode == "normal":
        analyze(periodic_time, deadline_time)
    else:
        print(
            f"--- OK={ok} Deadline miss={ng} mtt topic={count - 1} ---------------------------------"
        )
    print(f"(END:{VER})---------------------------------")


#
def main():
    parser = argparse.ArgumentParser(
        description="Check the deadline by Message tracking tag (mtt).",
        usage="ros2 run mtt_checker mtt_checker [-h] [-m mode] [-p time] [-d time] [-t name] input file",
    )
    parser.add_argument("input", metavar="input", help="Input rosbag or MTT yaml file")
    parser.add_argument(
        "-m",
        "--mode",
        metavar="mode",
        # default="normal",
        # help="simple: check stamp only, normal: default",
        default="simple",
        help="simple: check stamp only",
    )
    parser.add_argument(
        "-p",
        "--periodic",
        metavar="time",
        type=float,
        default=100.0,
        help="periodic time default:100.0 (ms)",
    )
    parser.add_argument(
        "-d",
        "--deadline",
        metavar="time",
        type=float,
        default=200.0,
        help="deadline detect time default:200.0 (ms)",
    )
    parser.add_argument(
        "-t",
        "--topic",
        metavar="name",
        type=str,
        default="for_tilde_interpolator_mtt",
        help="topic name: Specify the target topic if there are multiple MTTs in the rosbag. \
                         \ndefault: for_tilde_interpolator_mtt",
    )
    args = parser.parse_args()

    debug_ctrl(False)

    if args.mode != "normal" and args.mode != "simple":
        print(f"## args mode error {args.mode}")
        sys.exit(-1)

    if args.mode == "normal":
        print(f"## Not supported normal mode {args.mode}")
        sys.exit(-1)

    if args.mode == "normal" and (args.periodic is None or args.periodic <= 0.0):
        print(f"## {args.mode} periodic {args.periodic} (ms) error", flush=True)
        sys.exit(-1)
    if args.deadline is None or args.deadline <= 0.0:
        print(f"## deadline {args.deadline} (ms) error", flush=True)
        sys.exit(-1)
    elif args.mode == "normal" and (args.deadline <= args.periodic):
        print(
            f"## deadline {args.deadline} should be greater than periodic {args.periodic}",
            flush=True,
        )
        sys.exit(-1)
    print(f"--- START ({args.mode}: {args.input}) ---")
    print(f"    periodic_time={args.periodic}(ms) deadline_time={args.deadline}(ms)\n")
    mtt_check(args.mode, args.input, args.periodic / 1000, args.deadline / 1000, args.topic)


if __name__ == "__main__":
    main()
