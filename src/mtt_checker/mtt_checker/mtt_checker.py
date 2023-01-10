#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

from multiprocessing.context import get_spawning_popen
import os, sys
from stat import FILE_ATTRIBUTE_INTEGRITY_STREAM
import argparse
from operator import *
from dataclasses import dataclass, field, asdict
from typing import List, Literal, Tuple, Dict, ClassVar

import yaml
import copy
import itertools

import numpy as np

from pprint import pprint

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py import message_to_ordereddict
import rosbag2_py

from .tm_common import *

from std_msgs.msg import *
import builtin_interfaces.msg
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tilde_msg.msg import *


@dataclass(frozen=False)
class MttInfo1line:
    line: int = 1
    org_topic: str = ""
    org_stamp: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time(sec=0, nanosec=0)
    pub_topic: str = ""        # mtt topic
    pub_stamp: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time(sec=0, nanosec=0)

whole_mtt = []

@dataclass(frozen=False)
class MttTimingAnalizer:
    ok: int = 0
    p_ok: int = 0
    ng: int = 0
    p_ng: int = 0
    # current line
    line: int = 1
    rel_time: float = 0.0       # r_{i, j-1}
    org_interval: float = 0.0
    prev_rel_time: float = 0.0
    pub_time: float = 0.0
    pub_interval: float = 0.0
    prev_pub_time: float = 0.0
    next_rel_time: float = 0.0  # r_{i, j} = r_{i, j-1} + p_i
    proc_time: float = 0.0      # pub_time - rel_time
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
    ana = MttTimingAnalizer()
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
            print_line(ana, p_i, d_i, "fail: deadline miss occured")
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
            #print(f"--- [{ana.l2_line}]{ana.l2_org_interval=} {ana.l2_pub_interval=} {ana.l2_proc_time=} ---")
            if ana.l2_org_interval > d_i  or ana.l2_pub_interval > d_i or ana.l2_proc_time > d_i:
                print_next_line(ana, p_i, d_i, "skip: deadline miss occured")
                ana.l2_prev_rel_time = ana.l2_rel_time
                ana.l2_prev_pub_time = ana.l2_pub_time
                ana.prev_rel_time = ana.l2_rel_time
                ana.prev_pub_time = ana.l2_pub_time
                skip += 1
                continue
            break
        else:
            print_line(ana, p_i, d_i, "one before last line")
            continue
        # analize
        # 前回設定したデッドラインタイマをチェックする
        if skip == 0:
            for t in ana.deadline_timer:
                rel = ana.rel_time + p_i
                if t <= rel:
                    ana.cur_ok += 1
                    ana.ok += 1
                    print_desc(ana, t, "OK")
                else:
                    print_desc(ana, t, "-- IGNORE --")
                    
            ana.deadline_timer.clear()
        #
        # 今回のデッドラインタイマを設定する
        #print_analyze(ana, p_i, d_i, "")
        for t in np.arange(ana.rel_time + p_i, ana.l2_rel_time, p_i):
            if t + d_i <= ana.l2_rel_time:
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
    print(f"--- ({len(whole_mtt):4}) OK:{ana.ok:3} NG:{ana.ng:3} pseudo OK:{ana.p_ok:3} pseudo NG:{ana.p_ng:3} completed_j={ana.ok+ana.p_ok+ana.ng+ana.p_ng:3} ---")
        
def print_line(ana, p_i, d_i, desc):
    print(f"{LOC()}|{ana.line:4}|{ana.rel_time:6f}|{ana.org_interval:6f}|{ana.pub_time:6f}|{ana.pub_interval:6f}|{ana.proc_time:6f}|{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    ana.stime.append(ana.rel_time)

def print_next_line(ana, p_i, d_i, desc):
    print(f"{LOC()}|{ana.l2_line:4}|{ana.l2_rel_time:6f}|{ana.l2_org_interval:6f}|{ana.l2_pub_time:6f}|{ana.l2_pub_interval:6f}|{ana.l2_proc_time:6f}|{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    ana.stime.append(ana.rel_time)
    
def print_analyze(ana, p_i, d_i, desc):
    print(f"{LOC()}|p_i |{ana.next_rel_time:6f}|{ana.next_rel_time - ana.rel_time:6f}|{ana.next_rel_time + d_i:6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    ana.stime.append(ana.next_rel_time)
    
def print_periodic(ana, t, p_i, d_i, desc):
    print(f"{LOC()}|-p_i|{t:6f}|{t - ana.rel_time:6f}|{t + d_i:6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    ana.stime.append(t)

def print_deadline_to(ana, t, desc):
    print(f"{LOC()}|-d_i|{t:.6f}|        |                 |        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    ana.stime.append(t)
    
def print_desc(ana, dead, desc):
    print(f"{LOC()}|    |                 |        |{dead:.6f}|        |        |{len(ana.deadline_timer):3}|{ana.ok=:3}|{ana.p_ok=:3}|{ana.ng=:3}|{ana.p_ng=:3}|{desc}|")
    
def print_title():
    print(f"MTT: {whole_mtt[0].pub_topic}")
    print(f"ORG: {whole_mtt[0].org_topic}")
    #       |    |1585897255.632508|1585897257.086864|0.681261|1.454356|
    print(f"release_time: SRC topic stamp")
    print(f"interval1(s): SRC(n+1)-SRC(n) topic stamp interval")
    print(f"pub_time: MTT topic publish time")
    print(f"interval2(s): MTT(n+1)-MTT(n)) topic stamp interval")
    print(f"proc_time(s): pub_time - release_time(EKF+NDT process time)")
    print(f"interval(s): SRC(n+1)-SRC(n) topic stamp interval\nproc_time(s): pub_time - release_time(EKF+NDT process time)")
    print(f"| No |  release_time   | dur-1  |     pub_time    |  due-2 |  proc  |   |   OK     |    P-OK    |   NG     |    P-NG    |  remarks               |", flush=True)


def mtt_check(input, periodic_time, deadline_time):
    mtt = MessageTrackingTag()
    try:
        count = 0
        with open(input, 'r') as f:
            ydata = yaml.load_all(f, Loader=yaml.FullLoader)
            for mtt in ydata:
                count += 1
                DP(f'count={count}=====================================================')
                if not mtt:
                    break
                ln = MttInfo1line()
                ln.line = count
                ln.pub_topic = mtt['output_info']['topic_name']
                ln.pub_stamp = mtt['output_info']['header_stamp']
                ln.org_topic = mtt['input_infos'][0]['topic_name']
                ln.org_stamp = mtt['input_infos'][0]['header_stamp']
                whole_mtt.append(ln)
    except Exception as e:
        print(f"{location()}[Exception] {e}", file=sys.stderr) # discard error
        sys.exit(-1)
    analyze(periodic_time, deadline_time)
    print(f'(END)---------------------------------')
    
#
def main():
    parser = argparse.ArgumentParser(description=f"check the mtt for deadline.",
                                     usage="ros2 run tilde_timing_monitor mtt_checker [-h] mtt_config(=yaml) periodic_time deadline_time(=ms)")
    parser.add_argument('mtt_file', metavar='mtt_file', help="Input file: mtt topic yaml file (extension must be 'yaml')")
    parser.add_argument('periodic_time', metavar='periodic_time', type=float, help="periodic_time(ms)")
    parser.add_argument('deadline_time', metavar='deadline_time', type=float, help="deadline_time(ms)")
    args = parser.parse_args()

    debug_ctrl(False)

    if args.mtt_file.endswith('.yaml') != True:
        print(f"## input file extension must be 'yaml' : {args.mtt_file}", flush=True)
        sys.exit(-1)
    if args.periodic_time == None or args.periodic_time <= 0.0:
        print(f"## periodic_time time (ms)", flush=True)
        sys.exit(-1)
    if args.deadline_time == None or args.deadline_time <= 0.0:
        print(f"## deadline time (ms)", flush=True)
        sys.exit(-1)
    if args.deadline_time <= args.periodic_time:
        print(f"## deadline time should be greater than periodic time", flush=True)
        sys.exit(-1)
    print(f"--- START ({args.mtt_file}) ---")
    print(f"    periodic_time={args.periodic_time}(msec) deadline_time={args.deadline_time}(msec)\n")
    mtt_check(args.mtt_file, args.periodic_time / 1000, args.deadline_time / 1000)

if __name__ == "__main__":
    main()
    