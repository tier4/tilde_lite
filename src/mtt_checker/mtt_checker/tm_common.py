#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import _thread
import os
import sys
import inspect

from rclpy.clock import Clock
from rclpy.clock import ClockType

DEBUG_LEVEL_DEBUG = 1
DEBUG_LEVEL_INFO = 2

g_debug_level = DEBUG_LEVEL_DEBUG
g_prev_time = 0.0
TILDE_TM_DEBUG = "TILDE_TM_DEBUG"

DEBUG = False
g_msg_dump_enable = False
g_tid_prev_time = {}
g_loc_prev_time = 0.0

COMMAND_SHOW_INFO = "show info"
COMMAND_REQ_INFO = "req info"
COMMAND_CLR_INFO = "clr info"
COMMAND_SHOW_HIST = "show hist"
COMMAND_CLR_HIST = "clr hist"
COMMAND_HIST_ON = "histon"
COMMAND_HIST_OFF = "histoff"
COMMAND_HIST_DISP_ON = "dispon"
COMMAND_HIST_DISP_OFF = "dispoff"
COMMAND_MSG_DUMP_ON = "msgon"
COMMAND_MSG_DUMP_OFF = "msgoff"
COMMAND_DEBUG_ON = "debug on"
COMMAND_DEBUG_OFF = "debug off"

def LOC(depth=0):
    if DEBUG:
        frame = inspect.currentframe().f_back
        loc = f"[{frame.f_code.co_name:18}:{frame.f_lineno:04}]"
        return loc
    else:
        return ""

def location(depth=0):
    frame = inspect.currentframe().f_back
    return os.path.basename(frame.f_code.co_filename), frame.f_code.co_name, frame.f_lineno, "---"

def init_debug():
    global DEBUG

    v = os.getenv(TILDE_TM_DEBUG, default=False)
    if type(v) is str and v.upper() == "TRUE":
        DEBUG = True
    elif type(v) is bool and v is True:
        DEBUG = True
    else:
        DEBUG = False

def DP(dstr=None, level=DEBUG_LEVEL_DEBUG):
    if DEBUG:
        if level >= g_debug_level:
            tid = _thread.get_ident()
            if tid not in g_tid_prev_time.keys():
                g_tid_prev_time.update({tid: 0.0})
            prev = g_tid_prev_time[tid]
            # print(f"{location()}: !!! {tid=} {prev=}", flush=True)
            difftime = clk_nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now()) - prev
            if dstr is None or dstr == "" or len(dstr) == 0:
                print(f"[{difftime:.9f}]", flush=True)
            else:
                print(f"[{difftime:.9f}] {dstr}", flush=True)
            cur = clk_nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
            g_tid_prev_time.update({tid: cur})

def PP(dstr):
    tid = _thread.get_ident()
    if tid not in g_tid_prev_time.keys():
        g_tid_prev_time.update({tid: 0.0})
    prev = g_tid_prev_time[tid]
    difftime = clk_nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now()) - prev
    print(f"[{difftime:.9f}] --- {dstr} ---", flush=True)
    cur = clk_nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
    g_tid_prev_time.update({tid: cur})

def stamp_to_sec(stamp):
    return stamp["sec"] + stamp["nanosec"] / (1000 * 1000 * 1000)

def clk_nano_to_sec(t):
    # print(f"!!! {type(t)=} {t}")
    return float(t.nanoseconds) / (1000.0 * 1000.0 * 1000.0)

#
def msg_dump(path_name, topic_name, msg, sub_time):
    global g_msg_dump_enable
    # print(f"### {g_msg_dump_enable=}", flush=True)
    if g_msg_dump_enable:
        print(
            f"\n=== {sub_time:6f} [{topic_name}] in {path_name}: {msg.output_info.header_stamp} ==="
        )
        for w in msg.input_infos:
            print(f"   (sub) [{w.topic_name}] {w.header_stamp}")
        sys.stdout.flush()

def msg_ctrl(ope):
    global g_msg_dump_enable
    g_msg_dump_enable = ope
    print(f"### {g_msg_dump_enable=}", flush=True)

def debug_ctrl(ope):
    global DEBUG
    DEBUG = ope
    print(f"### {DEBUG=}", flush=True)

#
g_hist_on = True
g_hist_disp = False
hist_buf = []
hist_limit = 1000 * 100

def hist_log(m):
    global g_prev_hist
    if not g_hist_on:
        return
    if len(hist_buf) >= hist_limit:
        del hist_buf[0:1000]
    hist_buf.append(m)
    if g_hist_disp:
        print(f"{m}", flush=True)

def show_hist(command, node):
    print("\n--- start history ---\n")
    for w in hist_buf:
        print(w)
    print("\n---(END)---\n", flush=True)

def clr_hist(command):
    global hist_buf
    hist_buf.clear()

def hist_ctrl(ope):
    global g_hist_on
    g_hist_on = ope

def hist_disp(ope):
    global g_hist_disp
    g_hist_disp = ope
