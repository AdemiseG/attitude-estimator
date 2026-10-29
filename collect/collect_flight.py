"""
Generate a flight log with real attitude dynamics, and extract it to CSV.

The first dataset for this project came from a takeoff/hover/land, which turned
out to be useless for validation: the aircraft never tilted past 0.43 deg, so
an estimator that always outputs zero scored 0.02 deg RMS. No estimator can be
ranked on data like that.

This flies the simulated copter hard - alternating full-speed velocity demands
in GUIDED, which force it to bank to its ANGLE_MAX - and pulls raw IMU plus
simulator ground truth out of the resulting dataflash log.

Usage:  python collect/collect_flight.py [out.csv]
"""
import os
import subprocess
import sys
import time

from pymavlink import mavutil

ARDUPILOT = os.path.expanduser("~/ardupilot")
BINARY = os.path.join(ARDUPILOT, "build/sitl/bin/arducopter")
RUNDIR = os.path.join(ARDUPILOT, "ArduCopter")
SPEEDUP = 10

# Flight profile: (north m/s, east m/s, down m/s, seconds to hold).
# Reversing a 25 m/s demand makes it pitch/roll to the limit to decelerate
# and accelerate the other way, which is exactly the signal we lacked.
PROFILE = [
    (  25,   0, 0, 6), (-25,   0, 0, 6),
    (   0,  25, 0, 6), (  0, -25, 0, 6),
    (  18,  18, 0, 6), (-18, -18, 0, 6),
    (  25,   0, 0, 4), (  0,  25, 0, 4),
    ( -25,   0, 0, 4), (  0, -25, 0, 4),
    (  20, -20, 0, 6), (-20,  20, 0, 6),
    (   0,   0, 0, 4),
]


def log(msg):
    print("[collect] %s" % msg, flush=True)


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)


def request_streams(m, rate_hz=10):
    """Ask for telemetry. Nothing streams by default when we drive SITL
    directly instead of going through MAVProxy."""
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                   rate_hz, 1)
    time.sleep(0.5)


def wait_gps(m, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
        if msg and msg.fix_type >= 3:
            log("GPS 3D fix")
            return True
    raise RuntimeError("no GPS fix within %ss" % timeout)


def cmd(m, command, *params):
    args = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component,
                            command, 0, *args)


MODE_GUIDED = 4
MODE_LAND = 9


def ack(m, label, timeout=5):
    msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if msg is None:
        log("%s: no ACK received" % label)
    else:
        log("%s: ACK result=%d" % (label, msg.result))
    return msg


def wait_mode(m, mode_num, timeout=30):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and hb.custom_mode == mode_num:
            return True
    return False


def wait_armed(m, want=True, timeout=60):
    """Bounded arm/disarm wait. motors_armed_wait() blocks forever if arming
    is refused, which is how the first version of this script hung."""
    flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb and bool(hb.base_mode & flag) == want:
            return True
    return False


def drain_statustext(m, label=""):
    """ArduPilot explains every arming refusal in a STATUSTEXT. Throwing those
    away is why the first arming failure was opaque."""
    seen = []
    while True:
        msg = m.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            break
        text = msg.text.strip()
        if text:
            seen.append(text)
            log("  AP%s: %s" % (label, text))
    return seen


def wait_ekf(m, timeout=180):
    """A 3D GPS fix is not the same as a converged EKF. Copter refuses to arm
    in GUIDED until it has a horizontal position estimate."""
    need = (mavutil.mavlink.EKF_ATTITUDE |
            mavutil.mavlink.EKF_VELOCITY_HORIZ |
            mavutil.mavlink.EKF_POS_HORIZ_REL |
            mavutil.mavlink.EKF_POS_HORIZ_ABS)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=5)
        if msg and (msg.flags & need) == need:
            log("EKF ready (flags 0x%x)" % msg.flags)
            return True
    log("EKF never reported ready; trying to arm anyway")
    return False


def fly(m):
    request_streams(m)
    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "ANGLE_MAX", 4500)      # allow 45 deg of bank
    set_param(m, "LOG_BITMASK", 65535)   # make sure IMU + SIM are logged
    wait_gps(m)

    for name in ("ARMING_CHECK", "ANGLE_MAX"):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        log("param %s = %s" % (name, msg.param_value if msg else "NO REPLY"))

    log("switching to GUIDED")
    m.set_mode_apm("GUIDED")
    if not wait_mode(m, MODE_GUIDED):
        raise RuntimeError("never entered GUIDED")
    log("in GUIDED")

    wait_ekf(m)
    drain_statustext(m)

    armed = False
    for attempt in range(1, 6):
        log("arming (attempt %d)" % attempt)
        cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        ack(m, "arm")
        if wait_armed(m, True, timeout=15):
            armed = True
            break
        drain_statustext(m, " refused")
    if not armed:
        raise RuntimeError("could not arm after 5 attempts; see AP messages above")
    log("armed")

    log("takeoff to 80 m")
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 80)
    ack(m, "takeoff")
    deadline = time.time() + 180 / SPEEDUP + 30
    alt = 0.0
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            alt = msg.relative_alt / 1000.0
            if alt > 75:
                break
    log("altitude %.1f m" % alt)
    if alt < 60:
        raise RuntimeError("takeoff did not reach altitude (got %.1f m)" % alt)

    mask = 0b0000111111000111   # velocity only
    for north, east, down, hold in PROFILE:
        log("velocity N=%+d E=%+d for %ds" % (north, east, hold))
        end_t = time.time() + hold / SPEEDUP
        while time.time() < end_t:
            m.mav.set_position_target_local_ned_send(
                0, m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                0, 0, 0, north, east, down, 0, 0, 0, 0, 0)
            time.sleep(0.05)

    log("landing")
    m.set_mode_apm("LAND")
    if wait_armed(m, False, timeout=300 / SPEEDUP + 60):
        log("disarmed on ground")
    else:
        log("still armed after landing window; log is still usable")


def extract(binlog, out_csv):
    """Pull IMU and SIM out of the dataflash log and pair them up in time."""
    log("reading %s" % binlog)
    mlog = mavutil.mavlink_connection(binlog)

    imu, sim = [], []
    while True:
        msg = mlog.recv_match(type=["IMU", "SIM"])
        if msg is None:
            break
        if msg.get_type() == "IMU" and getattr(msg, "I", 0) == 0:
            imu.append((msg.TimeUS, msg.GyrX, msg.GyrY, msg.GyrZ,
                        msg.AccX, msg.AccY, msg.AccZ))
        elif msg.get_type() == "SIM":
            sim.append((msg.TimeUS, msg.Roll, msg.Pitch, msg.Yaw))

    log("%d IMU samples, %d SIM samples" % (len(imu), len(sim)))
    if not imu or not sim:
        raise RuntimeError("log is missing IMU or SIM messages")

    # merge walk: for each IMU sample take the nearest-in-time SIM sample
    rows, j = [], 0
    for rec in imu:
        t = rec[0]
        while j + 1 < len(sim) and abs(sim[j + 1][0] - t) <= abs(sim[j][0] - t):
            j += 1
        if abs(sim[j][0] - t) > 100000:      # >0.1 s apart, discard
            continue
        rows.append(rec + sim[j][1:])

    t0 = rows[0][0]
    with open(out_csv, "w") as f:
        f.write("t,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,"
                "truth_roll_deg,truth_pitch_deg,truth_yaw_deg\n")
        for r in rows:
            f.write("%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n"
                    % (((r[0] - t0) / 1e6,) + tuple(r[1:])))
    log("wrote %s (%d rows, %.1f s)" % (out_csv, len(rows), (rows[-1][0] - t0) / 1e6))
    return out_csv


def main():
    out_csv = sys.argv[1] if len(sys.argv) > 1 else "data/flight2.csv"
    logdir = os.path.join(RUNDIR, "logs")
    before = set(os.listdir(logdir)) if os.path.isdir(logdir) else set()

    log("starting SITL (speedup %d)" % SPEEDUP)
    proc = subprocess.Popen(
        [BINARY, "--model", "quad", "--speedup", str(SPEEDUP),
         "--home", "-35.363261,149.165230,584,353"],
        cwd=RUNDIR, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        time.sleep(3)
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
        m.wait_heartbeat()
        log("connected")
        fly(m)
    finally:
        log("stopping SITL")
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()

    after = set(os.listdir(logdir))
    new = sorted(n for n in after - before if n.endswith(".BIN"))
    if not new:
        raise RuntimeError("SITL produced no new .BIN log in %s" % logdir)
    extract(os.path.join(logdir, new[-1]), out_csv)


if __name__ == "__main__":
    main()
