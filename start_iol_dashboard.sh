#!/bin/bash
# Startup script for IOL Dashboard and Master
# Part of Big-Beautiful-Box project

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$HOME/iol_dashboard.log"
LOG_FILTER="$SCRIPT_DIR/src/log_filter.py"

# Find Xwayland auth file dynamically
if [ -d "/run/user/1000" ]; then
    export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwayland* 2>/dev/null | head -1)
fi

export DISPLAY=:0
export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"

# Disable screen blanking
disable_screensaver() {
    xset s off 2>/dev/null || true
    xset s noblank 2>/dev/null || true
    xset -dpms 2>/dev/null || true
    gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null || true
    gsettings set org.gnome.desktop.screensaver lock-enabled false 2>/dev/null || true
    gsettings set org.gnome.desktop.screensaver idle-activation-enabled false 2>/dev/null || true
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null || true
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing' 2>/dev/null || true
    gsettings set org.gnome.settings-daemon.plugins.power idle-dim false 2>/dev/null || true
    gsettings set org.gnome.desktop.notifications show-banners false 2>/dev/null || true
}

disable_screensaver

echo "$(date): Starting IOL Dashboard..." >> "$LOG_FILE"

# ---------------------------------------------------------------------------
# IOL master application (Pinetek iol-master-appl, vendored in this repo).
#
# 2026-09-05: the in-app updater never touched ~/iol-hat, so boxes ran whatever
# install.sh built once (the DEBUG binary, whose per-cycle logging through the
# log pipe stalled the IO-Link cycle thread and, with the old 100 ms watchdog,
# produced the field "flow meter disconnect" + pump-stop pulses). This script
# runs from the repo, so it now (a) syncs the vendored source into ~/iol-hat when
# it differs, (b) builds the RELEASE binary (no PINEDEBUG logging; ~8 s on a
# Pi 5), and (c) supervises the master, restarting it if it ever exits. Every
# step is fail-soft: if the sync/build fails, the previously built binary is used.
# ---------------------------------------------------------------------------
IOL_SRC_REPO="$SCRIPT_DIR/iol-hat/src-master-application"
IOL_HOME="$HOME/iol-hat/src-master-application"
IOL_RELEASE_BIN="$IOL_HOME/bin/release/iol-master-appl"
IOL_DEBUG_BIN="$IOL_HOME/bin/debug/iol-master-appl"
IOL_MASTER_ARGS="-m0 0 -m1 3 -i 34"

sync_and_build_iol_master() {
    [ -d "$IOL_SRC_REPO" ] || { echo "$(date): IOL master source not in repo; skipping sync/build" >> "$LOG_FILE"; return 0; }
    mkdir -p "$IOL_HOME"
    local changed=0 d
    for d in ilink include iol_osal src lib Makefile; do
        if ! diff -rq "$IOL_SRC_REPO/$d" "$IOL_HOME/$d" >/dev/null 2>&1; then
            changed=1
            rm -rf "$IOL_HOME/$d"
            cp -r "$IOL_SRC_REPO/$d" "$IOL_HOME/$d" 2>/dev/null || true
        fi
    done
    if [ "$changed" = 1 ] || [ ! -x "$IOL_RELEASE_BIN" ]; then
        echo "$(date): IOL master source $([ "$changed" = 1 ] && echo changed || echo unbuilt); building release binary..." >> "$LOG_FILE"
        if ( cd "$IOL_HOME" && rm -rf build/release && make BUILD=release >> "$LOG_FILE" 2>&1 ); then
            echo "$(date): IOL master release build OK" >> "$LOG_FILE"
        else
            echo "$(date): WARNING: IOL master release build FAILED; will fall back to an existing binary" >> "$LOG_FILE"
        fi
    fi
}

launch_iol_master() {
    # Prefer release; fall back to debug. Prints the PID of the launched process.
    local bin
    if [ -x "$IOL_RELEASE_BIN" ]; then bin="$IOL_RELEASE_BIN"; else bin="$IOL_DEBUG_BIN"; fi
    [ -x "$bin" ] || { echo "$(date): IOL Master binary not found" >> "$LOG_FILE"; return 1; }
    cd "$IOL_HOME"
    # Prefer real-time scheduling (FIFO) on core 3, but only if passwordless sudo is actually available.
    if command -v chrt >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1; then
        sudo -n chrt -f 80 taskset -c 3 "$bin" $IOL_MASTER_ARGS > >(python3 -u "$LOG_FILTER" "$LOG_FILE") 2>&1 &
        IOL_MASTER_PID=$!
        echo "$(date): IOL Master PID: $IOL_MASTER_PID (SCHED_FIFO 80, core 3) bin=$bin" >> "$LOG_FILE"
    else
        taskset -c 3 "$bin" $IOL_MASTER_ARGS > >(python3 -u "$LOG_FILTER" "$LOG_FILE") 2>&1 &
        IOL_MASTER_PID=$!
        echo "$(date): IOL Master PID: $IOL_MASTER_PID (SCHED_OTHER, core 3) bin=$bin" >> "$LOG_FILE"
    fi
    return 0
}

IOL_MASTER_PID=""
if [ -d "$IOL_SRC_REPO" ] || [ -d "$IOL_HOME" ]; then
    echo "$(date): Starting IOL Master Application..." >> "$LOG_FILE"
    sync_and_build_iol_master
    launch_iol_master || true
    if [ -n "$IOL_MASTER_PID" ]; then
        # Wait for IOL master to be ready
        for i in {1..10}; do
            if nc -z localhost 12011 2>/dev/null; then
                echo "$(date): IOL Master ready (TCP 12011 listening)" >> "$LOG_FILE"
                break
            fi
            sleep 1
        done
    fi
fi

# Start the dashboard
echo "$(date): Starting Dashboard GUI..." >> "$LOG_FILE"
cd "$SCRIPT_DIR"
python3 dashboard.py > >(python3 -u "$LOG_FILTER" "$LOG_FILE") 2>&1 &
DASHBOARD_PID=$!
echo "$(date): Dashboard PID: $DASHBOARD_PID" >> "$LOG_FILE"

# Keep disabling screen blanking periodically, and supervise the IOL master:
# if it exits (the i-link stack aborts on internal asserts), relaunch it with a
# short backoff instead of leaving the dashboard polling a dead port forever.
IOL_RESTARTS=0
(
    tick=0
    while kill -0 $DASHBOARD_PID 2>/dev/null; do
        sleep 5
        tick=$((tick + 5))
        if [ $((tick % 120)) -eq 0 ]; then disable_screensaver; fi
        if [ -n "$IOL_MASTER_PID" ] && ! kill -0 "$IOL_MASTER_PID" 2>/dev/null; then
            IOL_RESTARTS=$((IOL_RESTARTS + 1))
            echo "$(date): WARNING: IOL master (pid $IOL_MASTER_PID) exited; relaunch #$IOL_RESTARTS" >> "$LOG_FILE"
            sleep $(( IOL_RESTARTS < 6 ? IOL_RESTARTS * 2 : 10 ))
            launch_iol_master || true
        fi
    done
) &
SUPERVISOR_PID=$!

# Wait for dashboard to exit
wait $DASHBOARD_PID
EXIT_CODE=$?

echo "$(date): Dashboard exited with code $EXIT_CODE" >> "$LOG_FILE"

# Clean up the supervisor and IOL master if we started them
kill $SUPERVISOR_PID 2>/dev/null || true
# The supervisor may have relaunched the master (different PID, child of the
# subshell); stop any master this box is running.
pkill -f 'iol-master-appl -m0' 2>/dev/null || true
if [ -n "$IOL_MASTER_PID" ]; then
    echo "$(date): Stopping IOL Master..." >> "$LOG_FILE"
    kill $IOL_MASTER_PID 2>/dev/null || true
fi

exit $EXIT_CODE
