# AGENTS.md — working on the TrailerSync boxes (Big-Beautiful-Box / BBB)

This repo is the software that runs on the ~15 Raspberry Pi **TrailerSync boxes**
flown/driven by real ag-aviation crews (BLE + WiFi link to the RotorSync iPad
app, chemical fill control, tank sensors, a dashboard touchscreen). A shipped
bug can disrupt field operations. Read this before changing anything.

## The north star: everything must be fixable via a normal in-app update

The boxes are in the field. The **only** scalable way to fix or improve a box is
the in-app updater (`run_update` in `dashboard.py`, triggered from the box's
dashboard screen — it `git fetch`es `origin/master` and runs a deploy step).
**Design every change so it lands through that path with no one SSHing into a
box.** When you add a fix, ask: "does a box get this by just tapping Update?" If
the answer needs a human at the box or on SSH, you haven't finished the job —
close the gap (see the self-heal patterns below) or document exactly why not.

## UPDATE-PATH PITFALLS (the ones that bite)

1. **The updater runs the OLD deploy code (one-update lag).** `run_update`
   executes the `deploy_cmd` from the *currently installed* `dashboard.py`, then
   `git`-updates the tree. So a change you make to the **deploy step itself**
   only takes effect on the update AFTER the one that delivers it. Corollary: a
   fix that must run *during* the same update that delivers it (e.g. the
   bumble system-install that prevents a BLE crash-loop) cannot rely on being in
   the new deploy step — it has to already be in the deploy step of the version
   the box is coming *from*, or be idempotent/self-healing on the next cycle.

2. **Git-current but `/opt`-stale (cloned / hand-pulled boxes).** The updater
   judged "up to date" from git commit count alone and **skipped the deploy** —
   so a box whose git tree was advanced out-of-band (SD-card clone, a manual
   `git pull`, or a half-finished update) reported current forever while its
   RUNNING code in `/opt` stayed old. `rotorsync_bumble.py` + `src/` run from
   `/opt` (the dashboard/updater run from the repo), so this surfaced as old BLE
   code with a hard-pinned adapter MAC on a cloned box. The updater now
   drift-checks `/opt` vs the repo and **re-deploys even when git is current**
   (`_deployed_runtime_stale` in `run_update`) — a stale box self-heals on its
   next update. Prefer a clean `install.sh` over cloning; if you must clone, run
   a full update/deploy after (never just `git pull`).

3. **Stale git origin → "Already up to date" forever.** Many field boxes have
   `origin` pointing at a stale LOCAL mirror (e.g. `/home/pi/Big-Beautiful-box`),
   not GitHub, so `git fetch` never advances and the updater silently no-ops.
   Fix: `git -C /home/pi/Big-Beautiful-Box remote set-url origin
   https://github.com/RotorSync/Big-Beautiful-Box.git` (anonymous fetch works;
   the repo is public). A fresh `install.sh` provision forces this correctly, so
   only pre-existing field boxes are affected. **Symptom:** the box's VERSION
   won't advance after a crew "updates." **Remote fix without a site visit:** the
   admin maintenance shell (see below) — one command, no SSH.

4. **Bumble-as-root BLE crash-loop (latent, surfaces on restart/reboot).**
   `rotorsync.service` runs as ROOT for HCI access; if `bumble` is only in pi's
   `~/.local`, root can't import it → `No module named bumble` crash-loop that is
   **invisible until the next service restart/reboot** (which an update performs).
   The cure is `pip install --break-system-packages ... bumble==<pinned>`
   **system-wide** in both `install.sh` and the updater deploy step (already
   there). A fresh install is immune (system-wide from the start). A box coming
   from *pre-system-install* firmware runs its OLD deploy (no system bumble) then
   restarts → can crash-loop until that same update finishes installing it.

5. **`websockets` library version variance.** The fleet historically ran
   `websockets==10.4` (legacy asyncio impl). `>=14` removed that impl; the code
   works on both, but **pin `websockets==10.4` in any pip fallback** so a box
   doesn't silently jump to an incompatible API. Some boxes now run 15/16 — test
   changes against both the legacy and new impl if you touch the WS server.

6. **The deploy_cmd is one giant `sudo bash -lc "<string>"`.** It's assembled by
   string-concatenation in `dashboard.py` (`run_update`). Heredocs work inside it
   (there are `python3 - <<'PY'` blocks) but quoting is fragile — a stray quote
   breaks the whole deploy. When you add a step: mirror an existing block's
   style, keep it **fail-soft** (`|| true` / try-except) so one step can't abort
   the update and half-provision a box, and **verify it composes on a real box**
   (run the block via `sudo bash -lc` on sn009) before shipping.

7. **Maintenance secret is FIRST-WINS-FOREVER.** A box persists the first admin
   maintenance key it ever adopts (`_install_maintenance_secret` refuses to
   overwrite) and then can only verify frames signed with THAT key — so a box
   that adopted a wrong/retired key is locked out of the maintenance shell with
   no in-band recovery. **Never put a real secret in this repo (it is PUBLIC).**
   To retire a key fleet-wide, add its **sha256 fingerprint** to
   `deploy/retired-maintenance-secrets.txt`; the updater purges a matching
   persisted secret so the box re-adopts the current shared key on its next
   maintenance session. To provision a box with a key at install time:
   `BBB_MAINTENANCE_SECRET='<key>' ./install.sh`.

7b. **The IOL master binary is built by `start_iol_dashboard.sh`, not by the updater.**
   The updater's deploy step never touched `~/iol-hat`; until V2.50 every box ran the DEBUG
   binary `install.sh` built once. The start script now syncs `iol-hat/src-master-application`
   from the repo into `~/iol-hat`, builds `bin/release` when the source changed, prefers it,
   and relaunches the master if it exits. A change to the vendored master therefore lands on
   the update that delivers it (the script runs from the repo). Never point the launch back at
   `bin/debug` in the field: its per-cycle logging through the log pipe is what caused the
   2026 flow-meter disconnect/pump-stop reports. Do not shrink the OPERATE watchdog in
   `iolink_dl.c` below ~1 s.

## NON-UPDATE PITFALLS (runtime / networking)

8. **Field AP must stay LOCAL-ONLY.** `ipv4.method shared` hands out a default
   route + DNS by default, which makes a joined iPad try to route *internet*
   through the box and blackhole (cellular iPads lose live MQTT). `ensure_ap_profile`
   installs a dnsmasq drop-in (`dhcp-option=3`/`6` cleared, `port=0`) to make the
   AP a routeless local network. Don't regress that. Verify after any AP change:
   join the SSID and confirm a `10.42.0.x` lease with **no** default route.

9. **AP SSID goes stale after a trailer rename.** The AP broadcasts the SSID it
   was *created* with; a renamed trailer's box keeps the old SSID (so the app's
   auto-join, which targets the current name, never joins). `ensure_ap_profile`
   re-syncs the SSID to the current name — but only when it runs (service start /
   AP cycle). `AP_SSID` is computed at import, so a rename while rotorlink is
   running needs a service restart to take effect.

10. **mDNS goes stale across the AP↔STA flip.** The flip changes wlan0's IP; a
   python-zeroconf registration only re-registers on a NAME change. The NM
   dispatcher `deploy/90-rotorlink-readvertise` restarts rotorlink on wlan0-up to
   re-advertise. It must be installed both by `install.sh` (directly) and the
   fleet hook (`setup-cursor-control.sh`). Boxes advertise via **avahi** OR
   **python-zeroconf** depending on what's installed — avahi tracks IP changes
   itself and is immune; check `journalctl -u rotorlink | grep "mDNS advertised
   via"` to know which a box uses.

11. **Single radio can't AP-2.4 and STA-5GHz simultaneously.** The Pi has one
    WiFi radio; the network manager idle-gates the AP↔STA switch (won't flip
    while a client is connected). Don't assume a box can host its AP and be on
    the shop WiFi at once.

12. **Fast shutdown matters.** The rotorlink WS server aborts client transports
    on stop and the unit sets `TimeoutStopSec`/`KillMode` so `systemctl restart`
    doesn't hang ~40s on a half-open client. Broadcasts are per-client isolated
    (`asyncio.wait_for`) so one wedged iPad can't freeze state/history for the
    rest. Preserve both if you touch the server loop.

13. **The display redraw chain must never die - and logging must never kill
    it.** `update_dashboard()` is the screen's only redraw driver, a self-
    rescheduling Tk after-chain; the process, BLE, and the :9999 listener all
    survive a dead chain, so the field symptom is "screen frozen but the app
    still shows flow." The guards (keep all of them): the tick body is wrapped
    and the reschedule lives in `finally`; stdout/stderr go through
    `_ResilientStream` so a dead/stalled log pipe can't make `print()` raise or
    block the mainloop; `src/log_filter.py` must ALWAYS keep draining stdin
    even when its log file is unwritable; the flow-control thread revives a
    stale chain in place via `root.after` (heartbeat: `display_tick_age_s` in
    STATE_JSON). Never add an unguarded file write or blocking call inside the
    tick, and never "fix" a frozen display by restarting the service - that
    blanks the screen and drops pending-fill state mid-fill.

## THE MAINTENANCE SHELL = your remote hands

The admin app's maintenance terminal gives a remote root PTY on any box whose
iPad bridge is reachable (BLE or the RotorLink WiFi link). This is how you fix
stale-origin boxes, clear a bad key, or inspect a box **without a site visit** —
lean on it instead of designing fixes that need SSH. It needs a matching
maintenance secret on the box (see #6).

## VERIFY LOOP (do this — boxes are hard to get back)

- **Dev box: `trailersync-sn009`** — reachable at **eth `192.168.68.191`**
  (control it over ethernet so your session survives an AP↔STA flip) and wlan
  `192.168.68.193` when on STA. `ssh pi@…`, password `raspi`
  (`sshpass -p raspi` or a password file — a password ending in `!` needs a
  file). You may `systemctl restart rotorlink.service` on it; **never reboot** a
  box you can't physically reach, and never restart the dashboard/IOL services on
  a box a crew is using.
- Run the repo's offline test suites (a venv with `websockets==10.4`):
  `tests/test_unit.py`, `test_state_encoder.py`, `test_translate.py`, plus any
  `test_*` you added. `bash -n install.sh` / `python3 -m py_compile dashboard.py`
  for syntax.
- For an updater/deploy change: run the exact block via `sudo bash -lc` on sn009
  and confirm it does the right thing AND is a safe no-op on the healthy case
  (e.g. a key purge must NOT delete a good key).
- Before greenlighting a fleet update: confirm sn009 fetches `origin/master`,
  ff's to it, and restarts rotorlink healthy.

## HOUSE RULES

- **Never commit a secret** — this repo is public. Fingerprints (sha256) only.
- Keep every deploy/install step **idempotent and fail-soft** so a half-run can't
  brick a box and a re-run is safe.
- **Bump `VERSION`** (top-level file, e.g. `V2.23`) when shipping fleet changes —
  crews and the admin read it to know a box actually updated. It's the one signal
  that an update landed vs a stale-origin no-op.
- Commit messages: WHAT changed + WHY (root cause for fixes). End with a
  `Co-Authored-By:` trailer.
- Concurrent agents/devs can break the working tree mid-session — verify in
  isolation (fresh clone or a detached checkout) when in doubt.
- Push to `origin master`; boxes only pick it up on their next in-app update, so
  pushing is safe and does not touch running boxes.
