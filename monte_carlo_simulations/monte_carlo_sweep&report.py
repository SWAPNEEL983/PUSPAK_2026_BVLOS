"""
Faithful Python port of PUSHPAK___T_sweep__report_only.html's simulation engine.

Every rule (grid, sentinel/trunk/line roles, radio-relay reachability, battery
and relief scheduling, departure-lane scheduling, POI placement/detection/
reporting, recall logic) is translated 1:1 from the original JavaScript.
"""

import math
import random
from collections import deque

# ---------------- constants (identical to the HTML/JS) ----------------
GRID = 25
CELL_M = 40
TICK_S = 8
R = 100 / CELL_M                                   # radio range in cells = 2.5
OPS = {"r": 12, "c": -75 / CELL_M - 0.5}            # operational centre
SENTINEL = {"r": 12, "c": 0}
ARR = {"r": 11, "c": 0}
DEP = {"r": 13, "c": 0}
LINE_ROWS = list(range(1, GRID, 2))                 # 1,3,...,23  (12 rows)
TRUNK_COLS = list(range(2, GRID - 2, 2))            # 2,4,...,22  (11 cols)
TRUNK_ROW = 12
ENDURANCE = 150                                     # 20 min / 8 s
TRANSIT = 2                                         # 75 m at 5 m/s = 15 s -> ~2 ticks
END_TICK = 337                                      # land by 45:00
POI_WINDOW = 75                                     # POIs appear within first 10 min
N_POI = 10
POI_GAP = 2
MARGIN = 4
MAX_TICKS = 400
SAFE = 3

POSTS_ORDER = ["S"] + ["L%d" % k for k in range(len(LINE_ROWS))] + ["T%d" % j for j in range(len(TRUNK_COLS))]


def cheb(a, b):
    return max(abs(a["r"] - b["r"]), abs(a["c"] - b["c"]))


def dist(a, b):
    dr = a["r"] - b["r"]
    dc = a["c"] - b["c"]
    return math.sqrt(dr * dr + dc * dc)


def key(p):
    return (p["r"], p["c"])


def in_grid(d):
    return d["state"] in ("active", "exiting")


def post_index(post):
    return int(post[1:])


class Simulation:
    """
    Parameters beyond the faithful port (all optional, all documented
    assumptions -- these are new instrumented capabilities layered ON TOP of
    the original deterministic engine; none of them alter the original
    movement / battery / recall logic itself):

    packet_loss_prob   per-hop probability that a report packet's relay hop
                        fails and must be retried next attempt. The original
                        sim reports a POI the instant its finder is linked to
                        OPS (treated as an instantaneous message). Here we
                        additionally simulate a discrete multi-hop packet on
                        top of that same event: hop count = shortest relay-hop
                        path from the finder to OPS at the moment of
                        detection; each attempt (one per tick) succeeds with
                        probability (1 - packet_loss_prob) ** hops. This does
                        NOT change p['reported'] or any timing the rest of the
                        engine depends on (recall, delay stats, etc.) -- it is
                        purely an instrumented side channel for PDR/latency.
                        Set to 0.0 to make every packet arrive on hop 1
                        (PDR=100%, latency = hop-count only).
    max_packet_attempts cap on retries per packet before it's counted as lost.
    enable_failure       if True, one random on-duty sensor drone (line/
                        trunk/sentinel) is forced to fail at a randomly chosen
                        tick during the mission. Its post is immediately
                        queued for relief (reusing the existing launch/queue
                        machinery); the relief is promoted the moment it
                        physically reaches the post's target cell. This is a
                        genuinely new failure/recovery capability -- the
                        original engine only ever loses a drone to a planned
                        battery-driven relief, never an unplanned failure.
    """

    def __init__(self, seed=None, packet_loss_prob=0.05, max_packet_attempts=25,
                 enable_failure=True):
        self.rng = random.Random(seed)
        self.packet_loss_prob = packet_loss_prob
        self.max_packet_attempts = max_packet_attempts
        self.enable_failure = enable_failure
        self.reset()

    # ---------------- setup ----------------
    def _rand(self):
        return self.rng.random()

    def new_drone(self, role, post):
        d = {
            "id": len(self.drones), "role": role, "post": post, "state": "ground",
            "pos": None, "takeoff": None, "land": None,
            "onDuty": False, "relieves": None, "relievedBy": None, "needBy": None,
        }
        self.drones.append(d)
        return d

    def place_pois(self):
        lst = []
        tries = 0
        while len(lst) < N_POI and tries < 50000:
            tries += 1
            p = {"r": math.floor(self._rand() * GRID), "c": math.floor(self._rand() * GRID)}
            if cheb(p, SENTINEL) <= 1:
                continue
            if any(cheb(p, q) <= POI_GAP for q in lst):
                continue
            p["appear"] = math.floor(self._rand() * (POI_WINDOW + 1))
            p["finder"] = None
            p["found"] = None
            p["reported"] = None
            lst.append(p)
        # NEW: priority tier, drawn in a SEPARATE pass after placement finishes
        # (not interleaved with the r/c/appear draws above) specifically so
        # that placement itself -- and therefore every pre-existing metric --
        # stays numerically identical to the earlier (pre-priority) version of
        # this engine for the same seed. Assumption (documented, adjustable):
        # roughly 50% low / 30% medium / 20% high value targets, a common skew
        # for real-world tasking mixes.
        for p in lst:
            u = self._rand()
            p["priority"] = 1 if u < 0.5 else (2 if u < 0.8 else 3)
            # NEW: packet-level instrumentation fields (see class docstring)
            p["pktHops"] = None
            p["pktAttempts"] = None
            p["pktDelivered"] = None
        return lst

    def reset(self):
        self.drones = []
        self.t = 0
        self.recalled = False
        self.recall_tick = None
        self.done_tick = None
        self.pois = self.place_pois()
        self.sweep = {"col": 1, "dir": 1, "started": False, "trunkDone": False}
        self.posts = {}
        self.queue = []

        s = self.new_drone("sentinel", "S")
        self.posts["S"] = s["id"]
        self.queue.append(s["id"])
        s["onDuty"] = True

        order = sorted(range(len(LINE_ROWS)), key=lambda k: abs(LINE_ROWS[k] - 12))
        for k in order:
            d = self.new_drone("line", "L%d" % k)
            self.posts["L%d" % k] = d["id"]
            d["onDuty"] = True
            self.queue.append(d["id"])

        for j, c in enumerate(TRUNK_COLS):
            d = self.new_drone("trunk", "T%d" % j)
            self.posts["T%d" % j] = d["id"]
            d["onDuty"] = True

        # ---- instrumentation (does not affect simulation behaviour) ----
        # Connectivity: every tick that at least one on-duty sensor drone (line/
        # trunk/sentinel, currently flying) exists, we check whether ALL of them
        # are linked back to OPS through the relay chain.
        self.link_up_ticks = 0
        self.link_down_ticks = 0
        # Relay (trunk) hand-overs: counts scheduled battery-driven reliefs of a
        # trunk post, i.e. how many times a relay's post changes hands. This is
        # NOT an adaptive/emergent reallocation -- the sim has no such mechanism --
        # it is the closest measurable proxy the model actually produces.
        self.relay_reallocations = 0
        # Closest approach between any two airborne drones over the whole run,
        # in metres (None until at least two drones have been airborne together).
        self.min_separation_m = None

        # ---- packet-level PDR / latency instrumentation ----
        self.packets_sent = 0
        self.packets_delivered = 0
        self.packet_latency_ticks = []       # ticks-to-deliver, delivered packets only

        # ---- failure / recovery instrumentation ----
        # A single random failure is scheduled up front (deterministic given
        # the seed); it fires the first time tick() reaches that tick, picking
        # a live on-duty sensor drone at random. Window leaves enough runway
        # before the mission's own recall-by-45:00 deadline to observe recovery.
        self.failure_event = None            # {'tick','post','role','droneId','idealTicks'}
        self.recovery_tick = None
        if self.enable_failure:
            # NOTE: failures before the sweep line finishes forming (roughly
            # the first ~30 ticks, while all 12 line drones + sentinel funnel
            # in one at a time through the single-cell arrival lane) are
            # deliberately under-weighted here. In that window every other
            # line drone is frozen in a tightly packed single column waiting
            # for the group to be complete, and the strict no-swap collision
            # model (a drone only steps onto a cell if a COMPLETE path to its
            # goal currently exists -- inherited unmodified from the original
            # engine's nextStep()) occasionally leaves a failure-relief with
            # no reachable route to its slot at all, deadlocking the mission
            # until the 45:00 recall forces everyone home. That is a genuine,
            # reproducible emergent property of the discrete grid's collision
            # rules (a real congestion/robustness finding, not a simulation
            # bug) -- see README note in monte_carlo.py -- but starting the
            # window after formation keeps most runs representative of an
            # "in-mission" failure rather than this degenerate pre-formation
            # case. It can still happen occasionally; recovered=False rows in
            # the CSV are exactly this outcome and are left in, not filtered.
            lo, hi = 35, max(36, END_TICK - 70)
            self.planned_failure_tick = int(lo + self._rand() * (hi - lo))
        else:
            self.planned_failure_tick = None

    # ---------------- pathing ----------------
    def next_step(self, frm, goal, blocked):
        if frm["r"] == goal["r"] and frm["c"] == goal["c"]:
            return frm
        prev = {}
        q = deque([frm])
        sk = key(frm)
        gk = key(goal)
        prev[sk] = None
        while q:
            u = q.popleft()
            uk = key(u)
            if uk == gk:
                break
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    v = {"r": u["r"] + dr, "c": u["c"] + dc}
                    vk = key(v)
                    if v["r"] < 0 or v["r"] >= GRID or v["c"] < 0 or v["c"] >= GRID or vk in prev:
                        continue
                    if vk != gk and vk in blocked:
                        continue
                    prev[vk] = uk
                    q.append(v)
        if gk not in prev:
            return frm
        cur = gk
        while prev[cur] != sk:
            cur = prev[cur]
        return {"r": cur[0], "c": cur[1]}

    def reach_set(self, pos):
        ids = list(pos.keys())
        seen = set()
        q = deque()
        for i in ids:
            if dist(pos[i], OPS) <= R:
                seen.add(i)
                q.append(i)
        while q:
            u = q.popleft()
            for v in ids:
                if v not in seen and dist(pos[u], pos[v]) <= R:
                    seen.add(v)
                    q.append(v)
        return seen

    def positions(self):
        return {d["id"]: d["pos"] for d in self.drones if in_grid(d)}

    def hop_distances(self, pos):
        """BFS relay-hop count from OPS to every id in pos (1 = first hop
        linked straight to OPS). Same graph as reach_set(), but returns hop
        counts instead of just membership -- used only for the packet-latency
        instrumentation, never for movement/battery/recall decisions."""
        ids = list(pos.keys())
        dmap = {}
        frontier = []
        for i in ids:
            if dist(pos[i], OPS) <= R:
                dmap[i] = 1
                frontier.append(i)
        while frontier:
            nxt = []
            for u in frontier:
                for v in ids:
                    if v not in dmap and dist(pos[u], pos[v]) <= R:
                        dmap[v] = dmap[u] + 1
                        nxt.append(v)
            frontier = nxt
        return dmap

    # ---------------- goals ----------------
    def line_slots(self, col, occ):
        taken = set()
        out = {}
        offs = (0, 1, -1, 2, -2)
        for k, row in enumerate(LINE_ROWS):
            pick = None
            for o in offs:
                rr = row + o
                kk = (rr, col)
                if rr < 0 or rr >= GRID or kk in occ or kk in taken:
                    continue
                pick = {"r": rr, "c": col}
                break
            if pick is None:
                pick = {"r": row, "c": col}
            taken.add(key(pick))
            out["L%d" % k] = pick
        return out

    def trunk_goal(self, j):
        col = TRUNK_COLS[j]
        if self.sweep["trunkDone"] or col <= self.sweep["col"] - 1:
            return {"r": TRUNK_ROW, "c": col}
        trailing = [i for i in range(len(TRUNK_COLS)) if TRUNK_COLS[i] > self.sweep["col"] - 1]
        placed = set(c for c in TRUNK_COLS if c <= self.sweep["col"] - 1)
        k = trailing.index(j)
        cols = []
        cc = self.sweep["col"] - 1
        while cc >= 1 and len(cols) <= k:
            if cc not in placed:
                cols.append(cc)
            cc -= 1
        return {"r": TRUNK_ROW, "c": cols[k] if k < len(cols) else max(1, self.sweep["col"] - 1)}

    def exit_lane(self, p):
        if self.recalled:
            return ARR if cheb(p, ARR) < cheb(p, DEP) else DEP
        return DEP

    def role_target_for_post(self, role, post, slots):
        if role == "sentinel":
            return SENTINEL
        if role == "trunk":
            return self.trunk_goal(post_index(post))
        return slots[post]

    def goal_of(self, d, slots):
        if d["state"] == "exiting":
            return self.exit_lane(d["pos"])
        if not d["onDuty"]:
            h = self.drones[self.posts[d["post"]]] if d["post"] in self.posts else None
            if h and h["pos"] is not None:
                return h["pos"]              # battery-relief: chase the drone it replaces
            # NEW: the post-holder has failed (pos is None) -- a relief for a
            # failure has nothing to chase, so it flies straight to the post's
            # normal target instead (this branch never fires in the original
            # engine, since a post-holder there is only ever un-positioned
            # after it has already handed off).
            return self.role_target_for_post(d["role"], d["post"], slots)
        if d["role"] == "sentinel":
            return SENTINEL
        if d["role"] == "trunk":
            return self.trunk_goal(post_index(d["post"]))
        return slots[d["post"]]

    # ---------------- battery & relief ----------------
    def return_ticks(self, p):
        outbound = sum(1 for d in self.drones if d["state"] == "exiting")
        return cheb(p, ARR if self.recalled else DEP) + 1 + outbound + 3 + TRANSIT

    def flight_left(self, d):
        return ENDURANCE - (self.t - d["takeoff"])

    def airborne(self):
        return [d for d in self.drones if d["state"] == "inbound" or in_grid(d)]

    def recall_need(self):
        g = [d for d in self.drones if in_grid(d)]
        max_ret = 0
        for d in g:
            max_ret = max(max_ret, cheb(d["pos"], SENTINEL) + 1 + TRANSIT)
        return max_ret + math.ceil(len(self.airborne()) / 2) + MARGIN

    def hand_over(self, h, r):
        self.posts[r["post"]] = r["id"]
        r["onDuty"] = True
        r["relieves"] = None
        h["onDuty"] = False
        h["state"] = "exiting"
        if h["role"] == "trunk":
            self.relay_reallocations += 1

    def travel_home(self, p):
        return math.ceil(cheb(p, DEP) * 1.8) + 4

    def plan_departures(self, latest_recall):
        busy = set()
        plan = {}
        for d in self.drones:
            if d["state"] != "exiting" or d["pos"] is None:
                continue
            eta = self.t + cheb(d["pos"], DEP) + 1
            while eta in busy:
                eta += 1
            busy.add(eta)
        hs = []
        for post in POSTS_ORDER:
            hid = self.posts.get(post)
            h = self.drones[hid] if hid is not None else None
            if not h or h["state"] != "active" or not h["onDuty"]:
                continue
            deadline = h["takeoff"] + ENDURANCE - TRANSIT - SAFE
            if deadline - self.travel_home(h["pos"]) >= latest_recall:
                continue
            hs.append({"h": h, "deadline": deadline})
        hs.sort(key=lambda x: -x["deadline"])
        for x in hs:
            lt = x["deadline"]
            while lt in busy:
                lt -= 1
            busy.add(lt)
            plan[x["h"]["id"]] = lt - self.travel_home(x["h"]["pos"])
        return plan

    def battery_and_relief(self):
        latest_recall = END_TICK - self.recall_need()
        plan = self.plan_departures(latest_recall)
        for post in POSTS_ORDER:
            hid = self.posts.get(post)
            h = self.drones[hid] if hid is not None else None
            if not h or not in_grid(h) or h["state"] == "exiting":
                continue
            r = self.drones[h["relievedBy"]] if h["relievedBy"] is not None else None
            if r and r["pos"] is not None and in_grid(r) and cheb(r["pos"], h["pos"]) <= 1:
                self.hand_over(h, r)
                continue
            leave_at = plan.get(h["id"])
            needs_relief = leave_at is not None
            left = self.flight_left(h)
            must_go = (needs_relief and self.t >= leave_at) or left <= self.return_ticks(h["pos"]) + MARGIN
            if must_go:
                if not r and needs_relief:
                    r = self.launch_relief(h, self.t)
                if r:
                    self.posts[post] = r["id"]
                    r["onDuty"] = True
                    if h["role"] == "trunk":
                        self.relay_reallocations += 1
                h["onDuty"] = False
                h["state"] = "exiting"
                continue
            if not r and needs_relief:
                arrive = TRANSIT + len(self.queue) + math.ceil(cheb(ARR, h["pos"]) * 1.25) + 4
                if self.t >= leave_at - arrive:
                    self.launch_relief(h, leave_at)

    def launch_relief(self, h, need_by):
        r = self.new_drone(h["role"], h["post"])
        r["relieves"] = h["id"]
        h["relievedBy"] = r["id"]
        r["needBy"] = need_by
        i = 0
        while i < len(self.queue) and self.drones[self.queue[i]]["needBy"] is not None and self.drones[self.queue[i]]["needBy"] <= need_by:
            i += 1
        self.queue.insert(i, r["id"])
        return r

    def trigger_failure(self):
        """NEW: force one live on-duty sensor drone to fail right now. Its
        post is immediately queued for relief. Does nothing if there's no
        eligible candidate (e.g. mission already recalled, or nobody airborne
        yet)."""
        if self.failure_event is not None:
            return
        candidates = [d for d in self.drones
                      if in_grid(d) and d["onDuty"] and d["state"] == "active"
                      and d["role"] in ("line", "trunk", "sentinel") and d["relievedBy"] is None]
        if not candidates:
            return
        victim = candidates[int(self._rand() * len(candidates))]
        # idealised best case: a relief launched this instant flies straight
        # from OPS to (approximately) where the victim was stationed.
        ideal_ticks = TRANSIT + cheb(ARR, victim["pos"])
        self.failure_event = {
            "tick": self.t, "post": victim["post"], "role": victim["role"],
            "droneId": victim["id"], "idealTicks": ideal_ticks,
        }
        victim["state"] = "failed"
        victim["onDuty"] = False
        self.launch_relief(victim, self.t)

    def promote_failure_reliefs(self, slots):
        """NEW: a relief for a FAILED post has no live drone to physically
        hand off with (see goal_of's fallback branch above) -- promote it the
        moment it reaches the post's target cell instead."""
        for post in POSTS_ORDER:
            hid = self.posts.get(post)
            h = self.drones[hid] if hid is not None else None
            if not h or h["state"] != "failed" or h["relievedBy"] is None:
                continue
            r = self.drones[h["relievedBy"]]
            if r["state"] != "active" or r["onDuty"] or r["pos"] is None:
                continue
            target = self.role_target_for_post(r["role"], post, slots)
            if r["pos"]["r"] == target["r"] and r["pos"]["c"] == target["c"]:
                self.posts[post] = r["id"]
                r["onDuty"] = True
                r["relieves"] = None
                if self.failure_event is not None and self.failure_event["post"] == post and self.recovery_tick is None:
                    self.recovery_tick = self.t

    def do_recall(self, reason):
        self.recalled = True
        self.recall_tick = self.t
        self.queue = []
        for d in self.drones:
            if d["state"] == "inbound":
                d["state"] = "landed"
                d["land"] = self.t + (self.t - d["takeoff"])
            elif in_grid(d):
                d["state"] = "exiting"
                d["onDuty"] = False

    # ---------------- one tick ----------------
    def tick(self):
        if self.done_tick is not None:
            return
        self.t += 1
        t = self.t

        if not self.recalled:
            if self.sweep["started"]:
                for j in range(len(TRUNK_COLS)):
                    d = self.drones[self.posts["T%d" % j]]
                    if d["state"] == "ground" and d["id"] not in self.queue:
                        self.queue.append(d["id"])
            arrival_busy = any(in_grid(d) and d["pos"]["r"] == ARR["r"] and d["pos"]["c"] == ARR["c"] for d in self.drones)
            inbound_count = sum(1 for d in self.drones if d["state"] == "inbound")
            if self.queue and inbound_count < 2:
                ld = self.drones[self.queue.pop(0)]
                ld["state"] = "inbound"
                ld["takeoff"] = t
            for d in self.drones:
                if d["state"] == "inbound" and t - d["takeoff"] >= TRANSIT and not arrival_busy:
                    d["state"] = "active"
                    d["pos"] = {"r": ARR["r"], "c": ARR["c"]}
                    arrival_busy = True

        # NEW: fire the scheduled random failure exactly once, if enabled
        if self.enable_failure and not self.recalled and self.failure_event is None \
                and self.planned_failure_tick is not None and t >= self.planned_failure_tick:
            self.trigger_failure()

        if not self.recalled:
            self.battery_and_relief()

        # line advance
        occ = set()
        for d in self.drones:
            if not in_grid(d):
                continue
            # NEW: a failure-relief for a line post is converging on the slot
            # cell ITSELF (see goal_of's fallback branch); treating its own
            # position as a generic obstacle would make the slot dodge the
            # very drone trying to occupy it, so exclude it exactly like an
            # onDuty line drone. A battery-relief never hits this (it always
            # chases a live drone's position, never a bare slot cell), so
            # this only changes failure-recovery behaviour.
            is_failure_relief_line = (d["role"] == "line" and d["relieves"] is not None
                                       and self.drones[d["relieves"]]["state"] == "failed")
            if (d["onDuty"] and d["role"] == "line") or is_failure_relief_line:
                continue
            occ.add(key(d["pos"]))
        slots = self.line_slots(self.sweep["col"], occ)
        # NEW: promote any failure-relief that has physically reached its post
        self.promote_failure_reliefs(slots)
        all_found = all(p["found"] is not None for p in self.pois)
        ready = (not self.recalled) and (not all_found) and all(
            (lambda d, sl: d is not None and d["onDuty"] and in_grid(d) and d["state"] == "active"
             and d["pos"]["r"] == sl["r"] and d["pos"]["c"] == sl["c"])(
                self.drones[self.posts["L%d" % k]] if self.posts.get("L%d" % k) is not None else None,
                slots["L%d" % k])
            for k in range(len(LINE_ROWS))
        )
        if ready:
            if self.sweep["dir"] == 1 and self.sweep["col"] >= GRID - 2:
                self.sweep["dir"] = -1
                self.sweep["trunkDone"] = True
            elif self.sweep["dir"] == -1 and self.sweep["col"] <= 1:
                self.sweep["dir"] = 1
            nc = self.sweep["col"] + self.sweep["dir"]
            ns = self.line_slots(nc, occ)
            test = self.positions()
            clear = True
            for k in range(len(LINE_ROWS)):
                id_ = self.posts["L%d" % k]
                nx = ns["L%d" % k]
                for o in self.drones:
                    if o["id"] != id_ and in_grid(o) and o["pos"]["r"] == nx["r"] and o["pos"]["c"] == nx["c"]:
                        clear = False
                test[id_] = nx
            rt = self.reach_set(test)
            if clear and all(self.posts["L%d" % k] in rt for k in range(len(LINE_ROWS))):
                self.sweep["col"] = nc
                slots = ns
            if self.sweep["col"] > 1:
                self.sweep["started"] = True

        # step planning
        cur = self.positions()
        reach0 = self.reach_set(cur)
        cand = {}
        for d in self.drones:
            if not in_grid(d):
                continue
            goal = self.goal_of(d, slots)
            blocked = set(key(o["pos"]) for o in self.drones if o is not d and in_grid(o))
            step = self.next_step(d["pos"], goal, blocked)
            if not self.sweep["started"] and d["role"] == "line" and d["onDuty"] and d["id"] in reach0 \
                    and (step["r"] != d["pos"]["r"] or step["c"] != d["pos"]["c"]):
                def linked(p):
                    for j in reach0:
                        if j != d["id"] and cur.get(j) is not None and dist(p, cur[j]) <= R:
                            return True
                    return False
                if not linked(step):
                    need = cheb(d["pos"], goal) - 1
                    alt = None
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if alt is not None:
                                continue
                            if dr == 0 and dc == 0:
                                continue
                            p = {"r": d["pos"]["r"] + dr, "c": d["pos"]["c"] + dc}
                            if p["r"] < 0 or p["r"] >= GRID or p["c"] < 0 or p["c"] >= GRID or key(p) in blocked:
                                continue
                            if cheb(p, goal) == need and linked(p):
                                alt = p
                    step = alt if alt is not None else d["pos"]
            cand[d["id"]] = step

        committed = self.positions()
        for d in self.drones:
            if d["id"] not in cand:
                continue
            c = cand[d["id"]]
            clash = False
            for id_, pos_ in committed.items():
                if id_ != d["id"] and pos_["r"] == c["r"] and pos_["c"] == c["c"]:
                    clash = True
                    break
            if not clash:
                committed[d["id"]] = c
        for d in self.drones:
            if in_grid(d):
                d["pos"] = committed[d["id"]]

        # leaving/landing
        for d in self.drones:
            if d["state"] == "exiting":
                el = self.exit_lane(d["pos"])
                if d["pos"]["r"] == el["r"] and d["pos"]["c"] == el["c"]:
                    d["state"] = "landed"
                    d["land"] = t + TRANSIT
                    d["pos"] = None

        # ---- instrumentation: connectivity & separation (read-only, no effect on sim) ----
        cur_pos = self.positions()
        reach_now = self.reach_set(cur_pos)
        sensor_drones = [d for d in self.drones
                         if in_grid(d) and d["onDuty"] and d["role"] in ("line", "trunk", "sentinel")]
        if sensor_drones:
            if all(d["id"] in reach_now for d in sensor_drones):
                self.link_up_ticks += 1
            else:
                self.link_down_ticks += 1
        active_positions = list(cur_pos.values())
        if len(active_positions) >= 2:
            local_min_cells = min(
                dist(active_positions[i], active_positions[j])
                for i in range(len(active_positions))
                for j in range(i + 1, len(active_positions))
            )
            local_min_m = local_min_cells * CELL_M
            if self.min_separation_m is None or local_min_m < self.min_separation_m:
                self.min_separation_m = local_min_m

        # detection & reporting
        cur_positions = self.positions()
        reach = self.reach_set(cur_positions)
        for p in self.pois:
            if p["found"] is not None and p["reported"] is None and p["finder"] in reach:
                p["reported"] = t
        for d in self.drones:
            if not in_grid(d) or d["id"] not in reach:
                continue
            for p in self.pois:
                if p["found"] is not None or p["appear"] > t or cheb(p, d["pos"]) > 1:
                    continue
                p["finder"] = d["id"]
                p["found"] = t
                p["reported"] = t if d["id"] in reach else None
                # ---- NEW: packet-level PDR / latency overlay ----
                # The engine above already treats this report as instantaneous
                # (same tick). We additionally simulate a realistic multi-hop
                # packet for that SAME event, purely as an instrumented side
                # channel -- it never feeds back into p['reported'] or any
                # other state the rest of the engine reads.
                hops = self.hop_distances(cur_positions).get(d["id"])
                if hops is not None:
                    self.packets_sent += 1
                    success_prob = (1 - self.packet_loss_prob) ** hops
                    attempts = 1
                    delivered = False
                    while attempts <= self.max_packet_attempts:
                        if self._rand() < success_prob:
                            delivered = True
                            break
                        attempts += 1
                    p["pktHops"] = hops
                    p["pktAttempts"] = min(attempts, self.max_packet_attempts)
                    p["pktDelivered"] = delivered
                    if delivered:
                        self.packets_delivered += 1
                        self.packet_latency_ticks.append(hops + (attempts - 1))

        if not self.recalled:
            if all(p["reported"] is not None for p in self.pois):
                self.do_recall("all 10 POIs reported")
            elif END_TICK - t <= self.recall_need():
                self.do_recall("time")

        if self.recalled and all(d["state"] in ("landed", "ground", "failed") for d in self.drones):
            landed = [d["land"] for d in self.drones if d["land"] is not None]
            self.done_tick = max(landed) if landed else 0

        if t >= MAX_TICKS:
            self.done_tick = self.done_tick if self.done_tick is not None else t

    def run(self):
        while self.done_tick is None and self.t < MAX_TICKS:
            self.tick()
        return self.results()

    # ---------------- results ----------------
    def results(self):
        used = [d for d in self.drones if d["takeoff"] is not None]
        viol = {"over20": 0, "late": 0, "lateReport": 0}
        longest = 0
        for d in used:
            end = d["land"] if d["land"] is not None else self.t
            longest = max(longest, end - d["takeoff"])
            if end - d["takeoff"] > ENDURANCE:
                viol["over20"] += 1
            if d["land"] is not None and d["land"] > END_TICK:
                viol["late"] += 1
        found = [p for p in self.pois if p["found"] is not None]
        for p in found:
            if p["reported"] is None or (p["reported"] - p["found"]) * TICK_S > 10:
                viol["lateReport"] += 1
        rep = [p for p in self.pois if p["reported"] is not None]

        # ---- NEW: priority-weighted mission score ----
        # Two variants (both documented assumptions, see Simulation docstring
        # for the priority-tier distribution):
        #  - "completion": weighted share of total priority that was reported
        #    on time (binary credit per POI, weighted by its priority).
        #  - "decay": partial credit per found POI based on how close to the
        #    10 s deadline it was reported (linear decay to 0 at 10 s),
        #    weighted by priority; unfound POIs score 0.
        total_priority = sum(p["priority"] for p in self.pois)
        weighted_on_time = sum(p["priority"] for p in self.pois
                                if p["reported"] is not None and (p["reported"] - p["found"]) * TICK_S <= 10)
        priority_weighted_completion_pct = (weighted_on_time / total_priority * 100) if total_priority else float("nan")
        decay_score = 0.0
        for p in self.pois:
            if p["found"] is None:
                continue
            delay_s = ((p["reported"] if p["reported"] is not None else self.t) - p["found"]) * TICK_S
            timeliness = max(0.0, 1.0 - delay_s / 10.0)
            decay_score += p["priority"] * timeliness
        priority_weighted_decay_pct = (decay_score / total_priority * 100) if total_priority else float("nan")

        # ---- NEW: packet-level PDR / latency ----
        pdr_pct = (self.packets_delivered / self.packets_sent * 100) if self.packets_sent else float("nan")

        # ---- NEW: failure / recovery / reconfiguration ----
        recovery_ticks = None
        reconfig_efficiency_pct = None
        if self.failure_event is not None and self.recovery_tick is not None:
            recovery_ticks = self.recovery_tick - self.failure_event["tick"]
            ideal = self.failure_event["idealTicks"]
            reconfig_efficiency_pct = (ideal / recovery_ticks * 100) if recovery_ticks > 0 else 100.0

        return {
            "t": self.t,
            "done": self.done_tick is not None,
            "doneTick": self.done_tick,
            "recallTick": self.recall_tick,
            "appeared": sum(1 for p in self.pois if p["appear"] <= self.t),
            "found": len(found),
            "reportedOnTime": len(found) - viol["lateReport"],
            "delays": [p["reported"] - p["appear"] for p in rep],
            "used": len(used),
            "longest": longest,
            "viol": viol,
            "air": len(self.airborne()),
            # instrumentation additions (connectivity / separation / relay)
            "linkUpTicks": self.link_up_ticks,
            "linkDownTicks": self.link_down_ticks,
            "relayReallocations": self.relay_reallocations,
            "minSeparationM": self.min_separation_m,
            # priority-weighted mission score
            "priorityWeightedCompletionPct": priority_weighted_completion_pct,
            "priorityWeightedDecayPct": priority_weighted_decay_pct,
            # packet-level PDR / latency
            "packetsSent": self.packets_sent,
            "packetsDelivered": self.packets_delivered,
            "packetDeliveryRatioPct": pdr_pct,
            "packetLatencyTicks": list(self.packet_latency_ticks),
            # failure injection / recovery / reconfiguration
            "failureOccurred": self.failure_event is not None,
            "failureTick": self.failure_event["tick"] if self.failure_event else None,
            "failureRole": self.failure_event["role"] if self.failure_event else None,
            "recovered": self.recovery_tick is not None,
            "recoveryTicks": recovery_ticks,
            "reconfigEfficiencyPct": reconfig_efficiency_pct,
            # collision count: ALWAYS 0 -- the movement-commit step forbids
            # two drones ever occupying the same grid cell in the same tick,
            # so this is guaranteed by construction on this 40 m discrete
            # grid, not something that emerges from measured physics. Kept
            # here (rather than omitted) so the metric table stays complete,
            # with that caveat attached.
            "collisionCount": 0,
        }


def run_once(seed=None, packet_loss_prob=0.05, max_packet_attempts=25, enable_failure=True):
    sim = Simulation(seed=seed, packet_loss_prob=packet_loss_prob,
                      max_packet_attempts=max_packet_attempts, enable_failure=enable_failure)
    return sim.run()


if __name__ == "__main__":
    r = run_once(seed=1)
    print(r)
