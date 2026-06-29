/* ============================================================================
   widgets.js — offline, vanilla interactive widgets for the SAR defense deck.
   No network, no imports. Every rule/number traces to docs/thesis/main.pdf.
   Driven by the Reveal API (deck.js wires init + slidechanged).
   ========================================================================== */
(function () {
  "use strict";

  const NAVY = "#17304F", RESCUE = "#E8743B", TEAL = "#2A9D8F", GREY = "#9aa3ad";
  const SVGNS = "http://www.w3.org/2000/svg";
  const el = (tag, attrs) => {
    const n = document.createElementNS(SVGNS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    return n;
  };

  /* ==========================================================================
     WIDGET 1 — Animated greedy auction
     Rule (thesis §3, Def. "Assignment auction", eq. 3): every *eligible* drone
     bids u(d,t) = pi * kappa_d / max(||x_d - x_t||, 1); kappa_d = 1 for the
     homogeneous fleet, so u = pi / max(dist,1). Highest bid (= nearest eligible
     drone) wins; ties within tolerance are broken by a seeded, name-sorted draw.
     A low-battery / committed drone is ineligible and never bids.
     ======================================================================== */
  const Auction = (function () {
    const PI = 10;                     // target priority (illustrative)
    const VICT = { x: 56, y: 50 };     // victim position in 0..100 board
    // Drones. D5 sits *closest* but is ineligible (battery low) — the teaching
    // point: proximity does not win if you cannot finish the rescue.
    const DRONES = [
      { id: "D1", x: 39, y: 41, ok: true,  reason: "" },
      { id: "D2", x: 78, y: 31, ok: true,  reason: "" },
      { id: "D3", x: 31, y: 76, ok: true,  reason: "" },
      { id: "D4", x: 82, y: 68, ok: true,  reason: "" },
      { id: "D5", x: 60, y: 33, ok: false, reason: "battery low" }
    ];
    let svg, statusEl, runBtn, step = 0;

    function dist(d) { return Math.hypot(d.x - VICT.x, d.y - VICT.y); }
    function bid(d) { return PI / Math.max(dist(d), 1); }

    function build(container) {
      svg = el("svg", { viewBox: "0 0 100 78", class: "auc-svg" });
      // assignment lines + bid bubbles drawn later into these layers
      const gLines = el("g", { id: "auc-lines" });
      const gDrones = el("g", { id: "auc-drones" });
      const gBids = el("g", { id: "auc-bids" });
      svg.appendChild(gLines); svg.appendChild(gDrones); svg.appendChild(gBids);

      // victim (orange star-ish marker + pulse ring)
      const ring = el("circle", { cx: VICT.x, cy: VICT.y, r: 6, fill: "none",
        stroke: RESCUE, "stroke-width": 0.6, opacity: 0.5, class: "auc-pulse" });
      svg.appendChild(ring);
      svg.appendChild(el("circle", { cx: VICT.x, cy: VICT.y, r: 3.1, fill: RESCUE }));
      const vlab = el("text", { x: VICT.x + 5, y: VICT.y + 1, class: "auc-vlab",
        "text-anchor": "start" }); vlab.textContent = "victim (π=" + PI + ")";
      svg.appendChild(vlab);

      DRONES.forEach((d) => {
        const g = el("g", { class: "auc-drone", "data-id": d.id });
        const c = el("circle", { cx: d.x, cy: d.y, r: 3.4,
          fill: d.ok ? NAVY : "#fff", stroke: d.ok ? NAVY : GREY,
          "stroke-width": d.ok ? 0 : 0.8 });
        const t = el("text", { x: d.x, y: d.y + 1.1, "text-anchor": "middle",
          class: "auc-id", fill: d.ok ? "#fff" : GREY }); t.textContent = d.id;
        g.appendChild(c); g.appendChild(t);
        if (!d.ok) {
          const r = el("text", { x: d.x, y: d.y - 5.5, "text-anchor": "middle",
            class: "auc-x" }); r.textContent = "✗ " + d.reason;
          g.appendChild(r);
        }
        gDrones.appendChild(g);
      });
      container.appendChild(svg);
    }

    function clearTransient() {
      ["auc-lines", "auc-bids"].forEach((id) => {
        const g = svg.querySelector("#" + id);
        while (g.firstChild) g.removeChild(g.firstChild);
      });
      svg.querySelectorAll(".auc-drone").forEach((g) => g.classList.remove("win"));
    }

    function showBids() {
      const gLines = svg.querySelector("#auc-lines");
      const gBids = svg.querySelector("#auc-bids");
      DRONES.forEach((d, i) => {
        if (!d.ok) return;
        const ln = el("line", { x1: d.x, y1: d.y, x2: VICT.x, y2: VICT.y,
          stroke: TEAL, "stroke-width": 0.5, "stroke-dasharray": "1.5 1.2",
          opacity: 0, class: "auc-line" });
        gLines.appendChild(ln);
        // bid bubble at midpoint
        const mx = (d.x + VICT.x) / 2, my = (d.y + VICT.y) / 2;
        const b = el("g", { class: "auc-bubble", opacity: 0 });
        const rect = el("rect", { x: mx - 7, y: my - 3.2, width: 14, height: 6,
          rx: 1.4, fill: "#fff", stroke: TEAL, "stroke-width": 0.4 });
        const tx = el("text", { x: mx, y: my + 1.2, "text-anchor": "middle",
          class: "auc-bid" }); tx.textContent = bid(d).toFixed(2);
        b.appendChild(rect); b.appendChild(tx); gBids.appendChild(b);
        setTimeout(() => { ln.style.opacity = 1; b.style.opacity = 1; }, 90 * i);
      });
    }

    function showWinner() {
      const eligible = DRONES.filter((d) => d.ok);
      let win = eligible[0];
      eligible.forEach((d) => { if (bid(d) > bid(win)) win = d; });
      const g = svg.querySelector('.auc-drone[data-id="' + win.id + '"]');
      g.classList.add("win");
      const gLines = svg.querySelector("#auc-lines");
      const arrow = el("line", { x1: win.x, y1: win.y, x2: VICT.x, y2: VICT.y,
        stroke: RESCUE, "stroke-width": 1.3, class: "auc-assign" });
      gLines.appendChild(arrow);
      statusEl.innerHTML = "<strong>" + win.id + "</strong> wins — highest utility " +
        bid(win).toFixed(2) + " = nearest <em>eligible</em> drone. " +
        "D5 is closer but never bids (battery low). Seeded tie-break would settle any draw.";
    }

    function advance() {
      step++;
      if (step === 1) {
        statusEl.textContent = "Each eligible drone bids u = π / max(dist, 1)…";
        showBids();
        runBtn.textContent = "Pick winner ▸";
      } else if (step === 2) {
        showWinner();
        runBtn.textContent = "↻ Replay";
      } else {
        reset();
      }
    }

    function reset() {
      step = 0;
      clearTransient();
      statusEl.textContent = "A victim is confirmed. Press ▸ to open the auction.";
      runBtn.textContent = "Open auction ▸";
    }

    function init() {
      const container = document.getElementById("auction-stage");
      if (!container || container.dataset.ready) return;
      container.dataset.ready = "1";
      build(container);
      statusEl = document.getElementById("auction-status");
      runBtn = document.getElementById("auction-run");
      const replay = document.getElementById("auction-reset");
      runBtn.addEventListener("click", advance);
      if (replay) replay.addEventListener("click", reset);
      reset();
    }
    return { init: init, reset: reset };
  })();

  /* ==========================================================================
     WIDGET 2 — Canvas stigmergy / coverage sim
     Honest motor-schema physics (thesis §3, Def. "Motor-schema navigation" plus
     the scatter + anti-stall guards of Algorithm "Stigmergic navigation tick").
     A default fleet of FOUR drones (thesis MRS card) searches a disk; each blends
     five basis behaviours into one turn-rate-limited heading:
       b1 avoid-visited    repulsion from high-pheromone cells (the shared field)
       b2 explore-unvisited pull toward low-pheromone cells
       b3 avoid-peers       repulsion from nearby drones
       b4 stay-inside       zero within R/2, ramps to the disk boundary
       b5 goal-seek         pull toward the nearest unfound victim in range
     Surveying DEPOSITS a Gaussian stamp under the sensor footprint; the field
     DECAYS multiplicatively (phi <- beta*phi). Coverage — and the victims it
     uncovers — emerge with NO central planner.
     ======================================================================== */
  const Stigmergy = (function () {
    const GW = 110, GH = 70;            // pheromone grid (cells)
    const N = 4;                        // default fleet size (thesis MRS card)
    const BETA = 0.99;                  // multiplicative field decay per tick
    const SENSOR = 3.2;                 // sensor footprint radius (cells)
    const THETA = 0.16;                 // "visited" threshold (coverage + b1)
    const SCATTER = 42;                 // ticks of the initial scatter phase
    const SPEED = 0.5;                  // cells per tick
    const MAXTURN = 0.30;               // rad/tick -> smooth, quadrotor-like arcs
    const WIN = 7;                      // perception window radius (cells)
    const W = { explore: 1.15, avoid: 0.95, peers: 1.1, inside: 1.5, goal: 0.9 };
    const FPS = 30;
    const CX = GW / 2, CY = GH / 2, RX = GW / 2 * 0.95, RY = GH / 2 * 0.95;
    const VSRC = [[0.46, -0.52], [-0.58, 0.33], [0.12, 0.6]];  // fixed victim layout

    let cv, ctx, phi, seen, drones, victims, regionN = 0;
    let raf = null, running = false, cell = 6, ox = 0, oy = 0, ticks = 0, speed = 1, last = 0;
    let elCov, elBar, elVic, elTime, elSpeed;

    const idx = (x, y) => y * GW + x;
    const norm2 = (x, y) => ((x - CX) / RX) ** 2 + ((y - CY) / RY) ** 2;  // <=1 inside disk
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

    function reset() {
      phi = new Float32Array(GW * GH);
      seen = new Uint8Array(GW * GH);
      ticks = 0; regionN = 0;
      for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) if (norm2(x, y) <= 1) regionN++;
      drones = [];
      for (let i = 0; i < N; i++) {
        const a = (i + 0.5) * (2 * Math.PI / N);   // fan into separate sectors
        drones.push({ x: CX, y: CY, a: a, sa: a });
      }
      victims = VSRC.map(function (p) { return { x: CX + p[0] * RX, y: CY + p[1] * RY, found: false, t: 0 }; });
      if (ctx) { draw(); hud(); }
    }

    function stepDrone(d, i) {
      let vx, vy;
      if (ticks < SCATTER) {                          // scatter: fly own sector outward
        vx = Math.cos(d.sa); vy = Math.sin(d.sa);
      } else {
        let ex = 0, ey = 0, ax = 0, ay = 0;           // b2 explore, b1 avoid-visited
        const cx = Math.round(d.x), cy = Math.round(d.y);
        for (let dy = -WIN; dy <= WIN; dy++) for (let dx = -WIN; dx <= WIN; dx++) {
          const gx = cx + dx, gy = cy + dy;
          if (gx < 0 || gy < 0 || gx >= GW || gy >= GH) continue;
          const r2 = dx * dx + dy * dy; if (r2 === 0 || r2 > WIN * WIN) continue;
          const inv = 1 / Math.sqrt(r2), p = phi[idx(gx, gy)];
          if (p > THETA) { ax -= p * inv * dx * inv; ay -= p * inv * dy * inv; }
          else if (norm2(gx, gy) <= 1) { const w = (THETA - p) * inv; ex += w * dx * inv; ey += w * dy * inv; }
        }
        let px = 0, py = 0;                            // b3 avoid-peers
        drones.forEach(function (o) {
          if (o === d) return;
          const dx = d.x - o.x, dy = d.y - o.y, r = Math.hypot(dx, dy);
          if (r < 10 && r > 0.001) { px += dx / (r * r); py += dy / (r * r); }
        });
        let sx = 0, sy = 0;                            // b4 stay-inside (zero within R/2)
        const rn = Math.sqrt(norm2(d.x, d.y));
        if (rn > 0.5) { const k = (rn - 0.5) * 5; sx = -(d.x - CX) / RX * k; sy = -(d.y - CY) / RY * k; }
        let gxv = 0, gyv = 0, best = null, bd = 1e9;   // b5 goal-seek nearest unfound victim
        victims.forEach(function (v) { if (v.found) return; const r = Math.hypot(v.x - d.x, v.y - d.y); if (r < bd) { bd = r; best = v; } });
        if (best && bd < 22) { const inv = 1 / Math.max(bd, 1); gxv = (best.x - d.x) * inv; gyv = (best.y - d.y) * inv; }
        vx = W.explore * ex + W.avoid * ax + W.peers * px + W.inside * sx + W.goal * gxv;
        vy = W.explore * ey + W.avoid * ay + W.peers * py + W.inside * sy + W.goal * gyv;
        const m = Math.hypot(vx, vy);
        if (m < 1e-3) { d.a += Math.sin(ticks * 0.6 + i * 1.7) * 0.5; vx = Math.cos(d.a); vy = Math.sin(d.a); }
        else { vx /= m; vy /= m; }
      }
      const want = Math.atan2(vy, vx);                 // turn-rate-limited heading
      d.a = wrap(d.a + Math.max(-MAXTURN, Math.min(MAXTURN, wrap(want - d.a))));
      let nx = d.x + Math.cos(d.a) * SPEED, ny = d.y + Math.sin(d.a) * SPEED;
      if (norm2(nx, ny) > 1) { nx = d.x; ny = d.y; d.a = wrap(d.a + Math.PI); }  // reflect at rim
      d.x = nx; d.y = ny;
      deposit(d);
    }

    function deposit(d) {
      const r = Math.ceil(SENSOR), s2 = (SENSOR * 0.7) ** 2;
      for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
        const gx = Math.round(d.x) + dx, gy = Math.round(d.y) + dy;
        if (gx < 0 || gy < 0 || gx >= GW || gy >= GH) continue;
        const g = Math.exp(-(dx * dx + dy * dy) / (2 * s2));
        phi[idx(gx, gy)] = Math.min(1, phi[idx(gx, gy)] + 0.55 * g);
        if (g > 0.4 && norm2(gx, gy) <= 1) seen[idx(gx, gy)] = 1;
      }
      victims.forEach(function (v) { if (!v.found && Math.hypot(v.x - d.x, v.y - d.y) <= SENSOR + 0.5) { v.found = true; v.t = ticks; } });
    }

    function physics() {
      ticks++;
      for (let i = 0; i < phi.length; i++) phi[i] *= BETA;
      drones.forEach(stepDrone);
    }
    function coveragePct() { let c = 0; for (let i = 0; i < seen.length; i++) c += seen[i]; return regionN ? c / regionN : 0; }
    const px = (x) => ox + x * cell, py = (y) => oy + y * cell;

    function draw() {
      ctx.clearRect(0, 0, cv.width, cv.height);
      ctx.fillStyle = "#fbfbfd"; ctx.fillRect(0, 0, cv.width, cv.height);
      ctx.save();
      ctx.beginPath(); ctx.ellipse(px(CX), py(CY), RX * cell, RY * cell, 0, 0, 7); ctx.clip();
      ctx.fillStyle = "#eef2f7"; ctx.fillRect(0, 0, cv.width, cv.height);
      for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) {
        const k = idx(x, y), p = phi[k];
        if (p > 0.02) { ctx.fillStyle = "rgba(232,116,59," + Math.min(0.82, 0.15 + p) + ")"; ctx.fillRect(px(x), py(y), cell + 0.6, cell + 0.6); }
        else if (seen[k]) { ctx.fillStyle = "rgba(42,157,143,0.16)"; ctx.fillRect(px(x), py(y), cell + 0.6, cell + 0.6); }
      }
      ctx.restore();
      ctx.strokeStyle = "rgba(23,48,79,0.5)"; ctx.lineWidth = 1.4; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.ellipse(px(CX), py(CY), RX * cell, RY * cell, 0, 0, 7); ctx.stroke();
      ctx.strokeStyle = "rgba(23,48,79,0.16)"; ctx.lineWidth = 1; ctx.setLineDash([2, 4]);
      ctx.beginPath(); ctx.ellipse(px(CX), py(CY), RX * cell * 0.5, RY * cell * 0.5, 0, 0, 7); ctx.stroke();
      ctx.setLineDash([]);
      victims.forEach(function (v) {
        const X = px(v.x), Y = py(v.y);
        if (v.found) {
          const pulse = 1 + 0.25 * Math.sin(ticks * 0.25);
          ctx.strokeStyle = "rgba(42,157,143,0.5)"; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(X, Y, cell * 1.7 * pulse, 0, 7); ctx.stroke();
          ctx.fillStyle = TEAL; ctx.beginPath(); ctx.arc(X, Y, cell * 0.95, 0, 7); ctx.fill();
          ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.7; ctx.beginPath();
          ctx.moveTo(X - cell * 0.45, Y); ctx.lineTo(X - cell * 0.1, Y + cell * 0.38); ctx.lineTo(X + cell * 0.5, Y - cell * 0.45); ctx.stroke();
        } else {
          ctx.fillStyle = "#fff"; ctx.strokeStyle = "#8b94a0"; ctx.lineWidth = 1.3;
          ctx.save(); ctx.translate(X, Y); ctx.rotate(Math.PI / 4);
          ctx.fillRect(-cell * 0.62, -cell * 0.62, cell * 1.24, cell * 1.24);
          ctx.strokeRect(-cell * 0.62, -cell * 0.62, cell * 1.24, cell * 1.24); ctx.restore();
        }
      });
      drones.forEach(function (d) {
        const X = px(d.x), Y = py(d.y);
        ctx.fillStyle = "rgba(42,157,143,0.10)"; ctx.beginPath(); ctx.arc(X, Y, SENSOR * cell, 0, 7); ctx.fill();
        ctx.fillStyle = NAVY; ctx.beginPath(); ctx.arc(X, Y, cell * 0.9, 0, 7); ctx.fill();
        ctx.strokeStyle = RESCUE; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(X, Y); ctx.lineTo(X + Math.cos(d.a) * cell * 2.1, Y + Math.sin(d.a) * cell * 2.1); ctx.stroke();
      });
    }

    function hud() {
      const pct = Math.round(100 * coveragePct());
      if (elCov) elCov.textContent = pct + "%";
      if (elBar) elBar.style.width = pct + "%";
      if (elVic) elVic.textContent = victims.filter(function (v) { return v.found; }).length + " / " + victims.length;
      if (elTime) elTime.textContent = (ticks / 10).toFixed(1) + " s";
    }

    function frame(ts) {
      if (!running) return;
      raf = requestAnimationFrame(frame);
      if (ts - last < 1000 / FPS) return;
      last = ts;
      for (let s = 0; s < speed; s++) physics();
      draw(); hud();
    }
    function play() { if (running) return; running = true; last = 0; raf = requestAnimationFrame(frame); }
    function pause() { running = false; if (raf) cancelAnimationFrame(raf); raf = null; }
    function cycleSpeed() { speed = speed === 1 ? 2 : (speed === 2 ? 4 : 1); if (elSpeed) elSpeed.textContent = speed + "×"; }

    function init() {
      cv = document.getElementById("stig-canvas");
      if (!cv || cv.dataset.ready) return;
      cv.dataset.ready = "1";
      ctx = cv.getContext("2d");
      cell = Math.floor(Math.min(cv.width / GW, cv.height / GH));
      ox = Math.round((cv.width - GW * cell) / 2);
      oy = Math.round((cv.height - GH * cell) / 2);
      elCov = document.getElementById("stig-cov");
      elBar = document.getElementById("stig-bar");
      elVic = document.getElementById("stig-vic");
      elTime = document.getElementById("stig-time");
      elSpeed = document.getElementById("stig-speed");
      reset();
      const b = function (id, fn) { const e = document.getElementById(id); if (e) e.addEventListener("click", fn); };
      b("stig-play", play); b("stig-pause", pause);
      b("stig-reset", function () { pause(); reset(); });
      b("stig-speed", cycleSpeed);
    }
    function onLeave() { pause(); }
    return { init: init, pause: pause, onLeave: onLeave };
  })();

  /* ==========================================================================
     WIDGET 3 — Interactive evaluation chart
     Data = the ILLUSTRATIVE comparative table of thesis §9 (Table "Per-pattern
     metrics") + the pairwise Welch tests (Table "Pairwise Welch tests on
     coverage"). Mean ± bootstrap 95% CI. Labelled illustrative, as the thesis is.
     ======================================================================== */
  const EvalChart = (function () {
    const PATTERNS = [
      { id: "spiral_out",       cov: [88, 2], f1: [0.82, 0.04], jain: [0.97, 0.01], en: [0.018, 0.002], welch: "vs random: t=11.4, p<0.001 ***", base: false },
      { id: "parallel_track",   cov: [84, 3], f1: [0.78, 0.05], jain: [0.98, 0.01], en: [0.021, 0.002], welch: "vs random: t=9.1, p<0.001 ***", base: false },
      { id: "expanding_square", cov: [80, 3], f1: [0.73, 0.05], jain: [0.96, 0.02], en: [0.024, 0.003], welch: "vs random: t=7.3, p<0.001 ***", base: false },
      { id: "random_walk",      cov: [62, 5], f1: [0.55, 0.07], jain: [0.91, 0.03], en: [0.041, 0.005], welch: "baseline (no structure)", base: true }
    ];
    const METRICS = {
      cov:  { label: "Final coverage (%)", max: 100, fmt: (v) => v + "%", better: "higher" },
      f1:   { label: "F₁ score",           max: 1.0, fmt: (v) => v.toFixed(2), better: "higher" },
      jain: { label: "Jain fairness",      max: 1.0, fmt: (v) => v.toFixed(2), better: "higher" },
      en:   { label: "Energy (J / %)",     max: 0.05, fmt: (v) => v.toFixed(3), better: "lower" }
    };
    let svg, tip, cur = "cov";

    function render() {
      while (svg.firstChild) svg.removeChild(svg.firstChild);
      const m = METRICS[cur];
      const W = 100, H = 62, x0 = 4, y0 = 4, bw = 16, gap = 6, plotH = 46;
      // axis baseline
      svg.appendChild(el("line", { x1: x0, y1: y0 + plotH, x2: W - 2, y2: y0 + plotH,
        stroke: "#ccc", "stroke-width": 0.4 }));
      const title = el("text", { x: W / 2, y: 60, "text-anchor": "middle", class: "ev-axis" });
      title.textContent = m.label + " — " + (m.better === "higher" ? "higher is better" : "lower is better");
      svg.appendChild(title);

      PATTERNS.forEach((p, i) => {
        const val = p[cur][0], ci = p[cur][1];
        const x = x0 + 6 + i * (bw + gap);
        const h = (val / m.max) * plotH;
        const y = y0 + plotH - h;
        const g = el("g", { class: "ev-bar" });
        const rect = el("rect", { x: x, y: y, width: bw, height: h, rx: 0.8,
          fill: p.base ? GREY : (p.id === "spiral_out" ? RESCUE : NAVY), opacity: 0 });
        g.appendChild(rect);
        // CI whisker
        const ch = (ci / m.max) * plotH;
        const cxw = x + bw / 2;
        const whisk = el("g", { stroke: "#222", "stroke-width": 0.4, opacity: 0, class: "ev-ci" });
        whisk.appendChild(el("line", { x1: cxw, y1: y - ch, x2: cxw, y2: y + ch }));
        whisk.appendChild(el("line", { x1: cxw - 2, y1: y - ch, x2: cxw + 2, y2: y - ch }));
        whisk.appendChild(el("line", { x1: cxw - 2, y1: y + ch, x2: cxw + 2, y2: y + ch }));
        // value label
        const vl = el("text", { x: cxw, y: y - ch - 1.4, "text-anchor": "middle", class: "ev-val" });
        vl.textContent = m.fmt(val);
        // name label
        const nm = el("text", { x: cxw, y: y0 + plotH + 4, "text-anchor": "middle", class: "ev-name" });
        nm.textContent = p.id.replace("_", " ");
        g.appendChild(whisk); g.appendChild(vl); g.appendChild(nm);

        // hover tooltip
        const hit = el("rect", { x: x - gap / 2, y: y0, width: bw + gap, height: plotH,
          fill: "transparent", style: "cursor:pointer" });
        hit.addEventListener("mousemove", (e) => showTip(e, p, m));
        hit.addEventListener("mouseleave", hideTip);
        g.appendChild(hit);
        svg.appendChild(g);
        setTimeout(() => { rect.style.opacity = 1; whisk.style.opacity = 1; }, 60 * i);
      });
    }

    function showTip(e, p, m) {
      tip.style.display = "block";
      tip.innerHTML = "<b>" + p.id + "</b><br>" + m.label + ": " +
        m.fmt(p[cur][0]) + " ± " + m.fmt(p[cur][1]) + " (95% CI)<br>" +
        "<span class='ev-welch'>" + (cur === "cov" ? p.welch : "coverage " + p.welch) + "</span>";
      const r = tip.parentElement.getBoundingClientRect();
      tip.style.left = (e.clientX - r.left + 12) + "px";
      tip.style.top = (e.clientY - r.top + 8) + "px";
    }
    function hideTip() { tip.style.display = "none"; }

    function init() {
      const host = document.getElementById("eval-chart");
      if (!host || host.dataset.ready) return;
      host.dataset.ready = "1";
      svg = el("svg", { viewBox: "0 0 100 62", class: "ev-svg" });
      host.appendChild(svg);
      tip = document.getElementById("eval-tip");
      document.querySelectorAll("#eval-controls [data-metric]").forEach((btn) => {
        btn.addEventListener("click", () => {
          cur = btn.dataset.metric;
          document.querySelectorAll("#eval-controls [data-metric]").forEach((b) =>
            b.classList.toggle("on", b === btn));
          render();
        });
      });
      render();
    }
    return { init: init };
  })();

  /* ----- public hook used by the deck init ----- */
  window.SARWidgets = {
    initAll: function () { Auction.init(); Stigmergy.init(); EvalChart.init(); },
    onSlideChanged: function (current) {
      // pause the canvas whenever we are NOT on its slide (CPU guard)
      if (!current || !current.querySelector || !current.querySelector("#stig-canvas")) {
        Stigmergy.onLeave();
      }
      // re-arm the auction when (re)entering its slide
      if (current && current.querySelector && current.querySelector("#auction-stage")) {
        Auction.reset();
      }
    }
  };
})();
