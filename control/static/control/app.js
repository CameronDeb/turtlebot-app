// Control page: buttons -> /api/commands/, Stop -> /api/stop/, and a 1 s poll of /api/state/ for everything shown.
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const csrf = document.querySelector("[name=csrfmiddlewaretoken]").value;
  let busy = false;

  async function post(url, body) {
    const resp = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-CSRFToken": csrf },
      body: JSON.stringify(body || {}),
    });
    let data;
    try {
      data = await resp.json();
    } catch (e) {
      data = { ok: false, error: `server error (${resp.status})` };
    }
    return data;
  }

  function showJobMessage(text, cls) {
    const el = $("job");
    el.textContent = text;
    el.className = "job " + cls;
  }

  function commandFor(name) {
    const distance = parseFloat($("distance").value);
    const angle = parseFloat($("angle").value);
    const checkClear = $("check-clear").checked;
    switch (name) {
      case "forward": return { kind: "forward", params: { distance, check_clear: checkClear } };
      case "left": return { kind: "turn", params: { degrees: angle } };
      case "right": return { kind: "turn", params: { degrees: -angle } };
      case "photo": return { kind: "photo", params: {} };
      case "scan": return { kind: "scan", params: { steps: 8 } };
      case "undock": return { kind: "undock", params: {} };
      case "dock": return { kind: "dock", params: {} };
      case "return_home": return { kind: "return_home", params: { check_clear: checkClear } };
    }
    return null;
  }

  async function send(cmd) {
    const data = await post(TB.urls.submit, cmd);
    if (!data.ok) showJobMessage(data.error, "failed");
    else showJobMessage(data.job.description + "…", "running");
    poll();
  }

  document.querySelectorAll("button.cmd").forEach((b) =>
    b.addEventListener("click", () => {
      const cmd = commandFor(b.dataset.cmd);
      if (cmd) send(cmd);
    })
  );

  $("find-form").addEventListener("submit", (e) => {
    e.preventDefault();
    const object = $("find-text").value.trim();
    if (!object) return;
    send({ kind: "find", params: { object, approach: $("find-approach").checked, standoff: parseFloat($("find-standoff").value) } });
  });

  $("motors-on").addEventListener("click", async () => {
    const data = await post(TB.urls.motors);
    showJobMessage(data.ok ? "Motors on." : data.error, data.ok ? "succeeded" : "failed");
    poll();
  });

  $("stop").addEventListener("click", async () => {
    const data = await post(TB.urls.stop);
    if (!data.ok) showJobMessage(data.error, "failed");
    poll();
  });

  // Typed commands: "Check" shows how the text was understood; "Run" runs it.
  async function interpret(run) {
    const text = $("text").value.trim();
    if (!text) return;
    const data = await post(TB.urls.interpret, { text, run });
    const out = $("text-result");
    if (!data.ok) {
      out.textContent = data.error;
      out.className = "hint error";
      return;
    }
    out.textContent = (run ? "Running: " : "Understood: ") + data.preview.join(" → ");
    out.className = "hint ok";
    if (run) {
      $("text").value = "";
      poll();
    }
  }
  $("preview-btn").addEventListener("click", () => interpret(false));
  $("text-form").addEventListener("submit", (e) => {
    e.preventDefault();
    interpret(true);
  });

  function fmtTime(iso) {
    return iso ? new Date(iso).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit", timeZone: TB.timeZone }) : "";
  }

  function render(state) {
    const r = state.robot;
    const conn = $("conn");
    if (r.connected) {
      conn.textContent = r.backend === "sim" ? "simulator" : "connected";
      conn.className = "pill ok";
      $("battery").textContent = r.battery_percent == null ? "–" : `${Math.round(r.battery_percent)}%`;
      $("dock").textContent = r.docked == null ? "–" : r.docked ? "docked" : r.dock_visible ? "off dock (dock in view)" : "off dock";
      if (r.pose) {
        $("pose").textContent = `${r.pose.x.toFixed(2)}, ${r.pose.y.toFixed(2)} m`;
        $("heading").textContent = `${r.pose.heading_deg}°`;
      } else {
        $("pose").textContent = r.pose_error ? "unknown" : "–";
        $("heading").textContent = "–";
      }
    } else {
      conn.textContent = "offline";
      conn.className = "pill bad";
      conn.title = r.error || "";
    }

    $("estop").hidden = !(r.connected && r.wheels_enabled === false);
    busy = !!state.job;
    document.querySelectorAll("button.cmd, #find-btn").forEach((b) => (b.disabled = busy || !r.connected));
    if (state.job) {
      showJobMessage(state.job.description + "…", "running");
    } else if (!r.connected) {
      showJobMessage(r.error || "Robot offline", "failed");
    } else {
      const last = state.recent_jobs[0];
      if (last) showJobMessage(`${last.description}: ${last.message || last.state}`, last.state);
      else showJobMessage("Idle", "idle");
    }

    const trip = state.trip;
    $("trip-info").textContent = trip
      ? `Trip ${trip.id}: ${trip.moves} recorded move${trip.moves === 1 ? "" : "s"}; Return ${trip.started_docked ? "retraces them and docks" : "retraces them to the start"}.`
      : "No trip yet: the first move starts recording the path home.";

    const log = $("log");
    log.replaceChildren(
      ...state.recent_jobs.map((j) => {
        const tr = document.createElement("tr");
        tr.className = j.state;
        for (const text of [fmtTime(j.created_at), (j.source === "text" ? `“${j.text}” ` : "") + j.description, j.message || j.state]) {
          const td = document.createElement("td");
          td.textContent = text;
          tr.appendChild(td);
        }
        return tr;
      })
    );

    const p = state.latest_photo;
    if (p) {
      const img = $("latest-img");
      if (img.dataset.id !== String(p.id)) {
        img.src = p.url;
        img.dataset.id = p.id;
      }
      img.hidden = false;
      $("latest-empty").hidden = true;
      drawBoxes($("latest-boxes"), p);
      $("latest-link").href = p.detail_url;
      $("latest-meta").textContent = `#${p.id} · ${new Date(p.taken_at).toLocaleString([], { timeZone: TB.timeZone })}` +
        (p.pose ? ` · at ${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)} m` : "");
    }

    drawMap(trip, r.connected ? r.pose : null);
  }

  // Detection boxes over a picture, positioned in % so they follow the image's displayed size.
  function drawBoxes(layer, p) {
    layer.replaceChildren(
      ...(p.detections || []).map((d) => {
        const [x0, y0, x1, y1] = d.box;
        const b = document.createElement("span");
        b.className = "detbox";
        b.style.left = `${(100 * x0) / p.width}%`;
        b.style.top = `${(100 * y0) / p.height}%`;
        b.style.width = `${(100 * (x1 - x0)) / p.width}%`;
        b.style.height = `${(100 * (y1 - y0)) / p.height}%`;
        const t = document.createElement("span");
        t.textContent = `${d.label} ${d.score.toFixed(2)}` + (d.distance ? ` · ${d.distance.toFixed(2)} m` : "");
        b.appendChild(t);
        return b;
      })
    );
  }

  // Top-down map of the odom frame, drawn with +x pointing up the page and +y to the left.
  function drawMap(trip, pose) {
    const svg = $("map");
    const NS = "http://www.w3.org/2000/svg";
    const pts = trip ? trip.path.slice() : [];
    const all = pts.concat(pose ? [[pose.x, pose.y]] : []);
    if (trip && trip.dock) all.push(trip.dock);
    svg.replaceChildren();
    if (!all.length) return;
    // screen: right = -y, up = +x (so the robot's starting forward direction points up)
    const sx = (p) => -p[1];
    const sy = (p) => -p[0];
    const xs = all.map(sx), ys = all.map(sy);
    const pad = 0.5;
    const minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
    const minY = Math.min(...ys) - pad, maxY = Math.max(...ys) + pad;
    const size = Math.max(maxX - minX, maxY - minY, 2);
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    svg.setAttribute("viewBox", `${cx - size / 2} ${cy - size / 2} ${size} ${size}`);
    const sw = size / 150;
    const el = (tag, attrs) => {
      const e = document.createElementNS(NS, tag);
      for (const k in attrs) e.setAttribute(k, attrs[k]);
      svg.appendChild(e);
      return e;
    };
    // 1 m grid
    for (let g = Math.floor(cx - size / 2); g <= cx + size / 2; g++) el("line", { x1: g, y1: cy - size / 2, x2: g, y2: cy + size / 2, class: "grid" , "stroke-width": sw / 2 });
    for (let g = Math.floor(cy - size / 2); g <= cy + size / 2; g++) el("line", { x1: cx - size / 2, y1: g, x2: cx + size / 2, y2: g, class: "grid", "stroke-width": sw / 2 });
    const home = trip && trip.dock ? trip.dock : pts[0];
    if (home) el("rect", { x: sx(home) - 0.12, y: sy(home) - 0.12, width: 0.24, height: 0.24, class: "home", "stroke-width": sw });
    if (pts.length > 1) el("polyline", { points: pts.map((p) => `${sx(p)},${sy(p)}`).join(" "), class: "path", "stroke-width": sw * 2 });
    if (pose) {
      const p = [pose.x, pose.y];
      el("circle", { cx: sx(p), cy: sy(p), r: 0.17, class: "robot", "stroke-width": sw });
      const hx = [pose.x + 0.3 * Math.cos(pose.yaw), pose.y + 0.3 * Math.sin(pose.yaw)];
      el("line", { x1: sx(p), y1: sy(p), x2: sx(hx), y2: sy(hx), class: "heading", "stroke-width": sw * 2 });
    }
  }

  let polling = false;
  async function poll() {
    if (polling) return;
    polling = true;
    try {
      const resp = await fetch(TB.urls.state, { cache: "no-store" });
      render(await resp.json());
    } catch (e) {
      $("conn").textContent = "web server offline";
      $("conn").className = "pill bad";
    } finally {
      polling = false;
    }
  }
  poll();
  setInterval(poll, 1000);
})();
