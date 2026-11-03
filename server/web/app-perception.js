const perceptionView = (() => {
  const canvas = $("perceptionCanvas"), ctx = canvas.getContext("2d");
  let active = false, subscription = null, animation = null;
  let sample = null, received = 0, frameAt = 0, contours = [];
  const distances = new Map();
  const isLive = (time) => sample?.online && time - received < 1500 &&
    (!sample.service?.available || (sample.service.running && !sample.service.stopping));
  const color = (distance) => distance <= .5 ? "#c3544c" : distance <= 1 ? "#b58535" : "#6a8c80";

  function setActive(enabled) {
    active = enabled;
    subscription?.close();
    subscription = null;
    cancelAnimationFrame(animation);
    if (enabled && !document.hidden) {
      subscription = realtime.subscribe("perception", {hz: 10, ondata: receive,
        onstate: (state) => {
          if (state !== "online") {
            sample = null; contours = []; distances.clear(); renderReadings();
          }
        }});
      animation = requestAnimationFrame(animate);
    }
  }

  function groupPoints(points, resolution) {
    const size = Math.max(.1, resolution * 2), cells = new Set();
    for (const [x, y] of points) {
      cells.add(`${Math.floor(x / size)},${Math.floor(y / size)}`);
    }
    // Trace only exposed cell edges; keep gaps and holes instead of filling a convex hull.
    const edges = new Map();
    const add = (x, y, nx, ny) => {
      const key = `${x},${y}`;
      if (!edges.has(key)) edges.set(key, []);
      edges.get(key).push([nx, ny]);
    };
    for (const key of cells) {
      const [x, y] = key.split(",").map(Number);
      if (!cells.has(`${x},${y - 1}`)) add(x, y, x + 1, y);
      if (!cells.has(`${x + 1},${y}`)) add(x + 1, y, x + 1, y + 1);
      if (!cells.has(`${x},${y + 1}`)) add(x + 1, y + 1, x, y + 1);
      if (!cells.has(`${x - 1},${y}`)) add(x, y + 1, x, y);
    }
    const loops = [];
    while (edges.size) {
      const start = edges.keys().next().value, loop = [];
      let key = start;
      do {
        const [x, y] = key.split(",").map(Number);
        loop.push([x * size, y * size]);
        const outgoing = edges.get(key);
        if (!outgoing) break;
        const next = outgoing.pop();
        if (!outgoing.length) edges.delete(key);
        key = next.join(",");
      } while (key !== start);
      if (loop.length >= 3) loops.push(loop);
    }
    return loops;
  }

  function receive(data) {
    if (data.points_revision !== sample?.points_revision || data.depth_live !== sample?.depth_live || !sample)
      contours = groupPoints(data.points, data.resolution);
    sample = data; received = performance.now();
    renderReadings();
  }

  function renderReadings() {
    const fresh = isLive(performance.now()), service = sample?.service;
    $("perceptionState").textContent =
      service?.stopping ? "停止中" : fresh ? "实时观测" : service?.error ? "启动失败" :
      service?.running ? "启动中 · 等待数据" : service?.available ? "已关闭" : "观测离线";
    const error = service?.error || "";
    $("perceptionServiceError").textContent = error;
    $("perceptionServiceError").hidden = !error;
    $("perceptionState").dataset.online = String(Boolean(fresh));
    $("perceptionDepthState").textContent = fresh && sample.depth_live ? "双雷达在线" : "双雷达未就绪";
    const readings = ["前左", "前右", "左侧", "右侧"].map((name, i) => {
      const sensor = sample?.sensors[i], valid = fresh && sensor?.valid;
      const ink = valid && sensor.distance <= 1 ? color(sensor.distance) : "#373b3e";
      return `<div><span>${name}</span><strong style="color:${ink}">${valid ? `${sensor.distance.toFixed(2)}<small> m</small>` : "--"}</strong></div>`;
    }).join("");
    if ($("perceptionReadings").innerHTML !== readings) $("perceptionReadings").innerHTML = readings;
  }

  function animate(time) {
    if (!active || document.hidden) return;
    if (time - frameAt >= 50) { frameAt = time; draw(time); }
    animation = requestAnimationFrame(animate);
  }

  function draw(time) {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
      canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio);
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const range = Number($("perceptionRange").value), scale = Math.min(w - 48, h - 70) / (2 * range);
    const centerX = w / 2, centerY = (h - 20) / 2;
    const project = (x, y) => [centerX - y * scale, centerY - x * scale];
    const fresh = isLive(time);
    if (sample && !fresh) renderReadings();
    ctx.font = "11px sans-serif"; ctx.textAlign = "center";
    ctx.strokeStyle = "#9ba8a2"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(20, 23); ctx.lineTo(20 + .5 * scale, 23);
    ctx.moveTo(20, 19); ctx.lineTo(20, 26);
    ctx.moveTo(20 + .5 * scale, 19); ctx.lineTo(20 + .5 * scale, 26); ctx.stroke();
    ctx.fillStyle = "#7c8983"; ctx.fillText("0.5 m", 20 + .25 * scale, 40);

    if (fresh && sample.depth_live && $("perceptionDepth").checked) {
      ctx.save();
      ctx.beginPath(); ctx.rect(centerX - range * scale, centerY - range * scale, 2 * range * scale, 2 * range * scale); ctx.clip();
      ctx.beginPath();
      for (const loop of contours) {
        const points = loop.map(([x, y]) => project(x, y));
        const last = points[points.length - 1], first = points[0];
        ctx.moveTo((last[0] + first[0]) / 2, (last[1] + first[1]) / 2);
        points.forEach((p, i) => {
          const next = points[(i + 1) % points.length];
          ctx.quadraticCurveTo(p[0], p[1], (p[0] + next[0]) / 2, (p[1] + next[1]) / 2);
        });
        ctx.closePath();
      }
      ctx.fillStyle = "#e4ecef"; ctx.fill("evenodd");
      ctx.strokeStyle = "#94abb4"; ctx.lineWidth = 1.25; ctx.stroke();
      ctx.restore();
    }

    const sensors = sample?.sensors || [];
    const footprint = sample?.footprint;
    if (footprint) {
      const [xmin, xmax, ymin, ymax] = footprint;
      const [left, top] = project(xmax, ymax), width = (ymax - ymin) * scale, height = (xmax - xmin) * scale;
      const rounded = (x, y, rw, rh, radius) => { ctx.beginPath(); ctx.roundRect(x, y, rw, rh, radius); };
      ctx.fillStyle = "#f7f8f8"; ctx.strokeStyle = "#929e98"; ctx.lineWidth = 1.25;
      ctx.save(); ctx.shadowColor = "#20332a18"; ctx.shadowBlur = 10; ctx.shadowOffsetY = 3;
      rounded(left, top, width, height, Math.min(12, width * .12)); ctx.fill(); ctx.restore(); ctx.stroke();
      ctx.fillStyle = "#e9edeb";
      rounded(left + width * .15, top + height * .2, width * .7, height * .48, Math.min(8, width * .08)); ctx.fill();
      ctx.strokeStyle = "#769486"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(centerX - 5, top + 15); ctx.lineTo(centerX, top + 9); ctx.lineTo(centerX + 5, top + 15); ctx.stroke();
      ctx.fillStyle = "#406858"; ctx.font = `${width < 70 ? 10 : 12}px sans-serif`;
      ctx.fillText("车头", centerX, top + height * .46); ctx.fillText("车尾", centerX, top + height - 10);
      for (const sensor of sensors) {
        if (![sensor.x, sensor.y, sensor.yaw].every(Number.isFinite)) continue;
        const [x, y] = project(sensor.x, sensor.y);
        ctx.save(); ctx.translate(x, y); ctx.rotate(-sensor.yaw);
        ctx.fillStyle = fresh && sensor.valid ? color(sensor.distance) : "#9aa49f";
        rounded(-3, -2, 6, 4, 1); ctx.fill(); ctx.restore();
      }
      if (sample.camera) {
        const [cx, cy] = project(sample.camera[0], sample.camera[1]);
        ctx.save(); ctx.translate(cx, cy); ctx.rotate(-sample.camera[3]);
        ctx.fillStyle = "#588c99"; ctx.beginPath(); ctx.arc(0, 0, 2.5, 0, Math.PI * 2); ctx.fill(); ctx.restore();
      }
    }
    if ($("perceptionUltrasound").checked) for (const sensor of sensors) {
      if (!fresh || !sensor.valid) { distances.delete(sensor.address); continue; }
      const old = distances.get(sensor.address) ?? sensor.distance;
      // Ease only live measurements; disconnected echoes disappear immediately.
      const d = Math.abs(sensor.distance - old) < .005 ? sensor.distance : old + (sensor.distance - old) * .65;
      distances.set(sensor.address, d);
      const [sx, sy] = project(sensor.x, sensor.y), angle = -Math.PI / 2 - sensor.yaw;
      const half = Math.min(.26, .18 / Math.max(.2, d)), tint = color(sensor.distance);
      ctx.lineCap = "round";
      if (d <= range) {
        ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 8;
        ctx.beginPath(); ctx.arc(sx, sy, d * scale, angle - half, angle + half); ctx.stroke();
        ctx.strokeStyle = tint; ctx.lineWidth = 4; ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }

    $("perceptionEmpty").textContent = !fresh ? (sample?.service?.available && !sample.service.running ? "观测已关闭" : "等待实时观测数据") : !footprint ? "等待车体参数" :
      !sample.depth_live && !sensors.some(s => s.valid) ? "暂无有效感知数据" : "";
  }
  document.addEventListener("visibilitychange", () => setActive(appState.page === "perception"));
  for (const id of ["perceptionRange", "perceptionUltrasound", "perceptionDepth"])
    $(id).addEventListener("change", () => { if (active) draw(performance.now()); });
  return {setActive};
})();
