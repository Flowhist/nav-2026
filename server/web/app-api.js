const connectionState = { http: "pending", ros: "pending", realtime: "pending", stream: null };

async function api(path, method = "GET", body = null, options = {}) {
  const { headers = {}, timeoutMs = 8000, ...rest } = options;
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
  const opt = { method, headers: { "Content-Type": "application/json", ...headers }, signal: controller.signal, ...rest };
  if (body !== null && body !== undefined) opt.body = JSON.stringify(body);
  try {
    const res = await fetch(path, opt);
    setConnectionState("online", "http");
    if (!res.ok) {
      const err = new Error(await readApiError(res));
      err.status = res.status;
      throw err;
    }
    return await res.json();
  } catch (err) {
    if (!err?.status) setConnectionState("offline", "http");
    if (err?.name === "AbortError") throw new Error("请求超时，请检查 Finav 服务连接");
    throw err;
  } finally {
    window.clearTimeout(timeout);
  }
}

function setConnectionState(state, channel = "http") {
  connectionState[channel] = state;
  const badge = $("connectionBadge");
  const label = $("connectionBadgeText");
  if (!badge || !label) return;
  const failed = Object.keys(connectionState).find((key) => connectionState[key] === "offline");
  const pending = Object.values(connectionState).includes("pending");
  badge.classList.remove("pending", "online", "offline");
  badge.classList.add(failed ? "offline" : pending ? "pending" : "online");
  label.textContent = failed ? { http: "服务连接中断", ros: "ROS 未连接", realtime: "数据连接中断", stream: "画面连接中断" }[failed]
    : pending ? "正在连接" : "在线";
}

async function readApiError(res) {
  try {
    const data = await res.json();
    if (data?.error) return typeof data.error === "object"
      ? String(data.error.message || data.error.code || res.statusText) : String(data.error);
  } catch (_err) {
    // A proxy or disconnected server may return a non-JSON response.
  }
  return `${res.status} ${res.statusText}`;
}
