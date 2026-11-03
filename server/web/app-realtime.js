const realtime = (() => {
  const subscriptions = new Map();
  const maxPacket = 32 * 1024 * 1024;
  let socket = null, timer = null, watchdog = null, scheduled = false, failures = 0;

  async function inflate(bytes, expected) {
    const reader = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("deflate")).getReader();
    const output = new Uint8Array(expected);
    let offset = 0;
    try {
      while (true) {
        const {value, done} = await reader.read();
        if (done) break;
        if (offset + value.length > expected) throw new Error("Oversized map");
        output.set(value, offset); offset += value.length;
      }
      if (offset !== expected) throw new Error("Truncated map");
      return output;
    } finally {
      await reader.cancel();
    }
  }

  async function decode(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 8 || buffer.byteLength > maxPacket)
      throw new Error("Invalid realtime packet size");
    const view = new DataView(buffer);
    if (view.getUint32(0, false) !== 0x464e5631) throw new Error("Invalid realtime magic");
    const headerSize = view.getUint32(4, true), start = 8 + headerSize;
    if (start > buffer.byteLength) throw new Error("Truncated realtime header");
    const payload = JSON.parse(new TextDecoder("utf-8", {fatal: true}).decode(new Uint8Array(buffer, 8, headerSize)));
    if (payload.protocol !== 1 || !Number.isSafeInteger(payload.seq) || !Array.isArray(payload.attachments))
      throw new Error("Unsupported realtime protocol");
    for (const item of payload.attachments) {
      const {offset, length, decoded_length: size, encoding, compression} = item;
      if (![offset, length, size].every(Number.isSafeInteger) || offset < 0 || length < 0 || size < 0 ||
          size > maxPacket || start + offset + length > buffer.byteLength) throw new Error("Invalid attachment");
      let bytes = new Uint8Array(buffer, start + offset, length);
      if (compression === "deflate") bytes = await inflate(bytes, size);
      else if (compression !== "none" || bytes.length !== size) throw new Error("Invalid compression");
      const path = item.path.join(".");
      const data = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      if (path === "scene.map.data" && encoding === "int8") {
        const map = payload.scene.map;
        if (size !== map.width * map.height) throw new Error("Invalid map dimensions");
        map.data = new Int8Array(bytes.buffer, bytes.byteOffset, size);
      } else if (path === "scene.scan.ranges" && encoding === "uint16-le" && size % 2 === 0) {
        if (size / 2 !== payload.scene.scan.count) throw new Error("Invalid scan count");
        const ranges = new Uint16Array(size / 2);
        for (let i = 0; i < ranges.length; i++) ranges[i] = data.getUint16(i * 2, true);
        payload.scene.scan.ranges = ranges;
      } else if (path === "perception.points" && encoding === "float32-xy-le" && size % 8 === 0) {
        payload.perception.points = Array.from({length: size / 8}, (_, i) =>
          [data.getFloat32(i * 8, true), data.getFloat32(i * 8 + 4, true)]);
      } else throw new Error("Unknown attachment");
    }
    return payload;
  }

  function notify(state) {
    for (const sub of subscriptions.values()) sub.onstate?.(state);
  }

  function connect() {
    clearTimeout(timer); clearTimeout(watchdog);
    if (socket) { const previous = socket; socket = null; previous.close(); }
    if (!subscriptions.size) return;
    notify("pending");
    const source = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/realtime`, "finav.realtime.v1");
    socket = source; source.binaryType = "arraybuffer";
    let lastSequence = 0, session = null, processing = false;
    const armWatchdog = () => {
      clearTimeout(watchdog);
      watchdog = setTimeout(() => { if (socket === source) source.close(); }, 12000);
    };
    armWatchdog();
    source.onopen = () => {
      if (socket !== source) return;
      source.send(JSON.stringify({type: "subscribe", mode: subscriptions.get("scene")?.mode || "mapping",
        topics: Object.fromEntries([...subscriptions].map(([topic, sub]) => [topic, sub.hz]))}));
    };
    source.onmessage = async (event) => {
      if (socket !== source) return;
      try {
        if (processing) throw new Error("Unacknowledged realtime batch");
        processing = true;
        const payload = await decode(event.data);
        if (socket !== source) return;
        if (payload.seq !== lastSequence + 1 || (session && session !== payload.session_id))
          throw new Error("Realtime sequence mismatch");
        session = payload.session_id; lastSequence = payload.seq;
        // A restart resets scene caches; apply status before the accompanying full scene.
        if (payload.status && subscriptions.has("status")) subscriptions.get("status").ondata(payload.status, payload);
        await Promise.all([...subscriptions].filter(([topic]) => topic !== "status" && topic in payload)
          .map(([topic, sub]) => sub.ondata(payload[topic], payload)));
        if (socket !== source) return;
        source.send(JSON.stringify({type: "ack", seq: payload.seq}));
        processing = false; failures = 0;
        notify("online"); armWatchdog();
      } catch (error) {
        console.error("Realtime transport:", error);
        source.close(1002, "Invalid realtime batch");
      }
    };
    source.onclose = () => {
      if (socket !== source) return;
      socket = null; clearTimeout(watchdog); notify("offline");
      timer = setTimeout(connect, Math.min(10000, 500 * 2 ** Math.min(failures++, 5)) + Math.random() * 250);
    };
    source.onerror = () => { if (socket === source) source.close(); };
  }

  function reconnect() {
    if (scheduled) return;
    scheduled = true;
    queueMicrotask(() => { scheduled = false; connect(); });
  }

  function subscribe(topic, options) {
    const subscription = {...options};
    subscriptions.set(topic, subscription); reconnect();
    return {close() {
      if (subscriptions.get(topic) === subscription) { subscriptions.delete(topic); reconnect(); }
    }};
  }
  return {subscribe, decode};
})();
