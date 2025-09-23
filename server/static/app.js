const statusElements = {
  connection: document.getElementById("connection-status"),
  driveTarget: document.getElementById("drive-target"),
  driveAccel: document.getElementById("drive-accel"),
  driveVelocity: document.getElementById("drive-actual-velocity"),
  drivePosition: document.getElementById("drive-actual-position"),
  steerTarget: document.getElementById("steer-target"),
  steerVelocity: document.getElementById("steer-velocity"),
  steerAccel: document.getElementById("steer-accel"),
  steerAngleList: document.getElementById("steer-actual-angle"),
  steerVelList: document.getElementById("steer-actual-velocity"),
  steerOffsetList: document.getElementById("steer-offsets"),
  toast: document.getElementById("toast"),
};

const TOKEN_STORAGE_KEY = "amrControlToken";
let authToken = (window.localStorage && localStorage.getItem(TOKEN_STORAGE_KEY)) || "";

const steps = {
  drive: {
    speed_step_mps: 0.05,
    accel_step_pps2: 10000,
  },
  steer: {
    angle_step_deg: 1,
    velocity_step_dps: 5,
    accel_step_dps2: 20,
    offset_step_deg: 0.05,
  },
};

const buttonConfigs = {
  "drive-speed": {
    label: "Speed",
    unit: " m/s",
    decimals: 2,
    getter: () => steps.drive.speed_step_mps,
  },
  "drive-accel": {
    label: "Accel",
    unit: " pps²",
    decimals: 0,
    getter: () => steps.drive.accel_step_pps2,
  },
  "steer-angle": {
    label: "Angle",
    unit: " °",
    decimals: 2,
    getter: () => steps.steer.angle_step_deg,
  },
  "steer-velocity": {
    label: "Velocity",
    unit: " °/s",
    decimals: 2,
    getter: () => steps.steer.velocity_step_dps,
  },
  "steer-accel": {
    label: "Accel",
    unit: " °/s²",
    decimals: 2,
    getter: () => steps.steer.accel_step_dps2,
  },
};

const adjustConfigs = {
  "drive-speed": {
    endpoint: "/drive/speed/adjust",
    field: "delta",
    decimals: 2,
    unit: " m/s",
    label: "Drive speed",
    getter: buttonConfigs["drive-speed"].getter,
  },
  "drive-accel": {
    endpoint: "/drive/accel",
    field: "delta",
    decimals: 0,
    unit: " pps²",
    label: "Drive accel",
    getter: buttonConfigs["drive-accel"].getter,
  },
  "steer-angle": {
    endpoint: "/steer/angle/adjust",
    field: "delta",
    decimals: 2,
    unit: " °",
    label: "Steer angle",
    getter: buttonConfigs["steer-angle"].getter,
  },
  "steer-velocity": {
    endpoint: "/steer/velocity",
    field: "delta",
    decimals: 2,
    unit: " °/s",
    label: "Steer velocity",
    getter: buttonConfigs["steer-velocity"].getter,
  },
  "steer-accel": {
    endpoint: "/steer/accel",
    field: "delta",
    decimals: 2,
    unit: " °/s²",
    label: "Steer accel",
    getter: buttonConfigs["steer-accel"].getter,
  },
};

const formatters = {
  speed: (value) => (value ?? 0).toFixed(2),
  accel: (value) => (value ?? 0).toFixed(2),
  angle: (value) => (value ?? 0).toFixed(2),
};

function buildHeaders(includeJson = false) {
  const headers = {};
  if (includeJson) {
    headers["Content-Type"] = "application/json";
  }
  if (authToken) {
    headers["Authorization"] = `Bearer ${authToken}`;
  }
  return headers;
}

function clearToken() {
  authToken = "";
  if (window.localStorage) {
    localStorage.removeItem(TOKEN_STORAGE_KEY);
  }
}

function requestToken(message = "Enter API token") {
  const input = window.prompt(message);
  if (!input || !input.trim()) {
    return null;
  }
  authToken = input.trim();
  if (window.localStorage) {
    localStorage.setItem(TOKEN_STORAGE_KEY, authToken);
  }
  return authToken;
}

async function postJson(path, body, retry = true) {
  const response = await fetch(path, {
    method: "POST",
    headers: buildHeaders(true),
    body: JSON.stringify(body ?? {}),
  });
  if (response.status === 401 && retry) {
    clearToken();
    if (requestToken("Authorization required")) {
      return postJson(path, body, false);
    }
  }
  if (!response.ok) {
    const payload = await safeJson(response);
    const message = payload?.detail || payload?.error || response.statusText;
    throw new Error(message);
  }
  return safeJson(response);
}

async function safeJson(response) {
  try {
    return await response.json();
  } catch (err) {
    return null;
  }
}

function showToast(message, isError = false) {
  if (!statusElements.toast) return;
  statusElements.toast.textContent = message;
  statusElements.toast.hidden = false;
  statusElements.toast.style.backgroundColor = isError ? "#b91c1c" : "#111827";
  setTimeout(() => {
    statusElements.toast.hidden = true;
  }, 3500);
}

function updateNodeList(element, values, formatter = (v) => v.toFixed(2), suffix = "") {
  if (!element) return;
  element.replaceChildren();
  if (!values) return;
  Object.entries(values)
    .sort(([a], [b]) => Number(a) - Number(b))
    .forEach(([node, value]) => {
      const li = document.createElement("li");
      if (value === null || value === undefined) {
        li.textContent = `${node}: N/A`;
      } else {
        li.textContent = `${node}: ${formatter(Number(value))}${suffix}`;
      }
      element.appendChild(li);
    });
}

function applyStepConfig(stepInfo) {
  if (!stepInfo) return;
  steps.drive.speed_step_mps = stepInfo.drive?.speed_step_mps ?? steps.drive.speed_step_mps;
  steps.drive.accel_step_pps2 = stepInfo.drive?.accel_step_pps2 ?? steps.drive.accel_step_pps2;
  steps.steer.angle_step_deg = stepInfo.steer?.angle_step_deg ?? steps.steer.angle_step_deg;
  steps.steer.velocity_step_dps = stepInfo.steer?.velocity_step_dps ?? steps.steer.velocity_step_dps;
  steps.steer.accel_step_dps2 = stepInfo.steer?.accel_step_dps2 ?? steps.steer.accel_step_dps2;
  steps.steer.offset_step_deg = stepInfo.steer?.offset_step_deg ?? steps.steer.offset_step_deg;
  updateButtonLabels();
  const offsetInput = document.getElementById("steer-offset-delta");
  if (offsetInput && !offsetInput.dataset.userModified) {
    offsetInput.value = steps.steer.offset_step_deg.toFixed(2);
  }
}

function updateButtonLabels() {
  Object.entries(buttonConfigs).forEach(([action, config]) => {
    document.querySelectorAll(`[data-action="${action}"]`).forEach((button) => {
      const sign = Number(button.dataset.sign || "1");
      const delta = config.getter() * sign;
      const magnitude = Math.abs(delta);
      button.textContent = `${config.label} ${sign > 0 ? "+" : "-"}${magnitude.toFixed(config.decimals)}${config.unit}`;
    });
  });
}

function updateStatus(payload) {
  if (!payload) return;
  const { drive = {}, steer = {}, steps: stepInfo } = payload;

  statusElements.driveTarget.textContent = formatters.speed(drive.target_speed_mps ?? 0);
  statusElements.driveAccel.textContent = formatters.accel(drive.target_accel_mps2 ?? 0);
  updateNodeList(statusElements.driveVelocity, drive.actual_velocity_mps, (v) => v.toFixed(2), " m/s");
  updateNodeList(statusElements.drivePosition, drive.actual_position_m, (v) => v.toFixed(2), " m");

  statusElements.steerTarget.textContent = formatters.angle(steer.target_deg ?? 0);
  statusElements.steerVelocity.textContent = formatters.angle(steer.profile_vel_dps ?? 0);
  statusElements.steerAccel.textContent = formatters.angle(steer.profile_accel_dps2 ?? 0);
  updateNodeList(statusElements.steerAngleList, steer.actual_angle_deg, (v) => v.toFixed(2), " °");
  updateNodeList(statusElements.steerVelList, steer.actual_velocity_dps, (v) => v.toFixed(2), " °/s");
  updateNodeList(statusElements.steerOffsetList, steer.offsets_deg, (v) => v.toFixed(3), " °");

  applyStepConfig(stepInfo);
}

async function refreshStatus(retry = true) {
  try {
    const response = await fetch("/status", { headers: buildHeaders() });
    if (response.status === 401 && retry) {
      clearToken();
      if (requestToken("Authorization required")) {
        return refreshStatus(false);
      }
    }
    if (response.ok) {
      const payload = await response.json();
      updateStatus(payload);
    }
  } catch (err) {
    console.error("Failed to refresh status", err);
  }
}

function setConnectionState(isConnected) {
  if (!statusElements.connection) return;
  statusElements.connection.textContent = isConnected ? "Live" : "Offline";
  statusElements.connection.classList.toggle("connected", isConnected);
  statusElements.connection.classList.toggle("disconnected", !isConnected);
}

function buildWsUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const tokenParam = authToken ? `?token=${encodeURIComponent(authToken)}` : "";
  return `${protocol}://${window.location.host}/ws/status${tokenParam}`;
}

function connectStatusStream() {
  const socket = new WebSocket(buildWsUrl());

  socket.addEventListener("open", () => {
    setConnectionState(true);
  });

  socket.addEventListener("message", (event) => {
    try {
      const payload = JSON.parse(event.data);
      updateStatus(payload);
    } catch (err) {
      console.error("Invalid status payload", err);
    }
  });

  socket.addEventListener("close", (event) => {
    setConnectionState(false);
    if (event.code === 4401) {
      clearToken();
      if (requestToken("Authorization required for live status")) {
        setTimeout(connectStatusStream, 100);
      }
      return;
    }
    setTimeout(connectStatusStream, 2000);
  });

  socket.addEventListener("error", () => {
    socket.close();
  });
}

function formatDelta(value, decimals) {
  return Math.abs(value).toFixed(decimals);
}

async function adjustWithStep(action, sign) {
  const config = adjustConfigs[action];
  if (!config) return;
  const stepValue = config.getter();
  const delta = stepValue * sign;
  const body = { [config.field]: delta };
  await postJson(config.endpoint, body);
  const signLabel = delta >= 0 ? "+" : "-";
  showToast(`${config.label} ${signLabel}${formatDelta(delta, config.decimals)}${config.unit}`);
}

function bindForms() {
  document.getElementById("drive-set-speed-form")?.addEventListener("submit", async (event) => {
    event.preventDefault();
    const value = Number(document.getElementById("drive-speed-input").value);
    try {
      await postJson("/drive/speed", { value });
      showToast(`Drive speed set to ${value.toFixed(2)} m/s`);
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.getElementById("drive-stop")?.addEventListener("click", async () => {
    try {
      await postJson("/drive/emergency_stop");
      showToast("Emergency stop sent", true);
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.getElementById("steer-set-angle-form")?.addEventListener("submit", async (event) => {
    event.preventDefault();
    const value = Number(document.getElementById("steer-angle-input").value);
    try {
      await postJson("/steer/angle", { value });
      showToast(`Steer angle set to ${value.toFixed(2)} °`);
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.getElementById("steer-zero")?.addEventListener("click", async () => {
    try {
      await postJson("/steer/angle", { value: 0 });
      showToast("Steer angle set to 0°");
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.querySelectorAll("[data-action]").forEach((button) => {
    const action = button.dataset.action;
    const sign = Number(button.dataset.sign || "1");
    button.addEventListener("click", async () => {
      try {
        await adjustWithStep(action, sign);
      } catch (err) {
        showToast(err.message, true);
      }
    });
  });

  document.getElementById("steer-offset-form")?.addEventListener("submit", async (event) => {
    event.preventDefault();
    const node_id = Number(document.getElementById("steer-offset-node").value);
    const deltaInput = document.getElementById("steer-offset-delta");
    const delta = Number(deltaInput.value);
    try {
      await postJson("/steer/offset", { node_id, delta });
      showToast(`Offset adjusted for node ${node_id}`);
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.getElementById("steer-offset-delta")?.addEventListener("input", (event) => {
    event.target.dataset.userModified = "true";
  });

  document.getElementById("steer-save-offsets")?.addEventListener("click", async () => {
    try {
      await postJson("/steer/save_offsets");
      showToast("Offsets saved");
    } catch (err) {
      showToast(err.message, true);
    }
  });

  document.getElementById("steer-load-offsets")?.addEventListener("click", async () => {
    try {
      await postJson("/steer/load_offsets");
      showToast("Offsets loaded");
    } catch (err) {
      showToast(err.message, true);
    }
  });
}

async function init() {
  bindForms();
  updateButtonLabels();
  await refreshStatus();
  connectStatusStream();
}

init();
