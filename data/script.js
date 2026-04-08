// ===== CANVAS & DOM ELEMENTS =====
const canvas = document.getElementById("radarCanvas");
const ctx = canvas.getContext("2d");
const tableBody = document.getElementById("target-list");
const alertBox = document.getElementById("alert-box");
const alertMsg = document.getElementById("alert-msg");
const connStatus = document.getElementById("conn-status");

// ===== CẤU HÌNH =====
const MAX_RADAR_RANGE = 6000; // mm
const TRAIL_LENGTH = 15; // Số điểm trong trail
const STATUS_COLORS = {
  0: "#4ade80", // NORMAL - Xanh lá
  1: "#facc15", // MONITORING - Vàng
  2: "#ef4444", // FALLEN - Đỏ
  3: "#ef4444", // FAINTED - Đỏ
  4: "#94a3b8", // IN_BED - Xám
};

const STATUS_TEXT = {
  0: "Đi lại",
  1: "Đứng yên",
  2: "⚠️ NGÃ",
  3: "⚠️ NGẤT",
  4: "Nghỉ ngơi",
};

// ===== BIẾN TOÀN CỤC =====
let peopleTrails = [[], [], []]; // Trail cho 3 người
let zones = []; // Mảng lưu các zone vẽ
let isDrawingMode = false;
let isDrawingZone = false;
let startX = 0,
  startY = 0;
let lastAlertId = -1;
let updateInterval = null;

// ===== KHỞI TẠO CANVAS =====
function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height - 60;
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// ===== HÀM VẼ CHÍNH =====
function drawRadar(peopleData) {
  // 1. Xóa canvas
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const centerX = canvas.width / 2;
  const bottomY = canvas.height;
  const scale = canvas.height / MAX_RADAR_RANGE;

  // 2. Vẽ grid (các vòng tròn cự ly)
  ctx.strokeStyle = "#334155";
  ctx.lineWidth = 1;
  for (let r = 1000; r <= MAX_RADAR_RANGE; r += 1000) {
    ctx.beginPath();
    ctx.arc(centerX, bottomY, r * scale, Math.PI, 2 * Math.PI);
    ctx.stroke();
  }

  // 3. Vẽ trục
  ctx.strokeStyle = "#475569";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(centerX - MAX_RADAR_RANGE * scale, bottomY);
  ctx.lineTo(centerX + MAX_RADAR_RANGE * scale, bottomY);
  ctx.moveTo(centerX, bottomY);
  ctx.lineTo(centerX, bottomY - MAX_RADAR_RANGE * scale);
  ctx.stroke();

  // 4. Vẽ các zone (hình chữ nhật vùng giường/ghế)
  drawZones(centerX, bottomY, scale);

  // 5. Vẽ mục tiêu với trail
  let tableHTML = "";
  let hasAlert = false;

  peopleData.forEach((person, idx) => {
    if (!person.active) return;

    const canvasX = centerX + person.x * scale;
    const canvasY = bottomY - person.y * scale;

    // Cập nhật trail
    peopleTrails[idx].push({ x: canvasX, y: canvasY });
    if (peopleTrails[idx].length > TRAIL_LENGTH) {
      peopleTrails[idx].shift();
    }

    const color = STATUS_COLORS[person.st] || "#94a3b8";
    const statusTxt = STATUS_TEXT[person.st] || "N/A";

    // Vẽ trail (đuôi quỹ đạo)
    drawTrail(peopleTrails[idx], color);

    // Vẽ chấm tròn mục tiêu
    ctx.beginPath();
    ctx.arc(canvasX, canvasY, 12, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Vẽ ID người (chữ trắng)
    ctx.fillStyle = "#fff";
    ctx.font = "bold 14px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(idx, canvasX, canvasY);

    // Cập nhật bảng
    tableHTML += `<tr>
      <td><strong>${idx}</strong></td>
      <td>${person.x}, ${person.y}</td>
      <td>${person.s}</td>
      <td style="color: ${color}">${statusTxt}</td>
    </tr>`;

    // Kiểm tra cảnh báo
    if ((person.st === 2 || person.st === 3) && lastAlertId !== idx) {
      hasAlert = true;
      lastAlertId = idx;
    }
  });

  tableBody.innerHTML =
    tableHTML ||
    `<tr><td colspan="4" style="text-align:center;color:#94a3b8;">Không có mục tiêu nào</td></tr>`;

  // Hiển thị alert nếu có
  if (hasAlert) {
    showAlert(lastAlertId, peopleData);
    alertBox.className = "alert alert-visible";
  }
}

// ===== VẼ TRAIL (ĐUÔI QUỸ ĐẠO) =====
function drawTrail(trail, color) {
  if (trail.length < 2) return;

  ctx.strokeStyle = color + "44"; // Màu mờ
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(trail[0].x, trail[0].y);

  for (let i = 1; i < trail.length; i++) {
    ctx.lineTo(trail[i].x, trail[i].y);
  }
  ctx.stroke();

  // Vẽ các chấm nhỏ trên đường trail
  ctx.fillStyle = color + "88";
  for (let i = 0; i < trail.length; i += 2) {
    ctx.beginPath();
    ctx.arc(trail[i].x, trail[i].y, 2, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// ===== VẼ ZONES (VÙNG GIƯỜNG/GHẾ) =====
function drawZones(centerX, bottomY, scale) {
  zones.forEach((zone) => {
    const x1 = centerX + zone.x1 * scale;
    const y1 = bottomY - zone.y1 * scale;
    const x2 = centerX + zone.x2 * scale;
    const y2 = bottomY - zone.y2 * scale;

    const width = Math.abs(x2 - x1);
    const height = Math.abs(y2 - y1);
    const startX = Math.min(x1, x2);
    const startY = Math.min(y1, y2);

    ctx.fillStyle = "rgba(56, 189, 248, 0.2)";
    ctx.fillRect(startX, startY, width, height);

    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2;
    ctx.strokeRect(startX, startY, width, height);

    ctx.fillStyle = "#38bdf8";
    ctx.font = "bold 12px Arial";
    ctx.fillText(zone.name, startX + 5, startY + 15);
  });
}

// ===== QUẢN LÝ ZONE DRAWING =====
function enableZoneDrawing() {
  isDrawingMode = true;
  canvas.style.cursor = "crosshair";

  canvas.addEventListener("mousedown", startDrawZone);
  canvas.addEventListener("mousemove", drawZonePreview);
  canvas.addEventListener("mouseup", endDrawZone);
  canvas.addEventListener("mouseleave", cancelDrawZone);
}

function disableZoneDrawing() {
  isDrawingMode = false;
  canvas.style.cursor = "default";

  canvas.removeEventListener("mousedown", startDrawZone);
  canvas.removeEventListener("mousemove", drawZonePreview);
  canvas.removeEventListener("mouseup", endDrawZone);
  canvas.removeEventListener("mouseleave", cancelDrawZone);
}

function startDrawZone(e) {
  isDrawingZone = true;
  const rect = canvas.getBoundingClientRect();
  startX = e.clientX - rect.left;
  startY = e.clientY - rect.top;
}

function drawZonePreview(e) {
  if (!isDrawingZone) return;

  const rect = canvas.getBoundingClientRect();
  const endX = e.clientX - rect.left;
  const endY = e.clientY - rect.top;

  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const centerX = canvas.width / 2;
  const bottomY = canvas.height;
  const scale = canvas.height / MAX_RADAR_RANGE;

  ctx.strokeStyle = "#334155";
  ctx.lineWidth = 1;
  for (let r = 1000; r <= MAX_RADAR_RANGE; r += 1000) {
    ctx.beginPath();
    ctx.arc(centerX, bottomY, r * scale, Math.PI, 2 * Math.PI);
    ctx.stroke();
  }

  drawZones(centerX, bottomY, scale);

  ctx.fillStyle = "rgba(250, 204, 21, 0.3)";
  ctx.fillRect(startX, startY, endX - startX, endY - startY);
  ctx.strokeStyle = "#facc15";
  ctx.lineWidth = 2;
  ctx.strokeRect(startX, startY, endX - startX, endY - startY);
}

function endDrawZone(e) {
  if (!isDrawingZone) return;
  isDrawingZone = false;

  const rect = canvas.getBoundingClientRect();
  const endX = e.clientX - rect.left;
  const endY = e.clientY - rect.top;

  if (Math.abs(endX - startX) < 20 || Math.abs(endY - startY) < 20) {
    return;
  }

  const centerX = canvas.width / 2;
  const bottomY = canvas.height;
  const scale = canvas.height / MAX_RADAR_RANGE;

  const x1 = (startX - centerX) / scale;
  const y1 = (bottomY - startY) / scale;
  const x2 = (endX - centerX) / scale;
  const y2 = (bottomY - endY) / scale;

  const zoneName = `Zone ${zones.length + 1}`;
  zones.push({
    name: zoneName,
    x1: Math.min(x1, x2),
    y1: Math.min(y1, y2),
    x2: Math.max(x1, x2),
    y2: Math.max(y1, y2),
  });

  console.log("Zone added:", zones[zones.length - 1]);
  drawRadar(currentPeopleData || []);
}

function cancelDrawZone() {
  isDrawingZone = false;
}

// ===== ALERT SYSTEM =====
function showAlert(personId, peopleData) {
  const person = peopleData[personId];
  let alertText = `Phát hiện người ${personId} `;
  if (person.st === 2) alertText += "⚠️ NGÃ!";
  else if (person.st === 3) alertText += "⚠️ NGẤT!";
  alertMsg.textContent = alertText;
}

function resetAlert() {
  alertBox.className = "alert alert-hidden";
  lastAlertId = -1;

  fetch("/reset-alert").catch(() => {
    console.log("Cannot reach ESP32 (offline mode)");
  });
}

// ===== MODE TOGGLE =====
document.getElementById("modeViewBtn").addEventListener("click", () => {
  disableZoneDrawing();
  document.getElementById("modeViewBtn").classList.add("active");
  document.getElementById("modeDrawBtn").classList.remove("active");
  document.getElementById("btnSaveZone").style.display = "none";
  document.getElementById("btnClearZone").style.display = "none";
});

document.getElementById("modeDrawBtn").addEventListener("click", () => {
  enableZoneDrawing();
  document.getElementById("modeViewBtn").classList.remove("active");
  document.getElementById("modeDrawBtn").classList.add("active");
  document.getElementById("btnSaveZone").style.display = "inline-block";
  document.getElementById("btnClearZone").style.display = "inline-block";
});

// ===== ZONE BUTTONS =====
document.getElementById("btnSaveZone").addEventListener("click", async () => {
  if (zones.length === 0) {
    alert("Vui lòng vẽ ít nhất một zone!");
    return;
  }

  try {
    const response = await fetch("/save-zones", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ zones }),
    });

    if (response.ok) {
      alert("✓ Đã lưu zones vào ESP32!");
    }
  } catch (err) {
    console.log("Save failed (offline):", err);
    alert("⚠️ Không thể kết nối ESP32");
  }
});

document.getElementById("btnClearZone").addEventListener("click", () => {
  zones = [];
  console.log("Zones cleared");
  drawRadar(currentPeopleData || []);
});

document
  .getElementById("btnResetConfig")
  .addEventListener("click", async () => {
    if (confirm("Bạn chắc chắn muốn reset toàn bộ cấu hình?")) {
      zones = [];
      peopleTrails = [[], [], []];

      try {
        await fetch("/reset-config");
      } catch {}

      console.log("Config reset");
      alert("✓ Đã reset cấu hình!");
    }
  });

// ===== LẤY ZONES TỪ ESP32 =====
async function loadZonesFromESP32() {
  try {
    const response = await fetch("/load-zones");
    if (response.ok) {
      const data = await response.json();
      zones = data.zones || [];
      console.log("Zones loaded:", zones);
    }
  } catch (err) {
    console.log("Load zones failed (offline):", err);
  }
}

// ===== LẤY DỮ LIỆU TỪ ESP32 =====
let currentPeopleData = [];

async function fetchData() {
  try {
    const response = await fetch("/data");
    if (!response.ok) throw new Error("API error");

    const data = await response.json();
    currentPeopleData = data.people;

    drawRadar(data.people);

    connStatus.innerText = "🟢 ĐANG TRỰC TUYẾN";
    connStatus.className = "status online";
  } catch (err) {
    connStatus.innerText = "🔴 MẤT KẾT NỐI";
    connStatus.className = "status offline";

    simulateData();
  }
}

// ===== SIMULATE DỮ LIỆU (Khi offline) =====
let simulationAngle = 0;

function simulateData() {
  simulationAngle += 0.05;

  const fakeData = [
    {
      id: 0,
      x: Math.sin(simulationAngle) * 1500 + 1000,
      y: Math.cos(simulationAngle) * 1000 + 2000,
      s: 150,
      st: 0,
      active: true,
    },
    {
      id: 1,
      x: -1500,
      y: 1000 + (simulationAngle < 5 ? simulationAngle * 200 : 1000),
      s: simulationAngle < 5 ? 100 : 0,
      st: simulationAngle < 5 ? 0 : 1,
      active: true,
    },
    {
      id: 2,
      x: 1000,
      y: 4000,
      s: 0,
      st: simulationAngle > 3 ? 2 : 1,
      active: true,
    },
  ];

  currentPeopleData = fakeData;
  drawRadar(fakeData);
}

// ===== KHỞI ĐỘNG =====
window.addEventListener("load", async () => {
  await loadZonesFromESP32();

  updateInterval = setInterval(fetchData, 200);

  fetchData();
});
