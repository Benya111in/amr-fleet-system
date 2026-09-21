/* amr_dashboard 단일 페이지 앱 (vanilla JS, 외부 의존 없음)
 *
 * 데이터 흐름: EventSource('/events') → snapshot(전체) → status / alerts / task_event / map_updated /
 * estop / task_request / heartbeat 이벤트로 state 갱신 → 해당 영역만 다시 그린다.
 * 조작: POST /api/tasks (작업 투입), POST /api/estop (긴급 정지). 지도 이미지는 GET /api/map?format=png.
 */
'use strict';

(function () {
  var STATUS_COLORS = {
    0: '#9aa0a6', 1: '#34c759', 2: '#3b82f6', 3: '#f59e0b', 4: '#14b8a6', 5: '#ef4444', 6: '#d946ef'
  };
  var STATUS_LABELS = {
    IDLE: '대기', MOVING: '이동', DOCKING: '도킹', LOADING: '적재',
    CHARGING: '충전', ERROR: '오류', ESTOP: '긴급정지', UNKNOWN: '알 수 없음'
  };
  var TASK_LABELS = { PENDING: '대기', IN_PROGRESS: '진행', COMPLETED: '완료', FAILED: '실패', UNKNOWN: '?' };
  var LEVEL_LABELS = { OK: '정상', WARN: '경고', ERROR: '오류', STALE: '끊김', UNKNOWN: '?' };
  var BANNER_TTL_SEC = 60;
  var STALE_AFTER_SEC = 12;

  var state = {
    world: { origin_x: 0, origin_y: 0, width: 60, height: 40 },
    robotIds: [],
    status: null,
    alerts: [],
    taskEvents: [],
    tasks: {},
    map: null,
    estops: { all: false },
    lastEventAt: 0,
    connected: false,
    dismissedAlertKey: null
  };
  var mapImage = { img: null, stamp: null };

  function $(id) { return document.getElementById(id); }

  function esc(text) {
    return String(text === undefined || text === null ? '' : text)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function fmtTime(sec) {
    if (!sec) { return '–'; }
    var d = new Date(sec * 1000);
    return d.toLocaleTimeString('ko-KR', { hour12: false });
  }

  function fmtNum(v, digits) {
    if (v === undefined || v === null || isNaN(v)) { return '–'; }
    return Number(v).toFixed(digits === undefined ? 1 : digits);
  }

  // ----- 연결 표시 -----

  function setConn(mode, text) {
    var dot = $('conn-dot');
    dot.className = 'dot ' + mode;
    $('conn-text').textContent = text;
  }

  function touch() {
    state.lastEventAt = Date.now();
    if (!state.connected) {
      state.connected = true;
      setConn('on', '실시간 연결됨');
    }
  }

  setInterval(function () {
    if (!state.lastEventAt) { return; }
    var age = (Date.now() - state.lastEventAt) / 1000;
    if (age > STALE_AFTER_SEC && state.connected) {
      state.connected = false;
      setConn('stale', '수신 지연 ' + Math.round(age) + ' s');
    }
  }, 1000);

  // ----- SSE -----

  function connect() {
    var es = new EventSource('/events');
    es.addEventListener('snapshot', function (e) { touch(); applySnapshot(JSON.parse(e.data)); });
    es.addEventListener('status', function (e) { touch(); applyStatus(JSON.parse(e.data)); });
    es.addEventListener('alerts', function (e) { touch(); applyAlerts(JSON.parse(e.data)); });
    es.addEventListener('task_event', function (e) { touch(); applyTaskEvent(JSON.parse(e.data)); });
    es.addEventListener('map_updated', function (e) { touch(); applyMapMeta(JSON.parse(e.data)); });
    es.addEventListener('estop', function (e) { touch(); applyEstop(JSON.parse(e.data)); });
    es.addEventListener('task_request', function (e) { touch(); });
    es.addEventListener('heartbeat', function (e) {
      touch();
      $('server-time').textContent = fmtTime(JSON.parse(e.data).ts);
    });
    es.onerror = function () {
      state.connected = false;
      setConn('off', '연결 끊김 — 재연결 중…');
    };
  }

  // ----- 상태 적용 -----

  function applySnapshot(s) {
    if (s.world) { state.world = s.world; }
    state.robotIds = s.robot_ids || [];
    state.alerts = s.alerts || [];
    state.taskEvents = s.task_events || [];
    state.tasks = s.tasks || {};
    state.estops = s.estops || { all: false };
    $('server-time').textContent = fmtTime(s.server_time);
    if (s.map) { applyMapMeta(s.map); } else { renderMapMeta(); }
    if (s.status) { applyStatus(s.status); } else { renderRobots(); renderKpi(); }
    renderAlerts();
    renderTimeline();
    renderEstops();
    renderRobotOptions();
    resizeCanvas();
    draw();
  }

  function applyStatus(status) {
    state.status = status;
    var changed = false;
    (status.robots || []).forEach(function (r) {
      if (r.robot_id && state.robotIds.indexOf(r.robot_id) < 0) {
        state.robotIds.push(r.robot_id);
        changed = true;
      }
    });
    if (changed) { renderEstops(); renderRobotOptions(); }
    renderRobots();
    renderKpi();
    draw();
  }

  function applyAlerts(alerts) {
    state.alerts = state.alerts.concat(alerts).slice(-100);
    renderAlerts();
  }

  function applyTaskEvent(ev) {
    state.taskEvents.push(ev);
    if (state.taskEvents.length > 50) { state.taskEvents.shift(); }
    state.tasks[ev.task_id] = ev;
    renderTimeline();
    draw();
  }

  function applyMapMeta(meta) {
    state.map = meta;
    if (meta.width > 0 && meta.height > 0 && meta.resolution > 0) {
      state.world = {
        origin_x: meta.origin.x, origin_y: meta.origin.y,
        width: meta.width * meta.resolution, height: meta.height * meta.resolution
      };
    }
    renderMapMeta();
    resizeCanvas();
    loadMapImage(meta);
  }

  function applyEstop(data) {
    state.estops = data.estops || state.estops;
    renderEstops();
    $('estop-result').textContent = fmtTime(data.ts) + ' ' + data.target + ' → '
      + (data.active ? '긴급 정지 활성' : '해제');
  }

  // ----- 지도 -----

  function renderMapMeta() {
    var m = state.map;
    if (!m) {
      $('map-meta').textContent = '지도 없음 (기본 ' + state.world.width + ' × ' + state.world.height + ' m)';
      return;
    }
    $('map-meta').textContent = m.width + '×' + m.height + ' @ ' + m.resolution + ' m ('
      + fmtNum(m.width * m.resolution, 1) + ' × ' + fmtNum(m.height * m.resolution, 1) + ' m)';
  }

  function loadMapImage(meta) {
    if (mapImage.stamp === meta.stamp && mapImage.img) { draw(); return; }
    var img = new Image();
    img.onload = function () { mapImage.img = img; mapImage.stamp = meta.stamp; draw(); };
    img.onerror = function () { mapImage.img = null; draw(); };
    img.src = '/api/map?format=png&t=' + encodeURIComponent(meta.stamp);
  }

  function resizeCanvas() {
    var canvas = $('map-canvas');
    var cssW = Math.max(240, canvas.parentElement.clientWidth - 26);
    var cssH = cssW * (state.world.height / state.world.width);
    var dpr = window.devicePixelRatio || 1;
    canvas.style.width = cssW + 'px';
    canvas.style.height = cssH + 'px';
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
  }

  function draw() {
    var canvas = $('map-canvas');
    var ctx = canvas.getContext('2d');
    var w = state.world;
    var s = Math.min(canvas.width / w.width, canvas.height / w.height);
    var dpr = window.devicePixelRatio || 1;
    function tx(x) { return (x - w.origin_x) * s; }
    function ty(y) { return canvas.height - (y - w.origin_y) * s; }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#0b1220';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (mapImage.img && state.map) {
      var m = state.map;
      var mw = m.width * m.resolution * s;
      var mh = m.height * m.resolution * s;
      ctx.globalAlpha = 0.85;
      ctx.drawImage(mapImage.img, tx(m.origin.x), ty(m.origin.y + m.height * m.resolution), mw, mh);
      ctx.globalAlpha = 1.0;
    }

    // 5 m 격자
    ctx.strokeStyle = 'rgba(148,163,184,0.18)';
    ctx.lineWidth = 1 * dpr;
    ctx.font = (10 * dpr) + 'px sans-serif';
    ctx.fillStyle = 'rgba(148,163,184,0.7)';
    var gx, gy;
    for (gx = Math.ceil(w.origin_x / 5) * 5; gx <= w.origin_x + w.width; gx += 5) {
      ctx.beginPath(); ctx.moveTo(tx(gx), 0); ctx.lineTo(tx(gx), canvas.height); ctx.stroke();
      ctx.fillText(gx + '', tx(gx) + 2 * dpr, canvas.height - 3 * dpr);
    }
    for (gy = Math.ceil(w.origin_y / 5) * 5; gy <= w.origin_y + w.height; gy += 5) {
      ctx.beginPath(); ctx.moveTo(0, ty(gy)); ctx.lineTo(canvas.width, ty(gy)); ctx.stroke();
      ctx.fillText(gy + '', 2 * dpr, ty(gy) - 2 * dpr);
    }

    drawTaskTargets(ctx, tx, ty, dpr);
    drawRobots(ctx, tx, ty, dpr);
  }

  function drawMarker(ctx, x, y, dpr, color, label, filled) {
    var r = 6 * dpr;
    ctx.beginPath();
    ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y); ctx.closePath();
    ctx.lineWidth = 1.5 * dpr;
    ctx.strokeStyle = color;
    if (filled) { ctx.fillStyle = color; ctx.fill(); }
    ctx.stroke();
    ctx.fillStyle = '#e2e8f0';
    ctx.font = 'bold ' + (10 * dpr) + 'px sans-serif';
    ctx.fillText(label, x + r + 2 * dpr, y + 4 * dpr);
  }

  function drawTaskTargets(ctx, tx, ty, dpr) {
    var robots = (state.status && state.status.robots) || [];
    var byRobot = {};
    robots.forEach(function (r) { if (r.current_task_id) { byRobot[r.current_task_id] = r; } });
    Object.keys(state.tasks).forEach(function (id) {
      var t = state.tasks[id];
      var active = t.status_name === 'IN_PROGRESS' || byRobot[id];
      if (!active && t.status_name !== 'PENDING') { return; }
      var color = active ? '#38bdf8' : 'rgba(148,163,184,0.7)';
      var px = tx(t.pickup_pose.x), py = ty(t.pickup_pose.y);
      var dx = tx(t.dropoff_pose.x), dy = ty(t.dropoff_pose.y);
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1 * dpr;
      ctx.beginPath();
      var r = byRobot[id];
      if (r) { ctx.moveTo(tx(r.pose.x), ty(r.pose.y)); ctx.lineTo(px, py); } else { ctx.moveTo(px, py); }
      ctx.lineTo(dx, dy);
      ctx.stroke();
      ctx.setLineDash([]);
      drawMarker(ctx, px, py, dpr, color, 'P ' + id, !!active);
      drawMarker(ctx, dx, dy, dpr, color, 'D', false);
    });
  }

  function drawRobots(ctx, tx, ty, dpr) {
    var robots = (state.status && state.status.robots) || [];
    var s = Math.min(ctx.canvas.width / state.world.width, ctx.canvas.height / state.world.height);
    var len = Math.max(12 * dpr, 1.2 * s);
    robots.forEach(function (r) {
      var x = tx(r.pose.x), y = ty(r.pose.y), yaw = r.pose.yaw;
      var color = STATUS_COLORS[r.status] || '#9aa0a6';
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-yaw);   // 캔버스 y 는 아래로 증가하므로 부호 반전
      ctx.beginPath();
      ctx.moveTo(len * 0.6, 0);
      ctx.lineTo(-len * 0.4, len * 0.35);
      ctx.lineTo(-len * 0.4, -len * 0.35);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
      ctx.lineWidth = 1.5 * dpr;
      ctx.strokeStyle = '#0b1220';
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = '#e2e8f0';
      ctx.font = 'bold ' + (11 * dpr) + 'px sans-serif';
      ctx.fillText(r.robot_id + ' ' + Math.round(r.battery_level) + '%', x + len * 0.5, y - len * 0.5);
    });
  }

  window.addEventListener('resize', function () { resizeCanvas(); draw(); });

  // ----- 로봇 표 / KPI -----

  function statusTag(r) {
    var color = STATUS_COLORS[r.status] || '#9aa0a6';
    return '<span class="status-tag" style="background:' + color + '">'
      + esc(STATUS_LABELS[r.status_name] || r.status_name) + '</span>';
  }

  function batteryCell(level) {
    var pct = Math.max(0, Math.min(100, Number(level) || 0));
    var cls = pct < 15 ? ' critical' : (pct < 30 ? ' low' : '');
    return '<span class="battery"><span class="bar"><span class="fill' + cls + '" style="width:' + pct
      + '%"></span></span>' + fmtNum(pct, 0) + '%</span>';
  }

  function renderRobots() {
    var body = $('robot-body');
    var robots = (state.status && state.status.robots) || [];
    if (!robots.length) {
      body.innerHTML = '<tr><td colspan="5" class="muted">플릿 상태 수신 대기 중…</td></tr>';
      return;
    }
    body.innerHTML = robots.map(function (r) {
      var deg = (r.pose.yaw * 180 / Math.PI);
      return '<tr><td><b>' + esc(r.robot_id) + '</b></td><td>' + statusTag(r) + '</td><td>'
        + batteryCell(r.battery_level) + '</td><td>' + (r.current_task_id ? esc(r.current_task_id) : '<span class="muted">–</span>')
        + '</td><td>' + fmtNum(r.pose.x, 2) + ', ' + fmtNum(r.pose.y, 2) + ', ' + fmtNum(deg, 0) + '°</td></tr>';
    }).join('');
  }

  function renderKpi() {
    var k = state.status ? state.status.kpi : null;
    $('kpi-pending').textContent = k ? k.tasks_pending : '–';
    $('kpi-in-progress').textContent = k ? k.tasks_in_progress : '–';
    $('kpi-completed').textContent = k ? k.tasks_completed : '–';
    $('kpi-failed').textContent = k ? k.tasks_failed : '–';
    $('kpi-throughput').textContent = k ? fmtNum(k.throughput, 1) : '–';
    $('kpi-avg-duration').textContent = k ? fmtNum(k.avg_task_duration, 1) : '–';
    $('kpi-utilization').textContent = k ? fmtNum(k.robot_utilization * 100, 1) : '–';
    $('kpi-deadlock').textContent = k ? k.deadlock_count : '–';
    $('kpi-stamp').textContent = state.status
      ? '플릿 상태 ' + fmtTime(state.status.received_at) + ' 수신 (로봇 ' + state.status.robots.length + '대)'
      : '플릿 상태 수신 대기 중…';
  }

  // ----- 알림 -----

  function alertKey(a) { return a.received_at + '|' + a.name + '|' + a.message; }

  function renderAlerts() {
    var list = $('alert-list');
    var alerts = state.alerts.slice().reverse();
    $('alert-count').textContent = alerts.length ? '(' + alerts.length + ')' : '';
    if (!alerts.length) {
      list.innerHTML = '<li class="muted">알림 없음</li>';
    } else {
      list.innerHTML = alerts.map(function (a) {
        var lv = (a.level_name || 'UNKNOWN').toLowerCase();
        return '<li><span class="time">' + fmtTime(a.received_at) + '</span><span class="level-tag ' + esc(lv) + '">'
          + esc(LEVEL_LABELS[a.level_name] || a.level_name) + '</span><b>' + esc(a.hardware_id || '-') + '</b> '
          + esc(a.name) + ' <span class="muted">' + esc(a.message) + '</span></li>';
      }).join('');
    }
    renderBanner();
  }

  function renderBanner() {
    var banner = $('alert-banner');
    var now = Date.now() / 1000;
    var latest = null;
    state.alerts.forEach(function (a) {
      if (a.level >= 1 && (now - a.received_at) < BANNER_TTL_SEC) { latest = a; }
    });
    if (!latest || alertKey(latest) === state.dismissedAlertKey) {
      banner.className = 'banner hidden';
      return;
    }
    banner.className = 'banner level-' + (latest.level >= 2 ? 'error' : 'warn');
    banner.textContent = (latest.level >= 2 ? '오류' : '경고') + ' · ' + (latest.hardware_id || '플릿') + ' · '
      + latest.name + ' — ' + latest.message + ' (' + fmtTime(latest.received_at) + ')';
    banner.onclick = function () { state.dismissedAlertKey = alertKey(latest); renderBanner(); };
  }

  setInterval(renderBanner, 5000);

  // ----- 작업 타임라인 -----

  function renderTimeline() {
    var list = $('timeline-list');
    var events = state.taskEvents.slice().reverse();
    if (!events.length) {
      list.innerHTML = '<li class="muted">작업 이벤트 없음</li>';
      return;
    }
    list.innerHTML = events.map(function (t) {
      var st = (t.status_name || 'UNKNOWN').toLowerCase();
      return '<li><span class="time">' + fmtTime(t.received_at) + '</span><span class="task-tag ' + esc(st) + '">'
        + esc(TASK_LABELS[t.status_name] || t.status_name) + '</span><b>' + esc(t.task_id) + '</b><span class="muted">'
        + (t.robot_id ? esc(t.robot_id) : '미할당') + ' · ' + esc(t.item_type) + ' · P' + esc(t.priority) + '</span></li>';
    }).join('');
  }

  // ----- 작업 투입 폼 -----

  function renderRobotOptions() {
    var sel = $('task-robot');
    var current = sel.value;
    sel.innerHTML = '<option value="">자동 할당</option>' + state.robotIds.map(function (id) {
      return '<option value="' + esc(id) + '">' + esc(id) + '</option>';
    }).join('');
    sel.value = current;
  }

  function postJson(url, body) {
    return fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    }).then(function (resp) {
      return resp.json().then(function (data) { return { status: resp.status, data: data }; });
    });
  }

  $('task-form').addEventListener('submit', function (e) {
    e.preventDefault();
    var f = e.target;
    // 서버가 /fleet/task_request 와이어 형식(pickup/dropoff, deadline = 지금부터 초)으로 정규화한다
    var body = {
      task_id: f.task_id.value.trim() || undefined,
      robot_id: f.robot_id.value || undefined,
      priority: parseInt(f.priority.value, 10),
      deadline_sec: parseFloat(f.deadline_sec.value),
      pickup: { x: parseFloat(f.pickup_x.value), y: parseFloat(f.pickup_y.value), yaw: parseFloat(f.pickup_yaw.value) },
      dropoff: { x: parseFloat(f.dropoff_x.value), y: parseFloat(f.dropoff_y.value), yaw: parseFloat(f.dropoff_yaw.value) },
      item_type: f.item_type.value
    };
    var out = $('task-result');
    out.className = 'result';
    out.textContent = '전송 중…';
    postJson('/api/tasks', body).then(function (r) {
      if (r.data.ok) {
        out.className = 'result ok';
        out.textContent = '투입됨: ' + JSON.stringify(r.data.task, null, 1);
        f.task_id.value = '';
      } else {
        out.className = 'result err';
        out.textContent = '거부 (' + r.status + '): ' + (r.data.errors || []).join('\n');
      }
    }).catch(function (err) {
      out.className = 'result err';
      out.textContent = '요청 실패: ' + err;
    });
  });

  // ----- E-stop -----

  function estopButton(target, label) {
    var active = !!state.estops[target];
    var cls = active ? 'btn release' : 'btn danger' + (target === 'all' ? ' big' : '');
    var text = active ? label + ' 해제' : label + ' 정지';
    return '<button type="button" class="' + cls + '" data-target="' + esc(target) + '" data-active="'
      + (active ? '0' : '1') + '">' + esc(text) + '</button>';
  }

  function renderEstops() {
    var html = '<div class="estop-row">' + estopButton('all', '전체') + '</div>';
    html += state.robotIds.map(function (id) {
      return '<div class="estop-row">' + estopButton(id, id) + '</div>';
    }).join('');
    $('estop-buttons').innerHTML = html;
  }

  $('estop-buttons').addEventListener('click', function (e) {
    var btn = e.target.closest('button[data-target]');
    if (!btn) { return; }
    var target = btn.getAttribute('data-target');
    var active = btn.getAttribute('data-active') === '1';
    var name = target === 'all' ? '전체 로봇' : target;
    var question = active
      ? name + ' 긴급 정지를 실행할까요? (즉시 정지)'
      : name + ' 긴급 정지를 해제할까요? (safety/reset_estop 호출)';
    if (!window.confirm(question)) { return; }
    var out = $('estop-result');
    out.textContent = '전송 중…';
    postJson('/api/estop', { robot_id: target, active: active }).then(function (r) {
      if (r.data.ok) {
        state.estops = r.data.estops || state.estops;
        renderEstops();
        out.textContent = name + ' → ' + (active ? '긴급 정지 활성' : '해제 요청됨');
      } else {
        out.textContent = '거부 (' + r.status + '): ' + (r.data.errors || []).join(', ');
      }
    }).catch(function (err) { out.textContent = '요청 실패: ' + err; });
  });

  // ----- 시작 -----

  renderEstops();
  resizeCanvas();
  draw();
  connect();
})();
