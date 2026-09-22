/* amr_dashboard 단일 페이지 앱 (vanilla JS, 외부 의존 없음)
 *
 * 데이터 흐름: EventSource('/events') → snapshot(전체, id = seq) → status / alerts / task_event / map_updated /
 * estop / task_request / heartbeat 이벤트로 state 갱신 → 해당 영역만 다시 그린다.
 * 이벤트 id 는 서버 전역 순번: 스냅샷 seq 이하는 이미 반영된 것이라 버리고, 번호가 건너뛰면(느린 연결에서
 * 서버 큐가 넘쳐 버려진 이벤트) GET /api/state 로 다시 맞춘다 — 그동안 온 이벤트는 모았다가 새 스냅샷보다
 * 새것만 적용한다. 이력 한도(limits)는 서버 값을 따른다.
 * 조작: POST /api/tasks (작업 투입), POST /api/estop (긴급 정지). 서버가 401 이면 조작 토큰을 물어
 * sessionStorage 에 두고 X-Dashboard-Token 헤더로 보낸다. 지도 이미지는 GET /api/map?format=png.
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
  var TASK_TTL_SEC = 600;          // 끝난(완료/실패) 작업은 이만큼 지나면 지도·목록에서 뺀다
  var TOKEN_KEY = 'amr_dashboard_token';
  var DEFAULT_LIMITS = { max_alerts: 100, max_task_events: 50, max_tasks: 200 };
  // E-stop·배정 대상 robot_id: 토픽 이름 토큰 (서버 state_store.ROBOT_NAME_RE 와 같다)
  var ROBOT_NAME_RE = /^[A-Za-z][A-Za-z0-9_]{0,63}$/;

  var state = {
    world: { origin_x: 0, origin_y: 0, width: 60, height: 40 },
    robotIds: [],
    status: null,
    alerts: [],
    taskEvents: [],
    tasks: {},
    map: null,
    estops: { all: false },
    estopDetail: null,
    limits: DEFAULT_LIMITS,
    lastSeq: 0,
    resyncing: false,
    pending: [],
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

  // 순번 검사: 'apply' = 적용, 'skip' = 이미 스냅샷에 있음(버림), 'gap' = 번호가 건너뜀(스냅샷을 다시 받는다).
  function acceptSeq(id) {
    if (isNaN(id)) { return 'apply'; }
    if (id <= state.lastSeq) { return 'skip'; }
    if (state.lastSeq && id > state.lastSeq + 1) { return 'gap'; }
    state.lastSeq = id;
    return 'apply';
  }

  // 재동기화 중 도착한 이벤트는 모아 두었다가, 받은 스냅샷(seq)보다 새것만 순서대로 적용한다
  // (스냅샷을 요청한 뒤·응답 전에 생긴 이벤트가 사라지거나 두 번 들어가지 않게).
  function replayPending() {
    var pending = state.pending;
    state.pending = [];
    pending.sort(function (a, b) { return (a.id || 0) - (b.id || 0); });
    pending.forEach(function (p) {
      if (isNaN(p.id)) { p.fn(p.data); return; }
      if (p.id > state.lastSeq) { state.lastSeq = p.id; p.fn(p.data); }
    });
  }

  function resync() {
    if (state.resyncing) { return; }
    state.resyncing = true;
    fetch('/api/state').then(function (r) { return r.json(); }).then(function (s) {
      applySnapshot(s);
      state.resyncing = false;
      replayPending();
    }).catch(function () {
      // 스냅샷을 못 받으면 모은 이벤트라도 적용한다 (다음 건너뜀에서 다시 시도)
      state.resyncing = false;
      var pending = state.pending;
      state.pending = [];
      pending.forEach(function (p) {
        if (!isNaN(p.id)) { state.lastSeq = Math.max(state.lastSeq, p.id); }
        p.fn(p.data);
      });
    });
  }

  function on(es, name, fn) {
    es.addEventListener(name, function (e) {
      touch();
      var item = { id: parseInt(e.lastEventId, 10), fn: fn, data: JSON.parse(e.data) };
      if (state.resyncing) { state.pending.push(item); return; }
      var verdict = acceptSeq(item.id);
      if (verdict === 'gap') {
        state.pending.push(item);
        resync();
      } else if (verdict === 'apply') {
        fn(item.data);
      }
    });
  }

  function connect() {
    var es = new EventSource('/events');
    es.addEventListener('snapshot', function (e) {
      touch();
      state.pending = [];              // 새 스트림: 이전 연결에서 모은 이벤트는 스냅샷에 들어 있다
      applySnapshot(JSON.parse(e.data), true);
    });
    on(es, 'status', applyStatus);
    on(es, 'alerts', applyAlerts);
    on(es, 'task_event', applyTaskEvent);
    on(es, 'map_updated', applyMapMeta);
    on(es, 'estop', applyEstop);
    on(es, 'task_request', function () {});
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

  // newStream: SSE 연결마다 오는 스냅샷 (서버가 다시 떴으면 순번이 처음부터라 그대로 받는다).
  // resync(GET /api/state) 는 같은 스트림 안이므로 순번을 되돌리지 않는다.
  function applySnapshot(s, newStream) {
    if (s.world) { state.world = s.world; }
    if (typeof s.seq === 'number' && (newStream || s.seq >= state.lastSeq)) { state.lastSeq = s.seq; }
    state.limits = s.limits || DEFAULT_LIMITS;
    state.robotIds = s.robot_ids || [];
    state.alerts = s.alerts || [];
    state.taskEvents = s.task_events || [];
    state.tasks = s.tasks || {};
    pruneTasks();
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
      // 토픽 이름으로 쓸 수 없는 id 는 표에만 보이고 E-stop 버튼·배정 목록에는 넣지 않는다 (서버가 400)
      if (r.robot_id && ROBOT_NAME_RE.test(r.robot_id) && state.robotIds.indexOf(r.robot_id) < 0) {
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
    state.alerts = state.alerts.concat(alerts).slice(-state.limits.max_alerts);
    renderAlerts();
  }

  function applyTaskEvent(ev) {
    state.taskEvents.push(ev);
    while (state.taskEvents.length > state.limits.max_task_events) { state.taskEvents.shift(); }
    delete state.tasks[ev.task_id];     // 삽입 순서 = 최근 순서 (서버 OrderedDict 와 같게)
    state.tasks[ev.task_id] = ev;
    pruneTasks();
    renderTimeline();
    draw();
  }

  // 끝난 작업은 TASK_TTL_SEC 뒤 빼고, 전체는 서버 한도(max_tasks)로 자른다 (4 시간 연속 운용 대비)
  function pruneTasks(nowSec) {
    var now = nowSec === undefined ? Date.now() / 1000 : nowSec;
    var ids = Object.keys(state.tasks);
    ids.forEach(function (id) {
      var t = state.tasks[id];
      var done = t.status_name === 'COMPLETED' || t.status_name === 'FAILED';
      if (done && now - (t.received_at || 0) > TASK_TTL_SEC) { delete state.tasks[id]; }
    });
    ids = Object.keys(state.tasks);
    var extra = ids.length - state.limits.max_tasks;
    for (var i = 0; i < extra; i++) { delete state.tasks[ids[i]]; }
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

  function latchedText(detail) {
    var latched = (detail && detail.still_latched) || [];
    return latched.length ? ' — 해제 안 됨(래치 유지): ' + latched.join(', ') : '';
  }

  function applyEstop(data) {
    state.estops = data.estops || state.estops;
    state.estopDetail = data.detail || null;
    renderEstops();
    var latched = latchedText(data.detail);
    $('estop-result').textContent = fmtTime(data.ts) + ' ' + data.target + ' → '
      + (data.active ? '긴급 정지 활성' : (latched ? '해제 요청' : '해제')) + latched;
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

  function storedToken() {
    try { return window.sessionStorage.getItem(TOKEN_KEY) || ''; } catch (err) { return ''; }
  }

  function storeToken(token) {
    try { window.sessionStorage.setItem(TOKEN_KEY, token); } catch (err) { /* 저장 불가: 이번만 */ }
  }

  function readJson(resp) {
    // 서버 오류 페이지(HTML) 도 {errors} 형태로 돌려준다 — resp.json() 예외로 UI 가 멈추지 않게
    return resp.text().then(function (text) {
      var data;
      try { data = JSON.parse(text); } catch (err) {
        data = { ok: false, errors: ['HTTP ' + resp.status + ' (JSON 아님)'] };
      }
      return { status: resp.status, data: data };
    });
  }

  function postJson(url, body, retried) {
    var headers = { 'Content-Type': 'application/json' };
    var token = storedToken();
    if (token) { headers['X-Dashboard-Token'] = token; }
    return fetch(url, {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(body)
    }).then(readJson).then(function (r) {
      if (r.status === 401 && !retried) {
        var entered = window.prompt('조작 토큰 (X-Dashboard-Token) 을 입력한다', '');
        if (entered) {
          storeToken(entered.trim());
          return postJson(url, body, true);
        }
      }
      return r;
    });
  }

  $('task-form').addEventListener('submit', function (e) {
    e.preventDefault();
    var f = e.target;
    // 서버가 /fleet/task_request 와이어 형식(pickup/dropoff, deadline = 지금부터 초)으로 정규화한다
    // 마감은 선택: 비우면 키를 빼서 "마감 없음" 으로 보낸다 (0 이하는 서버가 거부한다)
    var deadline = String(f.deadline_sec.value || '').trim();
    var body = {
      task_id: f.task_id.value.trim() || undefined,
      robot_id: f.robot_id.value || undefined,
      priority: parseInt(f.priority.value, 10),
      deadline_sec: deadline === '' ? undefined : parseFloat(deadline),
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
    // 전체 E-stop 중에는 로봇별 해제를 막는다 (서버도 409) — 전체를 먼저 푼다
    var blocked = active && target !== 'all' && !!state.estops.all;
    return '<button type="button" class="' + cls + '" data-target="' + esc(target) + '" data-active="'
      + (active ? '0' : '1') + '"' + (blocked ? ' disabled title="전체 E-stop 해제 후 가능"' : '')
      + '>' + esc(text) + '</button>';
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
    if (!btn || btn.disabled) { return; }
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
      // 502(리셋 실패)·409(밀림) 도 서버의 실제 표시 상태(estops)를 싣는다 — 그대로 반영한다
      if (r.data.estops) {
        state.estops = r.data.estops;
        renderEstops();
      }
      if (r.data.ok) {
        out.textContent = name + ' → ' + (active ? '긴급 정지 활성' : '해제됨 (reset_estop 확인)');
      } else {
        out.textContent = (r.status === 502 ? '해제 실패' : '거부') + ' (' + r.status + '): '
          + (r.data.errors || []).join(', ');
      }
    }).catch(function (err) { out.textContent = '요청 실패: ' + err; });
  });

  // ----- 시작 -----

  renderEstops();
  resizeCanvas();
  draw();
  connect();

  // 헤드리스 검사용 훅 (브라우저 동작에는 영향 없음)
  window.__amrDashboard = { state: state, esc: esc, pruneTasks: pruneTasks, acceptSeq: acceptSeq };
})();
