#!/usr/bin/env node
// web/app.js 헤드리스 단위 검사: DOM·EventSource·fetch·sessionStorage 대역 위에서 실제 app.js 를 돌려
// 순번 중복/건너뜀, 이력 한도, 작업 정리, E-stop 버튼·결과 표시, 토큰 재시도, 마감 선택 입력을 확인한다.
// 사용: node app_unit.js <web/app.js>   (test_app_js.py 가 node 가 있을 때 부른다)
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const APP_JS = process.argv[2] || path.join(__dirname, '..', '..', 'web', 'app.js');
const results = [];
function check(name, cond, extra) {
  results.push([name, !!cond]);
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${cond || extra === undefined ? '' : '  → ' + extra}`);
}

// ----- DOM 대역 -----
const elements = {};
function makeCtx(canvas) {
  return new Proxy({ canvas }, {
    get(t, p) { return p in t ? t[p] : () => {}; },
    set(t, p, v) { t[p] = v; return true; },
  });
}
function el(id) {
  if (!elements[id]) {
    elements[id] = {
      id, textContent: '', innerHTML: '', className: '', value: '', style: {}, width: 900, height: 600,
      listeners: {}, parentElement: { clientWidth: 1200 },
      addEventListener(type, fn) { (this.listeners[type] = this.listeners[type] || []).push(fn); },
      getContext() { return this._ctx || (this._ctx = makeCtx(this)); },
    };
  }
  return elements[id];
}
const store = {};
const prompts = [];
const sandbox = {
  console, setInterval: () => 0, setTimeout, Date, JSON, Math, Number, String, Object, parseInt,
  parseFloat, isNaN, Promise,
  document: { getElementById: el },
  window: {
    devicePixelRatio: 1, addEventListener() {}, confirm: () => true,
    prompt: (q) => { prompts.push(q); return 'sekret'; },
    sessionStorage: {
      getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); },
    },
  },
  Image: class { set src(v) { this._src = v; } },
};

// ----- EventSource / fetch 대역 -----
let es = null;
sandbox.EventSource = class {
  constructor(url) { this.url = url; this.l = {}; es = this; }
  addEventListener(type, fn) { (this.l[type] = this.l[type] || []).push(fn); }
  emit(type, data, id) {
    (this.l[type] || []).forEach((fn) => fn({ data: JSON.stringify(data), lastEventId: id === undefined ? '' : String(id) }));
  }
};
const requests = [];
const responders = [];
sandbox.fetch = (url, opts) => {
  requests.push({ url, opts: opts || {} });
  const r = (responders.shift() || (() => ({ status: 200, body: { ok: true } })))(url, opts || {});
  const text = typeof r.body === 'string' ? r.body : JSON.stringify(r.body);
  return Promise.resolve({ status: r.status, text: () => Promise.resolve(text), json: () => Promise.resolve(JSON.parse(text)) });
};
sandbox.window.EventSource = sandbox.EventSource;
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(APP_JS, 'utf-8'), sandbox, { filename: 'app.js' });
const hooks = sandbox.window.__amrDashboard;
const S = hooks.state;
const tick = () => new Promise((r) => setTimeout(r, 0));

function snapshot(extra) {
  return Object.assign({
    server_time: 1000, seq: 10, limits: { max_alerts: 3, max_task_events: 2, max_tasks: 3 },
    world: { origin_x: 0, origin_y: 0, width: 60, height: 40 }, robot_ids: ['amr_01', 'amr_02'],
    status: null, alerts: [], task_events: [], tasks: {}, map: null, estops: { all: false },
  }, extra || {});
}
function alert(name) { return { level: 1, level_name: 'WARN', name, message: 'm', hardware_id: 'x', received_at: Date.now() / 1000 }; }
function task(id, st, received) { return { task_id: id, status_name: st, received_at: received, pickup_pose: { x: 1, y: 1 }, dropoff_pose: { x: 2, y: 2 }, robot_id: '', item_type: 'small', priority: 1 }; }
function click(target, active) {
  const btn = { disabled: false, getAttribute: (n) => ({ 'data-target': target, 'data-active': active ? '1' : '0' }[n]) };
  const html = el('estop-buttons').innerHTML;
  const m = html.match(new RegExp(`<button[^>]*data-target="${target}"[^>]*>`));
  btn.disabled = !!(m && / disabled/.test(m[0]));
  el('estop-buttons').listeners.click[0]({ target: { closest: () => btn } });
  return btn;
}

(async () => {
  check('esc() escapes html', hooks.esc('<a href="x">&</a>') === '&lt;a href=&quot;x&quot;&gt;&amp;&lt;/a&gt;');

  // 1) 스냅샷 → 순번 중복 버림, 연속 적용, 건너뜀이면 /api/state 재동기화
  es.emit('snapshot', snapshot({ alerts: [alert('a0')] }), 10);
  check('snapshot seq/limits applied', S.lastSeq === 10 && S.limits.max_alerts === 3);
  es.emit('alerts', [alert('dup')], 9);
  es.emit('alerts', [alert('dup10')], 10);
  check('events with id <= snapshot seq ignored', S.alerts.length === 1, JSON.stringify(S.alerts.map((a) => a.name)));
  es.emit('alerts', [alert('a1'), alert('a2'), alert('a3')], 11);
  check('alerts capped at limits.max_alerts', S.alerts.length === 3 && S.alerts[2].name === 'a3');
  responders.push(() => ({ status: 200, body: snapshot({ seq: 20, alerts: [alert('resynced')] }) }));
  es.emit('alerts', [alert('after_gap')], 14);
  await tick(); await tick();
  check('gap in ids triggers GET /api/state resync', requests.some((r) => r.url === '/api/state') && S.lastSeq === 20
    && S.alerts[0].name === 'resynced', JSON.stringify({ seq: S.lastSeq, reqs: requests.map((r) => r.url) }));
  es.emit('snapshot', snapshot({ seq: 2 }), 2);          // 서버 재시작: 순번이 처음부터
  check('new stream snapshot resets seq', S.lastSeq === 2);

  // 2) 작업 정리: 끝난 작업 TTL, 개수 한도
  const now = Date.now() / 1000;
  es.emit('task_event', task('old_done', 'COMPLETED', now - 3600), 3);
  es.emit('task_event', task('t1', 'IN_PROGRESS', now), 4);
  es.emit('task_event', task('t2', 'PENDING', now), 5);
  check('completed task older than TTL evicted', !('old_done' in S.tasks) && 't1' in S.tasks);
  es.emit('task_event', task('t3', 'PENDING', now), 6);
  es.emit('task_event', task('t4', 'PENDING', now), 7);
  check('tasks capped at limits.max_tasks (oldest dropped)', Object.keys(S.tasks).join(',') === 't2,t3,t4',
    Object.keys(S.tasks).join(','));
  check('timeline capped at limits.max_task_events', S.taskEvents.length === 2);

  // 3) E-stop: 전체 E-stop 중 로봇별 해제 버튼 비활성, 누르면 요청 없음
  es.emit('estop', { target: 'all', active: true, estops: { all: true, amr_01: true }, ts: now }, 8);
  const html = el('estop-buttons').innerHTML;
  check('per-robot release disabled while fleet-wide E-stop active',
    /data-target="amr_01" data-active="0" disabled/.test(html) && !/data-target="all"[^>]*disabled/.test(html), html);
  const before = requests.length;
  click('amr_01', false);
  await tick();
  check('clicking disabled button sends nothing', requests.length === before);

  // 4) 해제 실패(502) → 실패 문구 + 서버가 준 estops 반영 (amr_02 래치 유지)
  responders.push(() => ({ status: 502, body: { ok: false, errors: ['amr_02: reset_estop no_server'],
    estops: { all: false, amr_01: false, amr_02: true }, detail: { still_latched: ['amr_02'] } } }));
  click('all', false);
  await tick(); await tick();
  check('502 release shows failure text', /^해제 실패 \(502\): amr_02: reset_estop no_server/.test(el('estop-result').textContent),
    el('estop-result').textContent);
  check('502 release keeps latched robot stopped', /data-target="amr_02" data-active="0"/.test(el('estop-buttons').innerHTML)
    && S.estops.all === false);
  es.emit('estop', { target: 'all', active: false, estops: { all: false, amr_02: true }, detail: { still_latched: ['amr_02'] }, ts: now }, 9);
  check('SSE estop event shows still-latched robots', /all → 해제 요청 — 해제 안 됨\(래치 유지\): amr_02/.test(el('estop-result').textContent),
    el('estop-result').textContent);

  // 5) 토큰: 401 → prompt → X-Dashboard-Token 으로 재시도, 다음부터는 묻지 않음
  responders.push(() => ({ status: 401, body: { ok: false, errors: ['token'], auth: 'token' } }));
  responders.push((url, opts) => ({ status: opts.headers['X-Dashboard-Token'] === 'sekret' ? 200 : 401,
    body: { ok: true, target: 'amr_01', active: true, estops: { all: false, amr_01: true, amr_02: true } } }));
  click('amr_01', true);
  await tick(); await tick(); await tick();
  const last = requests[requests.length - 1];
  check('401 prompts for token and retries with header', prompts.length === 1 && last.opts.headers['X-Dashboard-Token'] === 'sekret'
    && /긴급 정지 활성/.test(el('estop-result').textContent), el('estop-result').textContent);
  responders.push((url, opts) => ({ status: 200, body: { ok: true, estops: S.estops, target: 'amr_01', active: true } }));
  click('amr_01', true);
  await tick(); await tick();
  check('stored token reused without prompting', prompts.length === 1
    && requests[requests.length - 1].opts.headers['X-Dashboard-Token'] === 'sekret');

  // 6) 작업 폼: 마감 비우면 deadline_sec 키 없음, 503 은 거부로 표시, HTML 오류 페이지도 처리
  const form = {};
  for (const [k, v] of Object.entries({ task_id: '', robot_id: '', priority: 5, deadline_sec: '  ', item_type: 'small',
    pickup_x: 1, pickup_y: 2, pickup_yaw: 0, dropoff_x: 3, dropoff_y: 4, dropoff_yaw: 0 })) { form[k] = { value: String(v) }; }
  responders.push(() => ({ status: 503, body: { ok: false, errors: ['/fleet/task_request 구독자가 없다'] } }));
  el('task-form').listeners.submit[0]({ preventDefault() {}, target: form });
  await tick(); await tick();
  const body = JSON.parse(requests[requests.length - 1].opts.body);
  check('empty deadline omitted from request', !('deadline_sec' in body), JSON.stringify(body));
  check('503 shown as rejection', /^거부 \(503\): \/fleet\/task_request 구독자가 없다/.test(el('task-result').textContent),
    el('task-result').textContent);
  form.deadline_sec.value = '30';
  responders.push(() => ({ status: 500, body: '<html>Internal Server Error</html>' }));
  el('task-form').listeners.submit[0]({ preventDefault() {}, target: form });
  await tick(); await tick();
  check('deadline sent as number when given', JSON.parse(requests[requests.length - 1].opts.body).deadline_sec === 30);
  check('non-JSON error page handled', /거부 \(500\): HTTP 500 \(JSON 아님\)/.test(el('task-result').textContent),
    el('task-result').textContent);

  // 6b) 토픽 이름으로 쓸 수 없는 robot_id 는 E-stop 버튼·배정 목록에 넣지 않는다 (표에는 보인다)
  const robot = (id) => ({ robot_id: id, status_name: 'IDLE', status: 0, battery_level: 80,
    pose: { x: 1, y: 1, yaw: 0 }, current_task_id: '' });
  es.emit('status', { robots: [robot('amr_01'), robot('rv-bad'), robot('amr_07')], kpi: {} }, 10);
  check('invalid robot id gets no E-stop button', !/data-target="rv-bad"/.test(el('estop-buttons').innerHTML)
    && /data-target="amr_07"/.test(el('estop-buttons').innerHTML) && S.robotIds.indexOf('rv-bad') < 0,
    JSON.stringify(S.robotIds));

  // 7) 재동기화 중 도착한 이벤트: 모았다가 새 스냅샷(seq 104)보다 새것(105)만 적용 — 유실·중복 없음
  const wide = { max_alerts: 10, max_task_events: 2, max_tasks: 3 };
  es.emit('snapshot', snapshot({ seq: 100, limits: wide }), 100);
  responders.push(() => ({ status: 200, body: snapshot({ seq: 104, limits: wide,
    alerts: [alert('s102'), alert('s103'), alert('s104')] }) }));
  const nReq = requests.length;
  es.emit('alerts', [alert('g102')], 102);             // 101 이 빠짐 → 재동기화 시작
  es.emit('alerts', [alert('d104')], 104);             // 응답 전 도착, 스냅샷에 이미 있음 → 버림
  es.emit('alerts', [alert('a105')], 105);             // 응답 전 도착, 스냅샷보다 새것 → 적용
  check('events during resync are buffered, not applied yet', S.resyncing && S.alerts.length === 0);
  await tick(); await tick(); await tick();
  const names = S.alerts.map((a) => a.name).join(',');
  check('buffered events replayed after resync without loss or duplicates',
    requests.length === nReq + 1 && names === 's102,s103,s104,a105' && S.lastSeq === 105 && !S.resyncing, names);
  es.emit('alerts', [alert('a106')], 106);
  check('in-order events after resync applied directly', S.alerts.length === 5 && S.lastSeq === 106);

  const failed = results.filter(([, ok]) => !ok).length;
  console.log(failed ? `JS UNIT FAIL (${failed})` : `JS UNIT PASS (${results.length})`);
  process.exit(failed ? 1 : 0);
})().catch((e) => { console.log('JS UNIT ERROR', e && e.stack || e); process.exit(2); });
