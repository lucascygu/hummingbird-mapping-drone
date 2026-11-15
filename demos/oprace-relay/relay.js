/* A task-level model. Coordinates are metres on a tabletop course, with y pointing down. */
(function () {
  "use strict";

  const points = {
    sample: [0.74, 2.00], door: [1.62, 2.00],
    northWest: [2.04, 1.12], northEast: [3.18, 1.12],
    southWest: [1.96, 3.03], southEast: [3.25, 3.03],
    fork: [3.63, 2.00], pad: [4.31, 2.00], clear: [4.35, 2.76],
    nest: [4.27, 0.68], northView: [2.61, 1.12], southView: [2.60, 3.03],
    receiver: [6.36, 2.00]
  };
  const edges = [
    {a: "pad", b: "fork"},
    {a: "fork", b: "northEast"}, {a: "northEast", b: "northWest", gate: "north"},
    {a: "northWest", b: "door"},
    {a: "fork", b: "southEast"}, {a: "southEast", b: "southWest", gate: "south"},
    {a: "southWest", b: "door"}, {a: "door", b: "sample", gate: "cover"},
    {a: "pad", b: "clear"}
  ];
  // The planner sees the candidate corridors and its observations. Only an
  // observation in simulate() consults a gate state. Field rehearsals supply it explicitly.
  const layouts = {
    south: {label: "南口通行", north: false, south: true, cover: true},
    north: {label: "北口通行", north: true, south: false, cover: true},
    both: {label: "双口通行", north: true, south: true, cover: true},
    closed: {label: "双口封闭", north: false, south: false, cover: true},
    cover: {label: "棚内封闭", north: true, south: true, cover: false}
  };
  const policies = {B0: "小鸭子自探", B1: "蜂鸟固定巡看", B2: "小鸭子按需问路"};
  const gates = {north: "北口", south: "南口", cover: "棚内"};
  const stations = {sample: "样品架", duck: "小鸭子", pad: "交接垫", bird: "蜂鸟", receiver: "收件台"};
  const views = {north: "northView", south: "southView"};
  const limits = {scoutReads: 2, scoutMetres: 10};
  const speed = {duck: 0.15, bird: 0.8}; // Model assumptions, not measurements.
  const sampleId = "HB-01";

  function distance(a, b) {
    return Math.hypot(points[a][0] - points[b][0], points[a][1] - points[b][1]);
  }

  function edgeOf(a, b) {
    return edges.find(e => (e.a === a && e.b === b) || (e.a === b && e.b === a));
  }

  // Unknown gates may appear in a proposed route. walk() requires a positive
  // observation of the gate before the simulated body actually crosses it.
  function route(from, to, known) {
    const best = new Map([[from, 0]]);
    const previous = new Map();
    const todo = new Set([from]);
    while (todo.size) {
      let node;
      for (const candidate of todo) {
        if (node === undefined || best.get(candidate) < best.get(node)) node = candidate;
      }
      if (node === to) {
        const result = [node];
        while (previous.has(node)) {
          node = previous.get(node);
          result.push(node);
        }
        return result.reverse();
      }
      todo.delete(node);
      for (const edge of edges) {
        if (edge.gate && known[edge.gate] === false) continue;
        const next = edge.a === node ? edge.b : edge.b === node ? edge.a : null;
        if (!next) continue;
        const cost = best.get(node) + distance(node, next);
        if (cost >= (best.get(next) ?? Infinity)) continue;
        best.set(next, cost);
        previous.set(next, node);
        todo.add(next);
      }
    }
    return [];
  }

  function firstOutdoorGate(path) {
    for (let i = 1; i < path.length; i++) {
      const gate = edgeOf(path[i - 1], path[i]).gate;
      if (gate === "north" || gate === "south") return gate;
    }
    return null;
  }

  function simulate(layoutId = "south", policy = "B2", reports = {}) {
    const field = layoutId === "field";
    if ((!field && !Object.hasOwn(layouts, layoutId)) || !Object.hasOwn(policies, policy)) {
      throw new Error("Unknown course or observation policy");
    }
    const world = field ? reports : layouts[layoutId];
    let pending = null;
    const known = {north: null, south: null, cover: null};
    let duck = "pad", bird = "nest", landed = true, owner = "sample", status = "running";
    let time = 0, duckM = 0, scoutM = 0, transportM = 0, reads = 0, mapVersion = 0;
    let toPadM = null, toPadS = null;
    const frames = [];

    function currentPath() {
      if (owner === "sample") return route(duck, "sample", known);
      if (owner === "duck") return route(duck, "pad", known);
      return [];
    }

    function record(kind, actor, title, detail, duration = 0, extra = {}) {
      time += duration;
      frames.push({
        seq: frames.length, time, duration, kind, actor, title, detail, ...extra,
        duck, bird, landed, owner, status, sampleId, mapVersion, known: {...known},
        plan: currentPath(), duckM, scoutM, transportM, reads, toPadM, toPadS
      });
    }

    function fly(destination, purpose, title) {
      const from = bird;
      const metres = distance(from, destination);
      if (purpose === "survey") scoutM += metres;
      else transportM += metres;
      bird = destination;
      landed = ["nest", "pad", "receiver"].includes(bird);
      const detail = purpose === "survey" ? "小鸭子停在原地，等蜂鸟返回。" :
        destination === "pad" ? "小鸭子已退出交接垫，样品留在垫上。" : "同一只样品盒随蜂鸟抵达沟另一侧。";
      record("fly", "bird", title, detail, metres / speed.bird, {from, to: bird, purpose});
    }

    function walk(destination, title) {
      const from = duck;
      const edge = edgeOf(from, destination);
      if (!edge || (edge.gate && known[edge.gate] !== true)) throw new Error("Unverified crossing");
      if (!landed) throw new Error("Ground movement overlaps a simulated flight");
      const metres = distance(from, destination);
      duckM += metres;
      duck = destination;
      const detail = owner === "duck" ? sampleId + " 随小鸭子沿已走通的路线返回。" :
        destination === "clear" ? "样品留在垫上，蜂鸟现在可以进场。" : "沿当前候选路线行走，未知关口必须先观察。";
      record("walk", "duck", title, detail, metres / speed.duck, {from, to: duck});
    }

    function observe(gate, actor) {
      if (known[gate] !== null) throw new Error("Duplicate gate observation");
      if (actor === "bird") {
        if (!views[gate] || bird !== views[gate]) throw new Error("Gate outside aerial view");
      } else {
        const edge = edges.find(e => e.gate === gate);
        if (duck !== edge.a && duck !== edge.b) throw new Error("Gate outside ground view");
      }
      if (!Object.hasOwn(world, gate)) {
        pending = {gate, actor};
        return false;
      }
      if (typeof world[gate] !== "boolean") throw new Error("Gate reports must be boolean");
      if (actor === "bird") reads++;
      const before = firstOutdoorGate(currentPath());
      known[gate] = world[gate];
      mapVersion++;
      const after = firstOutdoorGate(currentPath());
      let detail = actor === "bird" ? "蜂鸟把路口观测交给地面规划器。" : "小鸭子从近处确认，把结果补回共享地图。";
      if (before && after && before !== after) detail += " 候选路线从" + gates[before] + "改走" + gates[after] + "。";
      if (!currentPath().length) detail += " 已知通道中没有可达路线。";
      record("observe", actor, gates[gate] + (known[gate] ? "可通行" : "已封闭"), detail, 1,
        {gate, value: known[gate]});
      return true;
    }

    function survey(list) {
      const trip = [bird, ...list.map(g => views[g]), "nest"];
      const length = trip.slice(1).reduce((sum, point, i) => sum + distance(trip[i], point), 0);
      if (reads + list.length > limits.scoutReads || scoutM + length > limits.scoutMetres) return false;
      if (policy === "B2") {
        record("request", "duck", "小鸭子请求查看" + gates[list[0]], "分叉前方状态未知，先确认候选通道。");
      } else {
        record("schedule", "bird", "蜂鸟按固定顺序巡看", "起步前依次查看北口、南口，再把地图交给小鸭子。");
      }
      for (const gate of list) {
        fly(views[gate], "survey", "蜂鸟前往" + gates[gate] + "观察位");
        if (!observe(gate, "bird")) return true;
      }
      fly("nest", "survey", "蜂鸟返回停机位");
      return true;
    }

    function transfer(to, title, detail, duration) {
      const from = owner;
      const expected = {duck: "sample", pad: "duck", bird: "pad", receiver: "bird"};
      const station = {duck: "sample", pad: "pad", bird: "pad", receiver: "receiver"}[to];
      const groundReady = (to === "duck" || to === "pad") && duck === station && landed;
      const birdReady = (to === "bird" || to === "receiver") && bird === station && landed && duck === "clear";
      if (expected[to] !== from || !(groundReady || birdReady)) throw new Error("Invalid sample handoff");
      owner = to;
      if (to === "pad") {
        toPadM = duckM;
        toPadS = time + duration;
      }
      if (to === "receiver") status = "delivered";
      record("transfer", to === "duck" || to === "pad" ? "duck" : "bird", title, detail, duration,
        {custody: {id: sampleId, from, to, station}});
    }

    record("start", "station", "任务就绪", "样品位置和候选走廊已知，三个关口都还没有观测记录。");
    if (policy === "B1") survey(["north", "south"]);

    while (!pending && (owner === "sample" || owner === "duck")) {
      const path = currentPath();
      if (!path.length) {
        status = "blocked";
        record("stop", "station", "通道封闭，任务停止", "样品保留在最近确认的位置，本次没有签收。");
        break;
      }
      if (path.length === 1) {
        if (owner === "sample") {
          transfer("duck", "小鸭子取到 " + sampleId, "已进入低棚，样品从架上转到小鸭子。", 2);
        } else {
          transfer("pad", "样品交到 H 垫", "记录样品编号和到垫时间，等待蜂鸟接件。", 2);
        }
        continue;
      }
      const gateAhead = firstOutdoorGate(path);
      if (policy === "B2" && duck === "fork" && owner === "sample" && gateAhead && known[gateAhead] === null) {
        if (survey([gateAhead])) continue;
      }
      const next = path[1];
      const gate = edgeOf(duck, next).gate;
      if (gate && known[gate] === null) {
        observe(gate, "duck");
        continue; // A blocked edge changes the path before any movement.
      }
      const label = {fork: "小鸭子到达分叉", door: "小鸭子到达低棚入口", sample: "小鸭子进入样品位", pad: "小鸭子返回交接垫"};
      walk(next, label[next] || (owner === "duck" ? "小鸭子带件返回" : "小鸭子沿候选通道行走"));
    }
    if (owner === "pad") {
      walk("clear", "小鸭子退出交接垫");
      fly("pad", "transfer", "蜂鸟到垫并落稳");
      transfer("bird", "蜂鸟接收 " + sampleId, "在垫上核对编号，样品转由蜂鸟保管。", 2);
      fly("receiver", "transfer", "蜂鸟携件越过断路");
      transfer("receiver", "收件台签收 " + sampleId, "蜂鸟已落稳，收件端核对同一编号，交付完成。", 2);
    }
    return {version: 1, layoutId, policy, sampleId, pending, units: {distance: "m", time: "s"}, frames};
  }

  const api = {points, edges, layouts, policies, gates, stations, limits, speed, sampleId, distance, route, simulate};
  if (typeof module === "object" && module.exports) module.exports = api;
  else globalThis.OpraceRelay = api;
})();
