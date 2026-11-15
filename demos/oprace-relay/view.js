(() => {
  "use strict";
  const model = globalThis.OpraceRelay;
  const get = id => document.getElementById(id);
  const canvas = get("map"), ctx = canvas.getContext("2d");
  const layout = get("layout"), policy = get("policy");
  const colors = {duck: "#f6c16c", bird: "#74ddd0", plan: "#dfe8cf", pale: "#d5e2d4", ink: "#18382f"};
  const chain = ["sample", "duck", "pad", "bird", "receiver"];
  let run, index = 0, playing = false, elapsed = 0, last = null, requestId = null;
  let width = 0, height = 0, ratio = 1;
  let reports = {}, confirmations = [];
  const reduced = window.matchMedia("(prefers-reduced-motion: reduce)");

  function mapPoint(point) {
    return [point[0] * width / 7.12, point[1] * height / 3.88];
  }
  function pointOf(name) { return mapPoint(model.points[name]); }
  function mix(a, b, p) { return [a[0] + p * (b[0] - a[0]), a[1] + p * (b[1] - a[1])]; }

  function line(positions, color, size = 1, dash = []) {
    if (positions.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = size; ctx.setLineDash(dash);
    ctx.lineCap = "round"; ctx.lineJoin = "round"; ctx.beginPath();
    positions.forEach(([x, y], i) => { if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke(); ctx.restore();
  }

  function polygon(vertices, fill, stroke) {
    ctx.beginPath();
    vertices.map(mapPoint).forEach(([x, y], i) => { if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.closePath(); ctx.fillStyle = fill; ctx.fill();
    if (stroke) {ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke();}
  }

  function circle(x, y, r, fill, stroke, size = 1) {
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    if (fill) {ctx.fillStyle = fill; ctx.fill();}
    if (stroke) {ctx.strokeStyle = stroke; ctx.lineWidth = size; ctx.stroke();}
  }

  function label(text, x, y, options = {}) {
    const [px, py] = mapPoint([x, y]);
    const size = options.size || (width < 470 ? 10 : 12);
    ctx.save(); ctx.font = (options.bold ? "600 " : "400 ") + size + 'px "Noto Sans CJK SC","PingFang SC","Microsoft YaHei",sans-serif';
    ctx.textAlign = options.align || "center"; ctx.textBaseline = "middle";
    if (options.back) {
      const w = ctx.measureText(text).width;
      ctx.fillStyle = options.back; ctx.fillRect(px - w / 2 - 3, py - size / 2 - 2, w + 6, size + 4);
    }
    ctx.fillStyle = options.color || colors.pale; ctx.fillText(text, px, py); ctx.restore();
  }

  function terrain() {
    ctx.fillStyle = "#16362e"; ctx.fillRect(0, 0, width, height);
    for (let x = 0; x <= 7.12; x += 0.25) line([mapPoint([x, 0]), mapPoint([x, 3.88])], "#22443a", 0.6);
    for (let y = 0; y <= 3.88; y += 0.25) line([mapPoint([0, y]), mapPoint([7.12, y])], "#22443a", 0.6);
    const leftBank = [[4.98,0],[5.08,.55],[4.97,1.14],[5.06,1.67],[4.96,2.28],[5.07,2.91],[4.97,3.37],[5.05,3.88]];
    const rightBank = [[5.76,0],[5.87,.55],[5.75,1.14],[5.85,1.67],[5.75,2.28],[5.85,2.91],[5.76,3.37],[5.84,3.88]];
    polygon([...leftBank, ...rightBank.slice().reverse()], "#3e4339");
    line(leftBank.map(mapPoint), "#9b8c6a", 1.4); line(rightBank.map(mapPoint), "#9b8c6a", 1.4);
    for (let y = 0; y < 3.8; y += 0.20) {
      const offset = 0.07 * Math.sin(y * 6);
      line([mapPoint([5.17 + offset, y]), mapPoint([5.43 + offset, y + .21])], "#64644d", 1);
    }
    polygon([[1.95,1.53],[2.45,1.39],[2.96,1.5],[3.16,1.91],[3.06,2.48],[2.64,2.73],[2.08,2.56],[1.92,2.22]], "#364838", "#546148");
    const stones = [[2.17,1.75],[2.54,1.62],[2.81,1.75],[2.14,2.27],[2.36,2.42],[2.70,2.44],[2.88,2.20],[2.36,2.12]];
    for (const [x,y] of stones) polygon([[x-.07,y],[x-.04,y-.06],[x+.038,y-.075],[x+.073,y+.008],[x+.024,y+.038]], "#495b45");
    const roof = [[.25,1.50],[1.44,1.50],[1.44,2.51],[.25,2.51]];
    polygon(roof, "#294941", "#739185");
    ctx.save(); ctx.clip();
    for (let x = -1; x < 2.5; x += .16) line([mapPoint([x,1.42]),mapPoint([x+.62,2.62])], "#3d5b50", 1);
    ctx.restore();
    const units = width / 7.12;
    for (const e of model.edges) {
      const coords = [pointOf(e.a),pointOf(e.b)];
      line(coords, "#627564", Math.max(5, units * .115)); line(coords, "#304c3e", Math.max(3, units * .085));
    }
    label("碎石带",2.54,2.01,{color:"#b2bc98"});
    label("断路",5.42,3.36,{color:"#d9cba6",back:"#42463a"});
    const [sx,sy] = mapPoint([.28,3.55]);
    line([[sx,sy],[sx+units,sy]],"#b7c7b6",1.3);
    line([[sx,sy-3],[sx,sy+2]],"#b7c7b6",1.3);
    line([[sx+units,sy-3],[sx+units,sy+2]],"#b7c7b6",1.3);
    label("1 m",.78,3.70,{size:10,color:"#becbbd"});
  }

  function stations() {
    const rad = Math.max(7, Math.min(15, width / 56));
    for (const station of ["nest","pad","receiver"]) {
      const [x,y] = pointOf(station);
      ctx.save(); ctx.setLineDash([3,3]);
      circle(x,y,rad + 4,"#204639","#8cae93",1); ctx.restore();
      if (station !== "nest") {
        line([[x-rad*.4,y],[x+rad*.4,y]],"#8cae93",1);
        line([[x,y-rad*.4],[x,y+rad*.4]],"#8cae93",1);
      }
    }
    label("低棚取件",.82,1.29,{bold:true});
    label("停机位",4.27,.36,{color:"#b1c4b2"});
    label("H 交接垫",4.27,1.61,{bold:true,back:"#17372f"});
    label("R 收件台",6.34,1.61,{bold:true,back:"#17372f"});
  }

  function drawGate(gate, state, where, labelY) {
    const [px,py] = mapPoint(where);
    const r = width < 470 ? 5.8 : 7.5;
    const color = state === null ? "#c8d8be" : state ? "#95dec2" : "#ffa894";
    circle(px,py,r,colors.ink,color,1.5);
    if (state === null) {
      ctx.save(); ctx.font = "bold " + (r * 1.45) + "px monospace"; ctx.fillStyle = color;
      ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText("?",px,py+.3); ctx.restore();
    } else if (state) {
      line([[px-r*.55,py],[px-r*.12,py+r*.4],[px+r*.6,py-r*.47]],color,1.5);
    } else {
      line([[px-r*.45,py-r*.45],[px+r*.45,py+r*.45]],color,1.5);
      line([[px-r*.45,py+r*.45],[px+r*.45,py-r*.45]],color,1.5);
    }
    if (labelY) label(model.gates[gate],where[0],labelY,{color,back:"#17372f"});
  }

  function duckSymbol([x,y], direction) {
    const s = width < 470 ? .71 : .93;
    ctx.save(); ctx.translate(x,y); ctx.scale(direction * s,s);
    circle(0,0,13,"#17392f",colors.duck,1.5);
    ctx.fillStyle = colors.duck; ctx.beginPath(); ctx.ellipse(-1,3,6.5,4.2,0,0,Math.PI*2); ctx.fill();
    circle(4,-3,4.3,colors.duck); circle(5.2,-4,1,colors.ink);
    ctx.beginPath(); ctx.moveTo(7,-4); ctx.lineTo(11,-2.5); ctx.lineTo(7,-1); ctx.fillStyle=colors.duck;ctx.fill();
    line([[-4,7],[-4,9]],colors.duck,1.5);line([[1,7],[1,9]],colors.duck,1.5);
    ctx.restore();
  }

  function birdSymbol([x,y]) {
    const s = width < 470 ? .76 : .99;
    ctx.save();ctx.translate(x,y);ctx.scale(s,s);
    circle(0,0,14,"#153c34");
    for (const [dx,dy] of [[-6,-6],[6,-6],[6,6],[-6,6]]) {
      line([[0,0],[dx,dy]],colors.bird,2.4);circle(dx,dy,4.6,null,colors.bird,1.8);
    }
    circle(0,0,3,colors.bird);ctx.restore();
  }

  function box([x,y]) {
    const s = width < 470 ? 6.6 : 8;
    ctx.save();ctx.translate(x,y);ctx.rotate(Math.PI / 4);
    ctx.fillStyle="#fff3bd";ctx.fillRect(-s/2,-s/2,s,s);ctx.strokeStyle="#69552f";ctx.lineWidth=1;ctx.strokeRect(-s/2,-s/2,s,s);
    ctx.restore();
  }

  function draw(fraction = 0) {
    if (!width || !run) return;
    const f = run.frames[index], next = run.frames[index+1];
    const transition = next && fraction > 0 && !reduced.matches ? next : null;
    ctx.setTransform(ratio,0,0,ratio,0,0);
    terrain();
    if (f.plan.length > 1) line(f.plan.map(pointOf), colors.plan, width < 470 ? 1.6 : 2, [4,5]);
    for (let i=1; i<=index; i++) {
      const past=run.frames[i];
      if (past.kind === "walk" || past.kind === "fly") {
        ctx.globalAlpha = .73;
        line([pointOf(past.from),pointOf(past.to)],colors[past.actor],width < 470 ? 2.0 : 2.65,past.actor === "bird" ? [3,3] : []);
        ctx.globalAlpha=1;
      }
    }
    let dp=pointOf(f.duck), bp=pointOf(f.bird);
    if (transition && ["walk","fly"].includes(transition.kind)) {
      const from=pointOf(transition.from), to=pointOf(transition.to);
      const p = fraction * fraction * (3 - 2*fraction);
      const pos=mix(from,to,p);
      line([from,pos],colors[transition.actor],2.6,transition.actor === "bird" ? [3,3] : []);
      if (transition.actor === "duck") dp=pos; else bp=pos;
    }
    stations();
    drawGate("north",f.known.north,[2.61,1.12],.84);
    drawGate("south",f.known.south,[2.60,3.03],3.31);
    drawGate("cover",f.known.cover,[1.28,2.00],null);
    duckSymbol(dp, f.owner === "duck" ? 1 : -1);
    birdSymbol(bp);
    const pos=f.owner === "duck" ? [dp[0]-10,dp[1]-8] :
      f.owner === "bird" ? [bp[0]-10,bp[1]-8] : pointOf(f.owner);
    box(f.owner === "receiver" ? [pos[0]+9,pos[1]+7] : pos);
  }

  function resize() {
    const rect=canvas.getBoundingClientRect();
    width=rect.width;height=rect.height;ratio=Math.min(window.devicePixelRatio || 1, 2.5);
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
    draw();
  }
  function fieldMode() {return layout.value === "field";}
  function atFrontier() {return index === run.frames.length - 1;}
  function end() {return atFrontier() && !run.pending;}
  function waitingForGate() {return fieldMode() && atFrontier() && run.pending;}
  function realSeconds(seq) {
    if (confirmations[seq] === undefined || confirmations[0] === undefined) return 0;
    return (confirmations[seq] - confirmations[0]) / 1000;
  }
  function refresh() {
    const f=run.frames[index], ownerIndex=chain.indexOf(f.owner);
    const field=fieldMode(), gate=waitingForGate(), started=confirmations.length>0;
    let title=f.title, detail=f.detail;
    if (field && !started) {
      title="现场彩排就绪";
      detail="确认起点、样品编号和记录员后开始计时。每个动作都由人回报。";
    } else if (gate) {
      title="请回报" + model.gates[gate.gate] + "路况";
      detail=(gate.actor === "bird" ? "根据空中观察" : "根据小鸭子近处观察") + "，选择通行或封闭。看不清时保留待观察状态。";
    } else if (field && !end()) {
      title="待确认 · " + run.frames[index+1].title;
      detail="由操作员目视核实这一步后按「确认完成」。可随时导出已记录的步骤。";
    }
    get("event-title").textContent=title;
    get("event-detail").textContent=detail;
    get("mode").textContent=field ? "现场彩排 · 人工记录" : "二维任务仿真";
    get("note").textContent=field ?
      "现场路况由操作员输入。动作按确认时间记录，地面里程按场地图估算；可在 H 垫导出本轮记录。" :
      "地图采用预设走廊。路口由模拟观察逐一确认，棚内只能由小鸭子查看。距离和时间均来自模型。";
    get("clock-label").textContent=field ? "已记录用时" : "模拟用时";
    get("distance-label").textContent=field ? "图上里程" : "地面里程";
    get("version").textContent="MAP " + String(f.mapVersion).padStart(2,"0");
    get("time").textContent=(field ? realSeconds(index) : f.time).toFixed(1);
    get("metres").textContent=f.duckM.toFixed(1);
    get("reads").textContent=f.reads;
    get("owner").textContent=model.stations[f.owner];
    get("counter").textContent=field ? confirmations.length + " 条已确认" : (index+1) + " / " + run.frames.length;
    get("play").hidden=field && (Boolean(gate) || end());
    get("step").hidden=field;
    get("pass").hidden=!gate;
    get("block").hidden=!gate;
    get("play").textContent=field ? (started ? "确认完成" : "开始记录") :
      playing ? "暂停" : end() ? "重新演示" : index || elapsed ? "继续演示" : "开始演示";
    get("step").disabled=end();
    for (const [i,li] of Array.from(get("custody").children).entries()) {
      li.classList.toggle("visited",i<=ownerIndex);li.classList.toggle("active",i===ownerIndex);
      if (i===ownerIndex) li.setAttribute("aria-current","step");else li.removeAttribute("aria-current");
    }
    const items=run.frames.slice(0,field ? confirmations.length : index+1).reverse().map(entry => {
      const li=document.createElement("li"), time=document.createElement("time"), name=document.createElement("span");
      time.textContent=(field ? realSeconds(entry.seq) : entry.time).toFixed(1)+" s";name.textContent=entry.title;li.append(time,name);return li;
    });
    get("log").replaceChildren(...items);
    draw();
  }

  function stopPlayback() {
    playing=false;last=null;
    if (requestId!==null) cancelAnimationFrame(requestId);
    requestId=null;
  }
  function restart() {
    stopPlayback();index=0;elapsed=0;
    reports={};confirmations=[];
    run=model.simulate(layout.value,policy.value,reports);
    refresh();
  }
  function duration(f) {
    if (f.kind === "walk" || f.kind === "fly") return Math.min(1100,300+f.duration*65);
    return f.kind === "observe" || f.kind === "transfer" ? 870 : 620;
  }
  function tick(now) {
    if (!playing) return;
    if (last!==null) elapsed+=Math.min(now-last,160);
    last=now;
    const wait=duration(run.frames[index+1]);
    if (elapsed>=wait) {
      index++;elapsed=0;
      if (end()) stopPlayback();
      refresh();
    } else draw(elapsed/wait);
    if (playing) requestId=requestAnimationFrame(tick);
  }
  function reportGate(open) {
    const pending=waitingForGate();
    if (!pending || !confirmations.length) return;
    reports[pending.gate]=open;
    run=model.simulate("field",policy.value,reports);
    index++;confirmations[index]=Date.now();refresh();
  }
  get("pass").addEventListener("click",() => reportGate(true));
  get("block").addEventListener("click",() => reportGate(false));
  get("play").addEventListener("click",() => {
    if (fieldMode()) {
      if (!confirmations.length) confirmations[0]=Date.now();
      else if (!atFrontier()) {index++;confirmations[index]=Date.now();}
      refresh();return;
    }
    if (playing) {stopPlayback();refresh();return;}
    if (end()) restart();
    playing=true;last=null;refresh();requestId=requestAnimationFrame(tick);
  });
  get("step").addEventListener("click",() => {
    stopPlayback();elapsed=0;if(!end()) index++;refresh();
  });
  get("reset").addEventListener("click",restart);
  layout.addEventListener("change",restart);policy.addEventListener("change",restart);
  get("export").addEventListener("click",() => {
    const field=fieldMode();
    const frames=run.frames.slice(0,field ? confirmations.length : index+1);
    const data={
      ...run, pending:waitingForGate() || null,
      source:field ? "operator-reported rehearsal" : "2D task model",
      frames:field ? frames.map(f => ({...f,confirmedAt:new Date(confirmations[f.seq]).toISOString(),
        actualElapsedSeconds:realSeconds(f.seq)})) : frames
    };
    const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:"application/json"}));
    const link=document.createElement("a");link.href=url;
    link.download="oprace-"+run.layoutId+"-"+run.policy+"-"+index+".json";
    document.body.appendChild(link);link.click();link.remove();setTimeout(() => URL.revokeObjectURL(url),1500);
  });
  document.addEventListener("visibilitychange",() => {if (document.hidden && playing) {stopPlayback();refresh();}});
  restart();resize();new ResizeObserver(resize).observe(canvas);
})();
