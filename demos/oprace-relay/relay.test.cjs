"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const {simulate, route, points, edges, layouts, policies, limits} = require("./relay.js");

for (const [layoutId, layout] of Object.entries(layouts)) {
  for (const policy of Object.keys(policies)) {
    test(layoutId + " / " + policy, () => {
      const {frames, sampleId} = simulate(layoutId, policy);
      const final = frames.at(-1);
      const feasible = (layout.north || layout.south) && layout.cover;
      assert.equal(final.status, feasible ? "delivered" : "blocked");
      assert.deepEqual(frames[0].known, {north: null, south: null, cover: null});
      assert.ok(final.scoutM <= limits.scoutMetres && final.reads <= limits.scoutReads);
      if (layoutId === "closed" && policy === "B1") assert.equal(final.duckM, 0);
      else assert.ok(final.duckM > 0);
      assert.equal(frames.some(f => f.actor === "bird" && f.gate === "cover"), false);
      if (policy === "B0") assert.equal(final.scoutM + final.reads, 0);
      if (policy === "B1") assert.equal(final.reads, 2);
      if (policy === "B2") assert.equal(frames.filter(f => f.kind === "request").length, final.reads);

      const transfers = frames.filter(f => f.custody);
      assert.deepEqual(transfers.map(f => f.owner), feasible ? ["duck", "pad", "bird", "receiver"] : []);
      if (feasible) {
        assert.equal(transfers[0].duck, "sample");
        assert.equal(transfers[1].duck, "pad");
        assert.equal(transfers[2].bird, "pad");
        assert.equal(transfers[3].bird, "receiver");
        assert.ok(transfers[2].landed && transfers[2].duck === "clear");
        assert.ok(transfers[3].landed);
        assert.ok(transfers.every(f => f.custody.id === sampleId));
        assert.equal(final.toPadM, transfers[1].duckM);
        assert.equal(final.toPadS, transfers[1].time);
      } else {
        assert.equal(final.owner, "sample");
        assert.equal(final.toPadM, null);
        assert.equal(final.toPadS, null);
        assert.equal(final.transportM, 0);
      }
      for (let i = 1; i < frames.length; i++) {
        const prev = frames[i - 1], curr = frames[i];
        assert.ok(curr.time >= prev.time);
        const changed = Object.keys(curr.known).filter(k => curr.known[k] !== prev.known[k]);
        if (curr.kind === "observe") {
          assert.deepEqual(changed, [curr.gate]);
          assert.equal(curr.value, layout[curr.gate]);
          assert.equal(curr.mapVersion, prev.mapVersion + 1);
        } else {
          assert.deepEqual(changed, []);
        }
        if (curr.kind === "walk") {
          assert.equal(prev.bird, curr.bird);
          assert.ok(prev.landed && curr.landed);
          const edge = edges.find(e => [e.a, e.b].includes(prev.duck) && [e.a, e.b].includes(curr.duck));
          assert.ok(edge && (edge.a !== edge.b));
          if (edge.gate) assert.equal(prev.known[edge.gate], true);
        }
        if (curr.kind === "fly") {
          assert.equal(prev.duck, curr.duck);
          assert.ok(!["sample", "door"].includes(curr.bird));
        }
        if (curr.owner !== prev.owner) assert.deepEqual([curr.custody.from, curr.custody.to], [prev.owner, curr.owner]);
      }
    });
  }
}

test("a changed gate changes the route before the duck enters it", () => {
  const run = simulate("south", "B2").frames;
  const closure = run.find(f => f.kind === "observe" && f.gate === "north");
  assert.ok(run[closure.seq - 1].plan.includes("northEast"));
  assert.ok(closure.plan.includes("southEast") && !closure.plan.includes("northEast"));
  assert.ok(!run.some(f => f.duck === "northEast"));
  assert.ok(simulate("south", "B0").frames.at(-1).toPadM > run.at(-1).toPadM);
});

test("a ground observer cannot learn the hidden layout before the first observation", () => {
  for (const policy of Object.keys(policies)) {
    const a = simulate("south", policy).frames, b = simulate("north", policy).frames;
    const first = a.findIndex(f => f.kind === "observe");
    assert.deepEqual(a.slice(0, first), b.slice(0, first));
  }
});

test("only an observation of the covered edge distinguishes a closed interior", () => {
  for (const policy of Object.keys(policies)) {
    const a = simulate("both", policy).frames, b = simulate("cover", policy).frames;
    const index = a.findIndex(f => f.gate === "cover");
    assert.deepEqual(a.slice(0, index), b.slice(0, index));
  }
});

test("receipt is unreachable via the ground graph even if every gate opens", () => {
  assert.ok(points.receiver);
  assert.deepEqual(route("pad", "receiver", {north: true, south: true, cover: true}), []);
});

test("fixed and on-demand surveys incur measurable actual travel within the common limit", () => {
  const fixed = simulate("north", "B1").frames.at(-1);
  const cheap = simulate("north", "B2").frames.at(-1);
  const costly = simulate("south", "B2").frames.at(-1);
  assert.ok(cheap.reads < fixed.reads && cheap.scoutM < fixed.scoutM);
  assert.ok(costly.scoutM > fixed.scoutM);
});

test("a replay is deterministic, and rejects unknown course and policy names", () => {
  assert.deepEqual(simulate(), simulate());
  assert.throws(() => simulate("other", "B0"));
  assert.throws(() => simulate("south", "__proto__"));
});

test("field mode waits at the observing position and preserves the confirmed prefix", () => {
  for (const policy of Object.keys(policies)) {
    for (const layout of Object.values(layouts)) {
      const reports = {};
      let rehearsal = simulate("field", policy, reports);
      const first = rehearsal.frames.at(-1);
      assert.ok(rehearsal.pending && first.status === "running" && first.mapVersion === 0);
      while (rehearsal.pending) {
        const {gate, actor} = rehearsal.pending;
        assert.equal(Object.hasOwn(reports, gate), false);
        if (gate === "cover") assert.equal(actor, "duck");
        const prefix = rehearsal.frames;
        reports[gate] = layout[gate];
        rehearsal = simulate("field", policy, reports);
        assert.deepEqual(rehearsal.frames.slice(0, prefix.length), prefix);
        const observation = rehearsal.frames[prefix.length];
        assert.equal(observation.kind, "observe");
        assert.equal(observation.gate, gate);
        assert.equal(observation.value, layout[gate]);
      }
      assert.equal(rehearsal.frames.at(-1).status, (layout.north || layout.south) && layout.cover ? "delivered" : "blocked");
    }
  }
  assert.throws(() => simulate("field", "B1", {north: "false"}));
});
