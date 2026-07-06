import { describe, expect, it } from "vitest";
import { recordLap, topEntries, type LeaderboardEntry } from "../src/leaderboard";

/** Minimal in-memory KVNamespace: lexicographically sorted list(), like real KV. */
function mockKV(): KVNamespace {
  const store = new Map<string, string>();
  return {
    get: async (key: string, type?: string) => {
      const v = store.get(key) ?? null;
      return type === "json" && v !== null ? JSON.parse(v) : v;
    },
    put: async (key: string, value: string) => void store.set(key, value),
    delete: async (key: string) => void store.delete(key),
    list: async ({ prefix = "", limit = 1000 }: { prefix?: string; limit?: number }) => ({
      keys: [...store.keys()]
        .filter((k) => k.startsWith(prefix))
        .sort()
        .slice(0, limit)
        .map((name) => ({ name })),
      list_complete: true,
      cacheStatus: null,
    }),
  } as unknown as KVNamespace;
}

const entry = (pubkey: string, ticks: number): LeaderboardEntry => ({
  pubkey,
  name: pubkey.slice(0, 4),
  ticks,
  ms: Math.round((ticks * 1000) / 400),
  submittedAt: "2026-07-05T00:00:00Z",
  flags: [],
});

const TRACK = "3fbe91d5a2ca5851";

describe("leaderboard", () => {
  it("orders entries by lap time via key sorting", async () => {
    const kv = mockKV();
    await recordLap(kv, TRACK, entry("aa".repeat(32), 34000));
    await recordLap(kv, TRACK, entry("bb".repeat(32), 32000));
    await recordLap(kv, TRACK, entry("cc".repeat(32), 33000));
    const top = await topEntries(kv, TRACK);
    expect(top.map((e) => e.ticks)).toEqual([32000, 33000, 34000]);
  });

  it("keeps only a player's personal best", async () => {
    const kv = mockKV();
    const player = "aa".repeat(32);
    await recordLap(kv, TRACK, entry(player, 34000));
    const improved = await recordLap(kv, TRACK, entry(player, 33000));
    expect(improved.improved).toBe(true);
    const worse = await recordLap(kv, TRACK, entry(player, 35000));
    expect(worse.improved).toBe(false);
    const top = await topEntries(kv, TRACK);
    expect(top).toHaveLength(1);
    expect(top[0].ticks).toBe(33000);
  });

  it("computes rank among existing entries", async () => {
    const kv = mockKV();
    await recordLap(kv, TRACK, entry("aa".repeat(32), 30000));
    await recordLap(kv, TRACK, entry("bb".repeat(32), 31000));
    const third = await recordLap(kv, TRACK, entry("cc".repeat(32), 32000));
    expect(third.rank).toBe(3);
    const first = await recordLap(kv, TRACK, entry("dd".repeat(32), 29000));
    expect(first.rank).toBe(1);
  });
});
