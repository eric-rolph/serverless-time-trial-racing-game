// Stage 5 of the referee: global leaderboard in Cloudflare KV (CONTRACTS §7).
// Zero-padded tick counts in keys make KV's lexicographic list() return
// ascending lap times without any secondary index.

export interface LeaderboardEntry {
  pubkey: string; // hex, 64 chars
  name: string;
  ticks: number;
  ms: number;
  submittedAt: string;
  flags: string[];
}

const pad8 = (ticks: number) => ticks.toString(10).padStart(8, "0");

export async function recordLap(
  kv: KVNamespace,
  trackHashHex: string,
  entry: LeaderboardEntry,
): Promise<{ improved: boolean; rank: number }> {
  const bestKey = `best:${trackHashHex}:${entry.pubkey}`;
  const prevBest = await kv.get(bestKey);
  const prevTicks = prevBest === null ? Infinity : parseInt(prevBest, 10);

  if (entry.ticks < prevTicks) {
    const lbKey = `lb:${trackHashHex}:${pad8(entry.ticks)}:${entry.pubkey.slice(0, 16)}`;
    await kv.put(lbKey, JSON.stringify(entry));
    await kv.put(bestKey, String(entry.ticks));
    if (Number.isFinite(prevTicks)) {
      await kv.delete(`lb:${trackHashHex}:${pad8(prevTicks)}:${entry.pubkey.slice(0, 16)}`);
    }
    return { improved: true, rank: await rankOf(kv, trackHashHex, entry.ticks) };
  }
  return { improved: false, rank: await rankOf(kv, trackHashHex, prevTicks) };
}

async function rankOf(kv: KVNamespace, trackHashHex: string, ticks: number): Promise<number> {
  const list = await kv.list({ prefix: `lb:${trackHashHex}:`, limit: 1000 });
  let rank = 1;
  for (const key of list.keys) {
    const keyTicks = parseInt(key.name.split(":")[2], 10);
    if (keyTicks < ticks) rank++;
  }
  return rank;
}

export async function topEntries(
  kv: KVNamespace,
  trackHashHex: string,
  limit = 100,
): Promise<LeaderboardEntry[]> {
  const list = await kv.list({ prefix: `lb:${trackHashHex}:`, limit });
  const entries = await Promise.all(list.keys.map((k) => kv.get(k.name, "json")));
  return entries.filter((e): e is LeaderboardEntry => e !== null);
}
