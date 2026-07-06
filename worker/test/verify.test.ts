// Uses Node 22's WebCrypto Ed25519 (same algorithm family the Workers runtime
// provides) to prove the domain-tag framing and verification logic.
import { describe, expect, it } from "vitest";
import { SIGNING_DOMAIN_TAG } from "../src/protocol";
import { verifySignature } from "../src/verify";

async function makeKeyAndSign(message: Uint8Array) {
  const kp = (await crypto.subtle.generateKey({ name: "Ed25519" }, true, [
    "sign",
    "verify",
  ])) as CryptoKeyPair;
  const pubkey = new Uint8Array((await crypto.subtle.exportKey("raw", kp.publicKey)) as ArrayBuffer);
  const tag = new TextEncoder().encode(SIGNING_DOMAIN_TAG);
  const tagged = new Uint8Array(tag.length + message.length);
  tagged.set(tag, 0);
  tagged.set(message, tag.length);
  const sig = new Uint8Array(await crypto.subtle.sign({ name: "Ed25519" }, kp.privateKey, tagged));
  return { pubkey, sig };
}

describe("verifySignature", () => {
  const log = new TextEncoder().encode("pretend this is a LAPLOG");

  it("accepts a valid signature over tag||log", async () => {
    const { pubkey, sig } = await makeKeyAndSign(log);
    expect(pubkey.length).toBe(32);
    expect(sig.length).toBe(64);
    expect(await verifySignature(pubkey, sig, log)).toBe(true);
  });

  it("rejects a tampered log", async () => {
    const { pubkey, sig } = await makeKeyAndSign(log);
    const tampered = log.slice();
    tampered[0] ^= 0xff;
    expect(await verifySignature(pubkey, sig, tampered)).toBe(false);
  });

  it("rejects a wrong key", async () => {
    const { sig } = await makeKeyAndSign(log);
    const other = await makeKeyAndSign(log);
    expect(await verifySignature(other.pubkey, sig, log)).toBe(false);
  });

  it("rejects garbage pubkey bytes without throwing", async () => {
    const { sig } = await makeKeyAndSign(log);
    expect(await verifySignature(new Uint8Array(32).fill(7), sig, log)).toBe(false);
  });
});
