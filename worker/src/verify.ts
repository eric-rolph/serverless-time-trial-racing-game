// Stage 1 of the referee: Ed25519 signature verification (docs/CONTRACTS.md §4).
// Workers WebCrypto supports Ed25519 natively.

import { SIGNING_DOMAIN_TAG } from "./protocol";

const encoder = new TextEncoder();

export async function verifySignature(
  pubkey: Uint8Array, // 32 bytes
  sig: Uint8Array, // 64 bytes
  logBytes: Uint8Array,
): Promise<boolean> {
  let key: CryptoKey;
  try {
    key = await crypto.subtle.importKey("raw", pubkey, { name: "Ed25519" }, false, ["verify"]);
  } catch {
    return false;
  }
  const tag = encoder.encode(SIGNING_DOMAIN_TAG);
  const message = new Uint8Array(tag.length + logBytes.length);
  message.set(tag, 0);
  message.set(logBytes, tag.length);
  return crypto.subtle.verify({ name: "Ed25519" }, key, sig, message);
}
