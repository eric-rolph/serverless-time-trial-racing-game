//! Ed25519 lap signing — CONTRACTS.md §4.
//! Identity = public key. Keypair generated on first launch, persisted as raw
//! 32 seed bytes in the OS config dir, never transmitted.

use std::fs;
use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};

/// ASCII domain separation tag prepended to the LAPLOG bytes before signing.
pub const DOMAIN_TAG: &[u8] = b"sttr-lap-v1";

pub const KEY_FILE: &str = "key.bin";

pub struct Identity {
    key: SigningKey,
}

impl Identity {
    /// Load the persisted key, or generate + persist one on first launch.
    pub fn load_or_generate() -> Result<Identity> {
        let dir = config_dir()?;
        let path = dir.join(KEY_FILE);
        if path.exists() {
            let bytes = fs::read(&path)
                .with_context(|| format!("reading signing key '{}'", path.display()))?;
            let seed: [u8; 32] = bytes
                .as_slice()
                .try_into()
                .map_err(|_| anyhow::anyhow!("'{}' is not a 32-byte key file", path.display()))?;
            return Ok(Identity { key: SigningKey::from_bytes(&seed) });
        }
        // First launch: generate and write once.
        let mut seed = [0u8; 32];
        getrandom::fill(&mut seed).map_err(|e| anyhow::anyhow!("OS RNG failure: {e}"))?;
        fs::create_dir_all(&dir)
            .with_context(|| format!("creating config dir '{}'", dir.display()))?;
        fs::write(&path, seed)
            .with_context(|| format!("writing signing key '{}'", path.display()))?;
        eprintln!("generated new identity key at {}", path.display());
        Ok(Identity { key: SigningKey::from_bytes(&seed) })
    }

    /// Deterministic construction for tests.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn from_seed(seed: [u8; 32]) -> Identity {
        Identity { key: SigningKey::from_bytes(&seed) }
    }

    /// Sign `"sttr-lap-v1" ‖ laplog_bytes` (the exact bytes as transmitted).
    pub fn sign_laplog(&self, laplog_bytes: &[u8]) -> [u8; 64] {
        let mut msg = Vec::with_capacity(DOMAIN_TAG.len() + laplog_bytes.len());
        msg.extend_from_slice(DOMAIN_TAG);
        msg.extend_from_slice(laplog_bytes);
        self.key.sign(&msg).to_bytes()
    }

    pub fn public_key_bytes(&self) -> [u8; 32] {
        self.key.verifying_key().to_bytes()
    }

    /// Verify a signature produced by [`sign_laplog`] (used by tests; the
    /// real verifier is the Worker via WebCrypto).
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn verify_laplog(
        pubkey: &[u8; 32],
        laplog_bytes: &[u8],
        sig: &[u8; 64],
    ) -> Result<()> {
        let vk = VerifyingKey::from_bytes(pubkey).context("invalid public key")?;
        let mut msg = Vec::with_capacity(DOMAIN_TAG.len() + laplog_bytes.len());
        msg.extend_from_slice(DOMAIN_TAG);
        msg.extend_from_slice(laplog_bytes);
        let sig = Signature::from_bytes(sig);
        if vk.verify(&msg, &sig).is_err() {
            bail!("signature verification failed");
        }
        Ok(())
    }
}

/// App config directory (shared: signing key here, ffb.toml in ffb.rs).
pub(crate) fn config_dir() -> Result<PathBuf> {
    let dirs = directories::ProjectDirs::from("", "", "sttr-client")
        .context("could not determine OS config directory")?;
    Ok(dirs.config_dir().to_path_buf())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_verify_roundtrip() {
        let id = Identity::from_seed([7u8; 32]);
        let log = b"STLG-fake-lap-bytes-for-test";
        let sig = id.sign_laplog(log);
        let pk = id.public_key_bytes();
        Identity::verify_laplog(&pk, log, &sig).expect("roundtrip must verify");

        // tampered log fails
        let mut bad = log.to_vec();
        bad[0] ^= 0xFF;
        assert!(Identity::verify_laplog(&pk, &bad, &sig).is_err());

        // signature over a different domain tag must not verify: re-sign raw
        let raw_sig = id.key.sign(log).to_bytes();
        assert!(
            Identity::verify_laplog(&pk, log, &raw_sig).is_err(),
            "domain tag must be part of the signed message"
        );
    }

    #[test]
    fn key_and_signature_sizes_match_contract() {
        let id = Identity::from_seed([42u8; 32]);
        assert_eq!(id.public_key_bytes().len(), 32);
        let sig = id.sign_laplog(b"x");
        assert_eq!(sig.len(), 64);
    }

    #[test]
    fn identity_is_deterministic_from_seed() {
        let a = Identity::from_seed([1u8; 32]);
        let b = Identity::from_seed([1u8; 32]);
        assert_eq!(a.public_key_bytes(), b.public_key_bytes());
        assert_eq!(a.sign_laplog(b"lap"), b.sign_laplog(b"lap"));
    }
}
