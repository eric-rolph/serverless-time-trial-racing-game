//! WebSocket lap submission — CONTRACTS.md §5.
//! One JSON text frame (header), one binary frame (LAPLOG), then read ack /
//! result frames until a result arrives or 30 s elapse.

use std::time::Duration;

use anyhow::{bail, Context, Result};
use base64::engine::general_purpose::STANDARD as B64;
use base64::Engine as _;
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use tokio_tungstenite::tungstenite::Message;

pub const RESULT_TIMEOUT: Duration = Duration::from_secs(30);

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum ServerFrame {
    Ack { stage: String },
    Result {
        status: String,
        #[serde(rename = "lapTimeMs")]
        lap_time_ms: Option<u64>,
        rank: Option<u64>,
        reason: Option<String>,
        detail: Option<String>,
    },
}

#[derive(Debug, Clone)]
pub struct SubmitOutcome {
    pub accepted: bool,
    pub lap_time_ms: Option<u64>,
    pub rank: Option<u64>,
    pub reason: Option<String>,
    pub detail: Option<String>,
}

impl std::fmt::Display for SubmitOutcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.accepted {
            write!(f, "ACCEPTED")?;
            if let Some(ms) = self.lap_time_ms {
                write!(f, " — lap {}.{:03}s", ms / 1000, ms % 1000)?;
            }
            if let Some(rank) = self.rank {
                write!(f, ", rank #{rank}")?;
            }
        } else {
            write!(f, "REJECTED")?;
            if let Some(r) = &self.reason {
                write!(f, " ({r})")?;
            }
            if let Some(d) = &self.detail {
                write!(f, ": {d}")?;
            }
        }
        Ok(())
    }
}

/// Submit a signed LAPLOG; prints ack stages as they stream in.
pub async fn submit(
    url: &str,
    pubkey: &[u8; 32],
    sig: &[u8; 64],
    name: &str,
    laplog_bytes: &[u8],
) -> Result<SubmitOutcome> {
    let (mut ws, _resp) = tokio_tungstenite::connect_async(url)
        .await
        .with_context(|| format!("websocket connect to '{url}' failed"))?;

    let header = serde_json::json!({
        "type": "submit",
        "pubkey": B64.encode(pubkey),
        "sig": B64.encode(sig),
        "name": name,
        "logBytes": laplog_bytes.len(),
    });
    ws.send(Message::Text(header.to_string().into()))
        .await
        .context("sending submit header frame")?;
    ws.send(Message::Binary(laplog_bytes.to_vec().into()))
        .await
        .context("sending LAPLOG binary frame")?;

    let result = tokio::time::timeout(RESULT_TIMEOUT, async {
        loop {
            let msg = match ws.next().await {
                Some(m) => m.context("websocket read error")?,
                None => bail!("server closed connection before sending a result"),
            };
            let text = match msg {
                Message::Text(t) => t.to_string(),
                Message::Close(frame) => {
                    bail!("server closed connection: {frame:?}")
                }
                _ => continue, // ignore pings/binary
            };
            match serde_json::from_str::<ServerFrame>(&text) {
                Ok(ServerFrame::Ack { stage }) => {
                    println!("server: ack [{stage}]");
                }
                Ok(ServerFrame::Result { status, lap_time_ms, rank, reason, detail }) => {
                    return Ok(SubmitOutcome {
                        accepted: status == "accepted",
                        lap_time_ms,
                        rank,
                        reason,
                        detail,
                    });
                }
                Err(_) => {
                    eprintln!("server: unrecognized frame: {text}");
                }
            }
        }
    })
    .await
    .map_err(|_| anyhow::anyhow!("timed out after 30 s waiting for server result"))??;

    let _ = ws.close(None).await;
    Ok(result)
}

/// Blocking wrapper for use from the synchronous game/CLI paths.
pub fn submit_blocking(
    url: &str,
    pubkey: &[u8; 32],
    sig: &[u8; 64],
    name: &str,
    laplog_bytes: &[u8],
) -> Result<SubmitOutcome> {
    let rt = tokio::runtime::Runtime::new().context("starting tokio runtime")?;
    rt.block_on(submit(url, pubkey, sig, name, laplog_bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_server_frames() {
        let f: ServerFrame = serde_json::from_str(r#"{"type":"ack","stage":"verified"}"#).unwrap();
        assert!(matches!(f, ServerFrame::Ack { ref stage } if stage == "verified"));

        let f: ServerFrame = serde_json::from_str(
            r#"{"type":"result","status":"accepted","lapTimeMs":83452,"rank":4}"#,
        )
        .unwrap();
        match f {
            ServerFrame::Result { status, lap_time_ms, rank, .. } => {
                assert_eq!(status, "accepted");
                assert_eq!(lap_time_ms, Some(83452));
                assert_eq!(rank, Some(4));
            }
            _ => panic!("wrong variant"),
        }

        let f: ServerFrame = serde_json::from_str(
            r#"{"type":"result","status":"rejected","reason":"replay_mismatch","detail":"hash"}"#,
        )
        .unwrap();
        match f {
            ServerFrame::Result { status, reason, .. } => {
                assert_eq!(status, "rejected");
                assert_eq!(reason.as_deref(), Some("replay_mismatch"));
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn outcome_display() {
        let o = SubmitOutcome {
            accepted: true,
            lap_time_ms: Some(83452),
            rank: Some(4),
            reason: None,
            detail: None,
        };
        assert_eq!(format!("{o}"), "ACCEPTED — lap 83.452s, rank #4");

        let o = SubmitOutcome {
            accepted: false,
            lap_time_ms: None,
            rank: None,
            reason: Some("bad_signature".into()),
            detail: Some("nope".into()),
        };
        assert_eq!(format!("{o}"), "REJECTED (bad_signature): nope");
    }
}
