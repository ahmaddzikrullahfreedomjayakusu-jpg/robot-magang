// ========================================
// content_script.js — FINAL SYNC (MV3)
// ========================================

// Cek ketersediaan Chrome API di content script
function canUseRuntime() {
  return (
    typeof chrome !== "undefined" && !!(chrome.runtime && chrome.runtime.id)
  );
}

// ==============================
// 0) Mini toast (brutalism)
// ==============================
function showToast(msg) {
  try {
    const id = "satulink-toast";
    let el = document.getElementById(id);
    if (!el) {
      el = document.createElement("div");
      el.id = id;
      el.setAttribute("role", "status");
      el.style.cssText = [
        "position:fixed; left:50%; top:18px; transform:translateX(-50%); z-index:2147483647;",
        "background:#fff; color:#111; border:4px solid #000; border-radius:14px;",
        "padding:10px 14px; font:900 14px/1.2 system-ui, -apple-system, Segoe UI, Roboto, Arial;",
        "box-shadow:6px 6px 0 #000; text-align:center; white-space:pre-wrap; max-width:92vw;",
        "opacity:1; transition:opacity .25s ease;",
      ].join("");
      document.documentElement.appendChild(el);
    }
    el.textContent = msg;
    el.style.opacity = "1";
    clearTimeout(el._t);
    el._t = setTimeout(() => {
      el.style.opacity = "0";
    }, 2200);
  } catch (_) {}
}

// ==============================
// 1) Normalisasi payload
// ==============================
function mapSameSite(v) {
  if (!v) return undefined;
  const s = String(v).toLowerCase();
  if (s === "lax") return "Lax";
  if (s === "strict") return "Strict";
  if (s === "no_restriction" || s === "none") return "None";
  return undefined;
}

function sanitizeCookieItem(it) {
  // Terima field umum saja; background akan melengkapi 'url'
  if (!it || typeof it !== "object") return null;
  if (typeof it.name !== "string" || typeof it.value !== "string") return null;

  const out = {
    name: it.name,
    value: it.value,
  };

  if (typeof it.domain === "string" && it.domain) out.domain = it.domain;
  if (typeof it.path === "string" && it.path) out.path = it.path;

  if (typeof it.httpOnly === "boolean") out.httpOnly = it.httpOnly;
  if (typeof it.secure === "boolean") out.secure = it.secure;

  const ss = mapSameSite(it.sameSite);
  if (ss) out.sameSite = ss;

  // beberapa dump pakai 'expires' (epoch detik) atau 'expirationDate'
  if (typeof it.expires === "number") out.expirationDate = it.expires;
  if (typeof it.expirationDate === "number")
    out.expirationDate = it.expirationDate;

  if (typeof it.hostOnly === "boolean") out.hostOnly = it.hostOnly;
  if (typeof it.session === "boolean") out.session = it.session;

  // jika ada url di item
  if (typeof it.url === "string" && it.url) out.url = it.url;

  return out;
}

function normalizeIncomingPayload(raw) {
  // Return { url, list } — list = array cookies sudah disanitasi
  let data = raw;

  if (typeof data === "string") {
    try {
      data = JSON.parse(data);
    } catch (_) {
      return { url: null, list: [] };
    }
  }

  // Bentuk favorit: { url: "...", cookies: [...] }
  if (data && typeof data === "object" && Array.isArray(data.cookies)) {
    return {
      url: typeof data.url === "string" ? data.url : null,
      list: data.cookies.map(sanitizeCookieItem).filter(Boolean),
    };
  }

  // Bentuk lain: langsung array cookie (tanpa url)
  if (Array.isArray(data)) {
    return { url: null, list: data.map(sanitizeCookieItem).filter(Boolean) };
  }

  // Tidak valid
  return { url: null, list: [] };
}

function guessUrlFromCookies(list, fallbackOpenUrl) {
  // Urutan prioritas: openUrl (jika ada) → domain pertama cookie → location.origin
  if (typeof fallbackOpenUrl === "string" && fallbackOpenUrl)
    return fallbackOpenUrl;

  const c = list.find((x) => typeof x?.domain === "string" && x.domain);
  if (c) {
    const domain = c.domain.replace(/^\./, "");
    const secure = c.secure !== false; // default aman: https
    const path = typeof c.path === "string" && c.path ? c.path : "/";
    return (secure ? "https://" : "http://") + domain + path;
  }

  try {
    return location.origin + "/";
  } catch (_) {
    return "https://";
  }
}

function batchArray(arr, size) {
  const out = [];
  for (let i = 0; i < arr.length; i += size) out.push(arr.slice(i, i + size));
  return out;
}

// ==============================
// 2) Terima cookies dari halaman (inject.php / console)
// ==============================
window.addEventListener("message", function onInjectMsg(event) {
  if (event.source !== window || !event.data) return;
  if (event.data.type !== "inject_cookies_raw") return;
  if (!canUseRuntime()) {
    console.warn("Satulink: chrome.runtime tidak tersedia di halaman ini.");
    return;
  }

  // Normalisasi payload
  const { url: payloadUrl, list } = normalizeIncomingPayload(
    event.data.payload,
  );
  if (!list.length) {
    console.warn(
      "Satulink: payload data kosong / tidak valid:",
      event.data.payload,
    );
    return;
  }

  // Tentukan target URL untuk set cookie & (opsional) buka tab
  const openUrlFromEvent =
    typeof event.data.openUrl === "string" ? event.data.openUrl : null;
  const targetUrl = payloadUrl || guessUrlFromCookies(list, openUrlFromEvent);

  // Opsi auto open tab
  const wantOpen = event.data.openAfter === true; // default: tidak buka tab
  const mode = event.data.mode || "newtab"; // 'newtab' | 'reload'
  const focus = event.data.focus !== false; // default: fokus tab

  console.log("📩 Satulink: menerima", list.length, "data →", targetUrl);

  // Kirim dalam batch agar aman bila list sangat panjang
  const BATCH_SIZE = 150;
  const batches = batchArray(list, BATCH_SIZE);

  (async function sendBatches() {
    for (let i = 0; i < batches.length; i++) {
      const part = batches[i];

      await new Promise((resolve) => {
        chrome.runtime.sendMessage(
          {
            type: "storeCookies",
            // STRUKTUR HARUS { url, cookies: [...] } SESUAI background.js
            cookies: { url: targetUrl, cookies: part },

            // Buka tab hanya SEKALI di batch terakhir
            openAfter: wantOpen && i === batches.length - 1,
            openUrl: targetUrl,
            mode,
            focus,
          },
          (resp) => {
            if (chrome.runtime.lastError) {
              console.error(
                "❌ Satulink: kirim batch gagal:",
                chrome.runtime.lastError,
              );
            } else {
              console.log(
                `✅ Satulink: batch ${i + 1}/${batches.length} diterima:`,
                resp,
              );
            }
            resolve();
          },
        );
      });
    }

    showToast("✅ Data terkirim ke ekstensi");
    try {
      window.postMessage({ type: "inject_result", ok: true }, "*");
    } catch (_) {}
  })();
});

// ==============================
// 3) Reset token / Logout sinkron
// ==============================
window.addEventListener("message", function onResetLogout(event) {
  if (event.source !== window || !event.data) return;
  if (!canUseRuntime()) return;

  if (event.data.type === "reset_token_extension") {
    console.log("🔄 Satulink: reset_token_extension diterima");
    chrome.storage.local.clear(() => {
      console.log("🗑️ Satulink: local storage ekstensi dibersihkan");
    });
    chrome.runtime.sendMessage({ type: "clearAll" }, (res) => {
      console.log("✅ Satulink: clearAll data diproses:", res);
      chrome.runtime.sendMessage({ type: "resetToken" }, () => {});
      showToast("🔄 Token direset");
    });
  }

  if (event.data.type === "logout_extension") {
    console.log("🔒 Satulink: logout_extension diterima");
    chrome.storage.local.clear(() => {
      console.log("🗑️ Satulink: local storage dibersihkan (logout)");
    });
    chrome.runtime.sendMessage({ type: "clearAll" }, (res) => {
      console.log("✅ Satulink: clearAll data (logout):", res);
      chrome.runtime.sendMessage({ type: "autoLogout" }, () => {});
      showToast("🔒 Logout berhasil");
    });
  }
});

// ==============================
// 4) Listener dari background (debug/info)
// ==============================
if (canUseRuntime()) {
  chrome.runtime.onMessage.addListener((msg) => {
    try {
      if (msg && typeof msg === "object") {
        if (msg.type === "cookiesCleared") {
          console.log("🔔 Satulink: Semua data telah dihapus");
          showToast("🧹 Data dibersihkan");
        } else if (msg.type === "autoLogout") {
          console.log("🔔 Satulink: Auto logout terjadi");
          showToast("🔒 Auto logout");
        } else if (msg.type === "resetToken") {
          console.log("🔔 Satulink: Reset token");
          showToast("🔄 Token direset");
        } else if (msg.type === "cookiesAppliedForDomain" && msg.domain) {
          console.log(`🍪 Satulink: Data terpasang untuk ${msg.domain}`);
        } else if (msg.type === "cookiesSaved") {
          // opsional: ringkasan
          const total = msg.summary?.total ?? 0;
          showToast(`✅ ${total} data disimpan`);
        }
      }
    } catch (e) {
      console.warn("Satulink: gagal memproses pesan background:", e);
    }
  });
}
