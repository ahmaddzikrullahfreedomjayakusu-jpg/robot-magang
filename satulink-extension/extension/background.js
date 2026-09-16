// ========================
// background.js — FINAL (MV3)
// ========================

/** Kirim notifikasi ke semua context ekstensi (popup / content_script) */
function notifyAll(message) {
  try {
    chrome.runtime.sendMessage(message, () => {
      // Jika tak ada listener (popup tertutup), abaikan error.
      void chrome.runtime.lastError;
    });
  } catch (_) {}
}

/** Map nilai sameSite ke format Chrome */
function toChromeSameSite(v) {
  if (typeof v === "string") {
    const s = v.trim().toLowerCase();
    if (s === "none" || s === "no_restriction") return "no_restriction";
    if (s === "lax") return "lax";
    if (s === "strict") return "strict";
  } else if (typeof v === "number") {
    // 0(None), 1(Lax), 2(Strict)
    if (v === 0) return "no_restriction";
    if (v === 1) return "lax";
    if (v === 2) return "strict";
  }
  return "lax";
}

/** Build URL dari cookie jika belum ada */
function buildUrlFromCookie(c) {
  if (c && typeof c.url === "string" && c.url) return c.url;
  const scheme =
    typeof c.secure === "boolean"
      ? c.secure
        ? "https://"
        : "http://"
      : "https://";
  const host = (c && typeof c.domain === "string" ? c.domain : "").replace(
    /^\./,
    "",
  );
  const path = c && typeof c.path === "string" && c.path ? c.path : "/";
  if (!host) return "";
  return scheme + host + path;
}

/** Set satu cookie (patuh prefix __Host-/__Secure-, SameSite, session/expiry) */
function setOneCookie(raw, done) {
  try {
    const url = buildUrlFromCookie(raw);
    if (!url) return done(new Error("URL target kosong"));

    const isHostPrefix =
      typeof raw.name === "string" && raw.name.indexOf("__Host-") === 0;
    const isSecurePrefix =
      typeof raw.name === "string" && raw.name.indexOf("__Secure-") === 0;

    let sameSite = toChromeSameSite(raw.sameSite);
    const props = {
      url,
      name: String(raw.name || ""),
      value: String(raw.value || ""),
      path: raw.path || "/",
      secure: typeof raw.secure === "boolean" ? raw.secure : true,
      httpOnly: !!raw.httpOnly,
      sameSite, // "no_restriction" | "lax" | "strict"
    };

    // SameSite=None harus secure
    if (sameSite === "no_restriction") props.secure = true;

    // Expiration: jika session==true, jangan set expirationDate
    if (!raw.session) {
      const exp =
        (typeof raw.expirationDate === "number" && raw.expirationDate) ||
        (typeof raw.expires === "number" && raw.expires) ||
        Math.floor(Date.now() / 1000) + 60 * 60 * 24 * 7; // default 7 hari
      props.expirationDate = exp;
    }

    // Domain rules
    if (isHostPrefix) {
      // __Host-: wajib secure, path '/', TANPA domain
      props.secure = true;
      props.path = "/";
      // jangan set props.domain
    } else if (raw.domain) {
      props.domain = raw.domain;
    }

    // __Secure-: wajib secure
    if (isSecurePrefix) props.secure = true;

    chrome.cookies.set(props, () => {
      const err = chrome.runtime.lastError;
      if (err) return done(new Error(err.message || "cookies.set error"));
      done(null, props);
    });
  } catch (e) {
    done(e);
  }
}

/** Set array cookies dan kembalikan ringkasan per-domain */
function setCookiesBatch(list, cb) {
  if (!Array.isArray(list) || list.length === 0)
    return cb(null, { total: 0, byDomain: {} });

  let done = 0;
  const total = list.length;
  const byDomain = {};

  list.forEach((ck) => {
    setOneCookie(ck, (err, props) => {
      try {
        const url = props ? new URL(props.url) : null;
        const d = url ? url.hostname : (ck.domain || "").replace(/^\./, "");
        if (d) {
          byDomain[d] = byDomain[d] || { ok: 0, fail: 0 };
          if (err) byDomain[d].fail++;
          else byDomain[d].ok++;
        }
      } catch (_) {}

      done++;
      if (done === total) cb(null, { total, byDomain });
    });
  });
}

/** Hapus semua cookies & storage ekstensi */
function clearAllCookiesAndStorage(cb) {
  chrome.storage.local.clear(() => {});

  chrome.cookies.getAll({}, (cookies) => {
    if (!cookies || !cookies.length) {
      if (typeof cb === "function") cb();
      return;
    }
    let removed = 0;
    const total = cookies.length;

    cookies.forEach((c) => {
      const url =
        (c.secure ? "https://" : "http://") +
        (c.domain || "").replace(/^\./, "") +
        (c.path || "/");

      chrome.cookies.remove({ url, name: c.name }, () => {
        void chrome.runtime.lastError; // abaikan error per item
        removed++;
        if (removed === total && typeof cb === "function") cb();
      });
    });
  });
}

/** Helper buka / reload tab setelah pemasangan cookies */
function originFromUrl(u) {
  try {
    return new URL(u).origin;
  } catch {
    return null;
  }
}
function patternFromUrl(u) {
  const o = originFromUrl(u);
  return o ? o + "/*" : null;
}
function openAfterAction(openUrl, mode = "newtab", focus = true, cb) {
  try {
    const pattern = patternFromUrl(openUrl);
    if (!pattern) {
      // fallback: langsung buka
      chrome.tabs.create({ url: openUrl, active: !!focus }, () => {
        void chrome.runtime.lastError;
        if (typeof cb === "function") cb();
      });
      return;
    }

    chrome.tabs.query({ url: pattern }, (tabs) => {
      void chrome.runtime.lastError;
      const first = tabs && tabs[0];

      if (mode === "reload") {
        if (first) {
          chrome.tabs.update(first.id, { active: !!focus }, () => {
            void chrome.runtime.lastError;
            chrome.tabs.reload(first.id, {}, () => {
              void chrome.runtime.lastError;
              if (typeof cb === "function") cb();
            });
          });
        } else {
          // tidak ada tab → buka baru
          chrome.tabs.create({ url: openUrl, active: !!focus }, () => {
            void chrome.runtime.lastError;
            if (typeof cb === "function") cb();
          });
        }
      } else {
        // newtab
        chrome.tabs.create({ url: openUrl, active: !!focus }, () => {
          void chrome.runtime.lastError;
          if (typeof cb === "function") cb();
        });
      }
    });
  } catch {
    // fallback aman
    chrome.tabs.create({ url: openUrl, active: !!focus }, () => {
      void chrome.runtime.lastError;
      if (typeof cb === "function") cb();
    });
  }
}

// ========================================
// MESSAGE HANDLER
// ========================================
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
  // 1) Simpan cookies
  if (request && request.type === "storeCookies") {
    // Kompat: bisa berupa { cookies:[...] } atau { cookies: { url, cookies:[...] } }
    let list = [];
    let openUrl = null;

    if (Array.isArray(request.cookies)) {
      list = request.cookies;
    } else if (request.cookies && Array.isArray(request.cookies.cookies)) {
      list = request.cookies.cookies;
      if (request.cookies.url) openUrl = request.cookies.url;
    } else {
      sendResponse({ status: "error", message: "Payload data invalid" });
      return true;
    }

    // Jika openUrl tidak dikirim, coba tebak dari cookie pertama
    if (!openUrl && list.length) {
      openUrl = buildUrlFromCookie(list[0]) || null;
    }

    setCookiesBatch(list, (_err, summary) => {
      // kirim ringkasan global
      notifyAll({ type: "cookiesSaved", summary });
      // kirim ringkasan per-domain
      if (summary && summary.byDomain) {
        Object.keys(summary.byDomain).forEach((dom) => {
          notifyAll({
            type: "cookiesAppliedForDomain",
            domain: dom,
            result: summary.byDomain[dom],
          });
        });
      }

      // Auto open tab (hanya jika diminta)
      if (request.openAfter && openUrl) {
        const mode = request.mode || "newtab"; // 'newtab' | 'reload'
        const focus = request.focus !== false; // default true
        openAfterAction(openUrl, mode, focus, () => {
          sendResponse({ status: "ok", summary, opened: true, mode, openUrl });
        });
      } else {
        sendResponse({ status: "ok", summary, opened: false });
      }
    });

    return true; // async
  }

  // 2) Hapus semua cookies & storage
  if (request && request.type === "clearAll") {
    clearAllCookiesAndStorage(() => {
      notifyAll({ type: "cookiesCleared" });
      sendResponse({ status: "cleared" });
    });
    return true; // async
  }

  // 3) Forward-only notifikasi (biar content_script bisa log)
  if (
    request &&
    (request.type === "resetToken" || request.type === "autoLogout")
  ) {
    notifyAll(request);
    sendResponse({ ok: true });
    return true;
  }

  return true;
});

// ========================================
// ALARMS: Auto logout tiap 6 jam
// ========================================
function ensureAlarm() {
  chrome.alarms.create("autoLogout", { periodInMinutes: 360 }); // 6 jam
}
chrome.runtime.onInstalled.addListener(ensureAlarm);
chrome.runtime.onStartup.addListener(ensureAlarm);
chrome.alarms.onAlarm.addListener((alarm) => {
  if (alarm && alarm.name === "autoLogout") {
    clearAllCookiesAndStorage(() => {
      notifyAll({ type: "autoLogout" });
    });
  }
});
