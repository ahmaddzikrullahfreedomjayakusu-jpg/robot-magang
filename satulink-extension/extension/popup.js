// ========================================
// popup.js — FINAL FULL SYNC (MV3)
// ========================================
"use strict";

document.addEventListener("DOMContentLoaded", () => {
  // Elements
  const statusMsg = document.getElementById("statusMsg");
  const aksesWebBtn = document.getElementById("aksesWeb");
  const darkBtn = document.getElementById("darkModeBtn");

  // Toast elements
  const toast = document.getElementById("toast");
  const toastMsg = document.getElementById("toastMsg");
  const toastIcon = document.getElementById("toastIcon");

  // Sounds (opsional di popup.html)
  const toggleSnd = document.getElementById("toggleSound");

  // Small utils
  const openInTab = (url) => {
    try {
      if (chrome?.tabs?.create) chrome.tabs.create({ url });
      else window.open(url, "_blank", "noopener,noreferrer");
    } catch {
      window.open(url, "_blank", "noopener,noreferrer");
    }
  };

  const cssVar = (name, fallback) => {
    const val = getComputedStyle(document.documentElement).getPropertyValue(
      name,
    );
    return (val && val.trim()) || fallback;
  };

  // -------------------------
  // 0️⃣ Toast util
  // -------------------------
  function showToast(message, type = "info") {
    if (!toast || !toastMsg || !toastIcon) {
      console.log(`[TOAST:${type}] ${message}`);
      return;
    }

    toastMsg.textContent = message;

    const icons = { success: "✅", error: "❌", info: "ℹ️", warning: "⚠️" };
    toastIcon.textContent = icons[type] || icons.info;

    let bg = cssVar("--accent", "#ffcd38");
    if (type === "success") bg = cssVar("--success", "#28a745");
    if (type === "error") bg = cssVar("--danger", "#dc3545");
    if (type === "warning") bg = cssVar("--warning", "#fd7e14");
    toast.style.background = bg;

    // tampilkan (animasi masuk)
    toast.classList.remove("hidden");
    // force reflow agar transisi 'show' berjalan
    // eslint-disable-next-line no-unused-expressions
    toast.offsetHeight;
    toast.classList.add("show");

    clearTimeout(showToast._timer);
    showToast._timer = setTimeout(() => {
      toast.classList.remove("show");
      setTimeout(() => {
        toast.classList.add("hidden");
        toastIcon.textContent = "";
      }, 300); // waktu animasi keluar
    }, 4000);
  }

  // -------------------------
  // 1️⃣ Status teks (blink)
  // -------------------------
  if (statusMsg) {
    statusMsg.classList.add("status-blink");
    statusMsg.innerHTML =
      "✨ Extension ini hanya syarat saja,<br>" +
      "Akses via Dashboard Website " +
      '<a href="https://satulink.id" target="_blank" rel="noreferrer noopener">https://satulink.id</a>';
  }

  // -------------------------
  // 2️⃣ Tombol "Akses Web"
  // -------------------------
  aksesWebBtn?.addEventListener("click", () => {
    openInTab("https://satulink.id/user/profile.php");
    showToast("🌐 Membuka Dashboard Satulink…", "info");
  });

  // -------------------------
  // 3️⃣ Dark Mode (persist localStorage)
  // -------------------------
  if (localStorage.getItem("darkMode") === "true") {
    document.body.classList.add("dark");
  }

  darkBtn?.addEventListener("click", () => {
    const isDark = document.body.classList.toggle("dark");
    localStorage.setItem("darkMode", isDark ? "true" : "false");
    // play sound (opsional)
    try {
      toggleSnd && toggleSnd.play();
    } catch {}
    showToast(isDark ? "🌙 Dark Mode Aktif" : "☀️ Light Mode Aktif", "info");
  });

  // -------------------------
  // 4️⃣ Footer links buka via chrome.tabs (lebih andal untuk MV3)
  // -------------------------
  document.querySelectorAll("footer a[href]").forEach((link) => {
    link.addEventListener("click", (e) => {
      e.preventDefault();
      const url = link.getAttribute("href");
      if (!url) return;
      openInTab(url);
    });
  });

  // -------------------------
  // 5️⃣ Listener pesan dari background.js
  // -------------------------
  chrome.runtime.onMessage.addListener((message) => {
    if (!message || !message.type) return;

    if (message.type === "cookiesSaved") {
      const total = message.summary?.total ?? 0;
      const domains = message.summary?.byDomain
        ? Object.keys(message.summary.byDomain)
        : [];
      const info = domains.length ? ` (${domains.length} domain)` : "";
      showToast(`✅ Data tersimpan: ${total}${info}`, "success");
      return;
    }

    if (message.type === "cookiesAppliedForDomain") {
      const d = message.domain || "-";
      const ok = message.result?.ok ?? 0;
      const fail = message.result?.fail ?? 0;
      console.log(`🍪 Domain ${d}: ok=${ok}, fail=${fail}`);
      return;
    }

    if (message.type === "resetToken") {
      showToast("🔄 Token direset oleh website. Data dibersihkan!", "warning");
      return;
    }

    if (message.type === "autoLogout") {
      showToast("⏰ Auto logout! Data & storage dihapus otomatis.", "warning");
      return;
    }

    if (message.type === "cookiesCleared") {
      showToast("🗑️ Semua data telah dihapus!", "info");
      return;
    }
  });

  // -------------------------
  // 6️⃣ Ping kecil ke background (opsional)
  // -------------------------
  try {
    chrome.runtime.sendMessage(
      { type: "popupReady" },
      () => void chrome.runtime.lastError,
    );
  } catch {}
});

(function () {
  try {
    var darkBtn = document.getElementById("darkModeBtn");
    var isDark = localStorage.getItem("darkMode") === "true";
    if (isDark) document.body.classList.add("dark");
    if (darkBtn)
      darkBtn.setAttribute("aria-pressed", isDark ? "true" : "false");

    // keep aria-pressed in sync when button toggled by your popup.js
    darkBtn &&
      darkBtn.addEventListener("click", function () {
        var pressed = this.getAttribute("aria-pressed") === "true";
        this.setAttribute("aria-pressed", (!pressed).toString());
      });
  } catch (e) {
    /* silent */
  }
})();

(function initRipples() {
  // Delegated pointerdown handler for .ripple elements
  document.addEventListener(
    "pointerdown",
    function (e) {
      try {
        var el = e.target.closest && e.target.closest(".ripple");
        if (!el) return;

        // ignore disabled controls
        if (el.disabled || el.getAttribute("aria-disabled") === "true") return;

        // create wave
        var wave = document.createElement("span");
        wave.className = "ripple-wave";

        // append
        el.appendChild(wave);

        var rect = el.getBoundingClientRect();
        var maxDim = Math.max(rect.width, rect.height);
        var size = Math.round(maxDim * 1.6);
        wave.style.width = wave.style.height = size + "px";

        var x = e.clientX - rect.left - size / 2;
        var y = e.clientY - rect.top - size / 2;

        if (isNaN(x) || isNaN(y)) {
          x = (rect.width - size) / 2;
          y = (rect.height - size) / 2;
        }

        wave.style.left = Math.max(-rect.width, Math.min(rect.width, x)) + "px";
        wave.style.top =
          Math.max(-rect.height, Math.min(rect.height, y)) + "px";

        // cleanup after animation
        var removeWave = function () {
          if (wave && wave.parentNode) wave.parentNode.removeChild(wave);
        };
        wave.addEventListener("animationend", removeWave, { once: true });
        // safety fallback
        setTimeout(removeWave, 900);
      } catch (err) {
        // fail silently - ripple is decorative
        console.error("ripple error", err);
      }
    },
    { passive: true },
  );

  // Optional: keyboard support for Enter/Space (centered ripple)
  document.addEventListener("keydown", function (e) {
    if (e.key === "Enter" || e.key === " ") {
      var el = document.activeElement;
      if (!el || !el.classList.contains("ripple")) return;
      // create synthetic centered ripple
      var rect = el.getBoundingClientRect();
      var fakeEvent = {
        clientX: rect.left + rect.width / 2,
        clientY: rect.top + rect.height / 2,
        target: el,
      };
      // reuse pointerdown handler logic via dispatchEvent? create one-off:
      var evt = new PointerEvent("pointerdown", {
        clientX: fakeEvent.clientX,
        clientY: fakeEvent.clientY,
        bubbles: true,
      });
      el.dispatchEvent(evt);
    }
  });
})();
