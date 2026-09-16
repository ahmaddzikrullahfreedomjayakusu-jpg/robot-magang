<?php
/**
 * ======================================================================
 * FOLDER SHIELD / GATEKEEPER (DATABASE ACTIVE DEFENSE)
 * ======================================================================
 * - Auto-Logging ke tabel `keamanan_log_intruder`
 * - Auto-Banning ke tabel `keamanan_banned_ip` jika Strike >= 5
 * ======================================================================
 */

// Deteksi IP Asli (Anti Proxy/Cloudflare Bypass)
function get_real_visitor_ip() {
    if (isset($_SERVER["HTTP_CF_CONNECTING_IP"])) return $_SERVER["HTTP_CF_CONNECTING_IP"];
    if (isset($_SERVER['HTTP_X_FORWARDED_FOR'])) {
        $ip_array = explode(',', $_SERVER['HTTP_X_FORWARDED_FOR']);
        return trim($ip_array[0]);
    }
    if (isset($_SERVER['HTTP_X_REAL_IP'])) return $_SERVER['HTTP_X_REAL_IP'];
    return $_SERVER['REMOTE_ADDR'] ?? 'UNKNOWN_IP';
}

$root_path    = '';
$config_found = false;
$token_found  = false;
$search_paths = [__DIR__.'/', dirname(__DIR__).'/', dirname(dirname(__DIR__)).'/', dirname(dirname(dirname(__DIR__))).'/'];

foreach ($search_paths as $path) {
    if (!$config_found && file_exists($path . 'config.php')) { require_once $path . 'config.php'; $config_found = true; }
    if (!$token_found && file_exists($path . 'remember_tokens.php')) { require_once $path . 'remember_tokens.php'; $token_found = true; }
    if ($config_found && $token_found) { $root_path = $path; break; }
}

if (!$config_found || !$token_found) {
    http_response_code(403); exit('CRITICAL SYSTEM ERROR: Core config missing.');
}

rt_ensure_session_started();
$koneksi = rt_ensure_db_connection() ?? (isset($koneksi) ? $koneksi : null);
if (!($koneksi instanceof mysqli)) { http_response_code(500); exit('CRITICAL ERROR: DB offline.'); }
mysqli_report(MYSQLI_REPORT_OFF);

$ip_address = get_real_visitor_ip();
$ip_esc     = mysqli_real_escape_string($koneksi, $ip_address);

// CEK APAKAH IP SUDAH DIBANNED PERMANEN
$sql_ban_check = "SELECT id_banned FROM keamanan_banned_ip WHERE ip_address = '{$ip_esc}' LIMIT 1";
$res_ban = @mysqli_query($koneksi, $sql_ban_check);
if ($res_ban && mysqli_num_rows($res_ban) > 0) {
    http_response_code(403);
    exit("<h1 style='color:red; background:black; text-align:center; padding:50px; font-family:monospace;'>YOUR IP [{$ip_address}] IS PERMANENTLY BANNED FROM THIS SERVER.</h1>");
}

// VALIDASI SUPERADMIN
$user_id       = rt_get_current_user_id();
$is_superadmin = false;
$user_name     = 'UNKNOWN ENTITY';

if ($user_id > 0) {
    $sql_check = "SELECT nama_pengguna, role, status_akun FROM pengguna WHERE id_pengguna = ".(int)$user_id." LIMIT 1";
    $res_check = @mysqli_query($koneksi, $sql_check);
    if ($res_check && mysqli_num_rows($res_check) > 0) {
        $row = mysqli_fetch_assoc($res_check);
        if (strtolower(trim($row['role'])) === 'superadmin' && strtolower(trim($row['status_akun'])) === 'aktif') {
            $is_superadmin = true;
            $user_name = strtoupper(htmlspecialchars($row['nama_pengguna'], ENT_QUOTES, 'UTF-8'));
        }
    }
}

// LOGIKA PERTAHANAN (INTRUDER)
$req_uri    = $_SERVER['REQUEST_URI'] ?? 'UNKNOWN_URI';
$user_agent = $_SERVER['HTTP_USER_AGENT'] ?? 'UNKNOWN_UA';
$is_blacklisted = false;
$strikes    = 0;

if (!$is_superadmin) {
    http_response_code(403); // Forbidden
    
    if (!isset($_SESSION['intruder_strikes'])) $_SESSION['intruder_strikes'] = 0;
    $_SESSION['intruder_strikes'] += 1;
    $strikes = $_SESSION['intruder_strikes'];
    
    $status_tindakan = 'warning';

    if ($strikes >= 5) {
        $status_tindakan = 'banned';
        $is_blacklisted = true;
        // BANNED PERMANEN KE DATABASE
        @mysqli_query($koneksi, "INSERT IGNORE INTO keamanan_banned_ip (ip_address, alasan_blokir) VALUES ('{$ip_esc}', 'Mencoba akses folder terlarang secara brutal (Strike >= 5)')");
    } elseif ($strikes >= 3) {
        $status_tindakan = 'blacklisted';
        $is_blacklisted = true;
        sleep(2); // Delay
    }

    // CATAT LOG KE DATABASE
    $uri_esc = mysqli_real_escape_string($koneksi, $req_uri);
    $ua_esc  = mysqli_real_escape_string($koneksi, $user_agent);
    
    @mysqli_query($koneksi, "
        INSERT INTO keamanan_log_intruder (ip_address, user_agent, request_uri, strike_count, status_tindakan) 
        VALUES ('{$ip_esc}', '{$ua_esc}', '{$uri_esc}', {$strikes}, '{$status_tindakan}')
    ");

} else {
    http_response_code(200);
    $_SESSION['intruder_strikes'] = 0;
}

$json_auth = json_encode([
    'is_superadmin'  => $is_superadmin,
    'username'       => $user_name,
    'ip_address'     => $ip_address,
    'is_blacklisted' => $is_blacklisted,
    'strikes'        => $strikes
]);
?>
<!DOCTYPE html>
<html lang="id">
<head>
    <meta charset="UTF-8">
    <title>SHIELDED FOLDER - SATULINK CORE</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta name="robots" content="noindex, nofollow">
    <link href="https://fonts.googleapis.com/css2?family=Press+Start+2P&display=swap" rel="stylesheet">
    <style>
        :root { --bg-deep: #01040a; --hacker-green: #00ff66; --hacker-cyan: #00e5ff; --alert-red: #ff003c; --alert-yellow: #ffb700; --text-main: #f0f8ff; }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { background-color: var(--bg-deep); color: var(--text-main); font-family: monospace; min-height: 100vh; overflow: hidden; display: flex; align-items: center; justify-content: center; }
        
        .grid-bg { position: fixed; inset: -50%; background-image: linear-gradient(rgba(0, 255, 102, 0.1) 1px, transparent 1px), linear-gradient(90deg, rgba(0, 255, 102, 0.1) 1px, transparent 1px); background-size: 50px 50px; transform: perspective(700px) rotateX(60deg) translateY(-100px) translateZ(-200px); animation: gridMove 20s linear infinite; z-index: -3; }
        @keyframes gridMove { 0% { transform: perspective(700px) rotateX(60deg) translateY(0) translateZ(-200px); } 100% { transform: perspective(700px) rotateX(60deg) translateY(50px) translateZ(-200px); } }
        
        .scanlines { position: fixed; inset: 0; background: linear-gradient(to bottom, rgba(255,255,255,0), rgba(255,255,255,0) 50%, rgba(0,0,0,0.2) 50%, rgba(0,0,0,0.2)); background-size: 100% 4px; z-index: -1; pointer-events: none; }
        
        .terminal-overlay { position: fixed; inset: 0; background: rgba(1, 4, 10, 0.85); display: flex; align-items: center; justify-content: center; z-index: 9999; padding: 15px; opacity: 0; transition: opacity 0.3s ease; backdrop-filter: blur(4px); }
        .terminal-overlay.active { opacity: 1; }
        .terminal-box { width: 100%; max-width: 600px; background: rgba(2, 8, 18, 0.95); border: 2px solid var(--hacker-cyan); border-radius: 8px; box-shadow: 0 0 20px rgba(0, 229, 255, 0.4); overflow: hidden; }
        .terminal-overlay.warning .terminal-box { border-color: var(--alert-yellow); box-shadow: 0 0 30px rgba(255, 183, 0, 0.6); }
        .terminal-overlay.danger .terminal-box { border-color: var(--alert-red); box-shadow: 0 0 30px rgba(255, 0, 60, 0.6); animation: glitchShake 0.4s infinite; }
        @keyframes glitchShake { 0%, 100% { transform: translate(0); } 25% { transform: translate(-2px, 2px); } 50% { transform: translate(2px, -2px); } 75% { transform: translate(-2px, -2px); } }
        
        .terminal-header { background: rgba(0, 229, 255, 0.1); border-bottom: 1px solid var(--hacker-cyan); padding: 12px 18px; display: flex; justify-content: space-between; }
        .terminal-overlay.warning .terminal-header { background: rgba(255, 183, 0, 0.15); border-bottom-color: var(--alert-yellow); }
        .terminal-overlay.danger .terminal-header { background: rgba(255, 0, 60, 0.15); border-bottom-color: var(--alert-red); }
        .terminal-title { font-family: 'Press Start 2P', monospace; font-size: 11px; color: var(--hacker-cyan); text-transform: uppercase; }
        .terminal-overlay.warning .terminal-title { color: var(--alert-yellow); }
        .terminal-overlay.danger .terminal-title { color: var(--alert-red); }
        
        .terminal-body { padding: 25px; font-family: 'Consolas', monospace; font-size: 16px; font-weight: 700; line-height: 1.6; color: var(--hacker-green); text-shadow: 0 0 5px rgba(0, 255, 102, 0.6); }
        .terminal-overlay.warning .terminal-body { color: var(--alert-yellow); text-shadow: 0 0 5px rgba(255, 183, 0, 0.6); }
        .terminal-overlay.danger .terminal-body { color: var(--alert-red); text-shadow: 0 0 5px rgba(255, 0, 60, 0.6); }
        
        .typer { display: inline; }
        .cursor { display: inline-block; width: 10px; height: 18px; background: var(--hacker-green); margin-left: 5px; animation: blink 1s step-end infinite; }
        .terminal-overlay.warning .cursor { background: var(--alert-yellow); }
        .terminal-overlay.danger .cursor { background: var(--alert-red); }
        @keyframes blink { 50% { opacity: 0; } }

        .btn-action { display: inline-block; margin-top: 25px; padding: 12px 24px; background: transparent; color: var(--hacker-green); border: 2px solid var(--hacker-green); font-family: 'Press Start 2P', monospace; font-size: 11px; cursor: pointer; transition: all 0.2s ease; }
        .btn-action:hover { background: var(--hacker-green); color: #000; box-shadow: 0 0 15px var(--hacker-green); }

        .secured-content { display: none; text-align: center; }
        .secured-content h1 { font-family: 'Press Start 2P', monospace; font-size: 24px; color: var(--hacker-green); text-shadow: 0 0 10px var(--hacker-green); margin-bottom: 20px; }
    </style>
</head>
<body>

<div class="grid-bg"></div>
<div class="scanlines"></div>

<div class="secured-content" id="securedContent">
    <h1>CORE VAULT SECURED</h1>
    <button class="btn-action" onclick="window.location.href='../index.php'">KEMBALI KE BERANDA</button>
</div>

<div class="terminal-overlay" id="termOverlay">
    <div class="terminal-box">
        <div class="terminal-header">
            <span class="terminal-title" id="termTitle">SYSTEM TERMINAL</span>
            <span class="terminal-title">SHIELD v4.DB</span>
        </div>
        <div class="terminal-body">
            <div id="termOutput" class="typer"></div><div class="cursor"></div>
            <div id="termAction" style="display:none; margin-top:20px; font-family:'Press Start 2P', monospace; font-size: 12px;"></div>
        </div>
    </div>
</div>

<script>
    (function() {
        const authData = <?php echo $json_auth; ?>;
        const overlay = document.getElementById('termOverlay');
        const title = document.getElementById('termTitle');
        const output = document.getElementById('termOutput');
        const actionArea = document.getElementById('termAction');
        const securedContent = document.getElementById('securedContent');
        
        let msgArray = [];
        let speed = 35; 
        
        if (authData.is_superadmin) {
            title.textContent = "SECURITY CLEARANCE: VALID";
            document.querySelector('.cursor').style.background = 'var(--hacker-green)';
            msgArray = [
                "> DB LINK ESTABLISHED...",
                "> MATCH FOUND: " + authData.username,
                "> ROLE CONFIRMED: SUPERADMIN",
                "> ACCESS GRANTED."
            ];
            typeLines(msgArray, speed, function() {
                actionArea.innerHTML = '<button class="btn-action" id="btnEnter">TUTUP TERMINAL</button>';
                actionArea.style.display = 'block';
                document.getElementById('btnEnter').addEventListener('click', function() {
                    overlay.classList.remove('active');
                    securedContent.style.display = 'block';
                });
            });

        } else {
            if (authData.strikes >= 5) {
                overlay.classList.add('danger');
                title.textContent = "CRITICAL: IP BANNED PERMANENTLY";
                msgArray = [
                    "> MAXIMUM STRIKES EXCEEDED (5/5).",
                    "> IP: " + authData.ip_address + " HAS BEEN INSERTED TO BAN_LIST.",
                    "> FORCING CONNECTION DROP..."
                ];
                speed = 25; 
            } else if (authData.is_blacklisted) {
                overlay.classList.add('danger');
                title.textContent = "CRITICAL: RATE LIMIT EXCEEDED";
                msgArray = [
                    "> MULTIPLE INTRUSION ATTEMPTS DETECTED.",
                    "> STRIKES: " + authData.strikes + "/5",
                    "> IP: " + authData.ip_address + " HAS BEEN LOGGED TO DATABASE.",
                    "> THROTTLING CONNECTION..."
                ];
                speed = 25; 
            } else {
                overlay.classList.add('warning');
                title.textContent = "WARNING: UNAUTHORIZED";
                msgArray = [
                    "> DIRECT DIRECTORY ACCESS FORBIDDEN.",
                    "> LOGGING IP: " + authData.ip_address,
                    "> STRIKE: " + authData.strikes + "/5",
                    "> REDIRECTING REQUEST..."
                ];
            }
            
            typeLines(msgArray, speed, function() {
                let timeLeft = authData.is_blacklisted ? 8 : 4;
                actionArea.innerHTML = `MENGALIHKAN DALAM <span style="color:var(--alert-red);" id="cdNum">${timeLeft}</span> DETIK`;
                actionArea.style.display = 'block';
                
                let cdInterval = setInterval(function() {
                    timeLeft--;
                    document.getElementById('cdNum').textContent = timeLeft;
                    if (timeLeft <= 0) {
                        clearInterval(cdInterval);
                        window.location.href = '../index.php'; 
                    }
                }, 1000);
            });
        }
        
        setTimeout(() => { overlay.classList.add('active'); }, 300);

        function typeLines(lines, delay, callback) {
            let lineIndex = 0, charIndex = 0;
            function typeChar() {
                if (lineIndex < lines.length) {
                    if (charIndex < lines[lineIndex].length) {
                        output.innerHTML += lines[lineIndex].charAt(charIndex);
                        charIndex++; setTimeout(typeChar, delay);
                    } else {
                        output.innerHTML += '<br><br>';
                        lineIndex++; charIndex = 0;
                        setTimeout(typeChar, delay * 3);
                    }
                } else {
                    if (typeof callback === 'function') callback();
                }
            }
            typeChar();
        }
    })();
</script>
</body>
</html>