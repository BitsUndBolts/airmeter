/* AirMeter shared utilities */
const $ = id => document.getElementById(id);

function escH(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;')
          .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

function fmtBytes(b) {
  if (b >= 1048576) return (b / 1048576).toFixed(1) + ' MB';
  if (b >= 1024)    return (b / 1024).toFixed(0) + ' KB';
  return b + ' B';
}

function getSignalBars(rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -67) return 3;
  if (rssi >= -78) return 2;
  return 1;
}

function thumbHtml(image, name) {
  const initials = (name || '??').replace(/[^A-Z0-9]/gi,'')
                                  .slice(0,2).toUpperCase() || '??';
  if (image) {
    return `<img src="${image}" alt="${escH(name)}"
                onload="this.classList.add('loaded')"
                onerror="this.style.display='none';
                         this.nextElementSibling.style.display='flex'">
            <span class="meter-thumb-fallback" style="display:none">
              ${initials}
            </span>`;
  }
  return `<span class="meter-thumb-fallback">${initials}</span>`;
}

let _toastTimer = null;
function showToast(msg, type = 'success') {
  const t = $('toast');
  if (!t) return;
  t.textContent = msg;
  t.className   = `toast ${type} visible`;
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => t.classList.remove('visible'), 2800);
}