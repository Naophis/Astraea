// DOM要素
const elements = {
  connectionStatus: document.getElementById('connection-status'),
  refreshPortsBtn: document.getElementById('refresh-ports'),
  portSelect: document.getElementById('port-select'),
  connectBtn: document.getElementById('connect-btn'),
  disconnectBtn: document.getElementById('disconnect-btn'),
  modeSelect: document.getElementById('mode-select'),
  systemFiles: document.getElementById('system-files'),
  profileFiles: document.getElementById('profile-files'),
  sendAllBtn: document.getElementById('send-all-btn'),
  logOutput: document.getElementById('log-output'),
  clearLogBtn: document.getElementById('clear-log-btn'),
};

// 状態管理
let state = {
  connected: false,
  selectedMode: '',
  currentPort: '',
};

// ログ追加関数
function addLog(message, type = 'info') {
  const logLine = document.createElement('div');
  logLine.className = `log-line ${type}`;
  const timestamp = new Date().toLocaleTimeString();
  logLine.textContent = `[${timestamp}] ${message}`;
  elements.logOutput.appendChild(logLine);
  elements.logOutput.scrollTop = elements.logOutput.scrollHeight;
}

// 接続状態更新
function updateConnectionStatus(connected) {
  state.connected = connected;
  if (connected) {
    elements.connectionStatus.textContent = `接続済: ${state.currentPort}`;
    elements.connectionStatus.className = 'status-connected';
    elements.connectBtn.disabled = true;
    elements.disconnectBtn.disabled = false;
    elements.sendAllBtn.disabled = false;
  } else {
    elements.connectionStatus.textContent = '未接続';
    elements.connectionStatus.className = 'status-disconnected';
    elements.connectBtn.disabled = false;
    elements.disconnectBtn.disabled = true;
    elements.sendAllBtn.disabled = true;
    state.currentPort = '';
  }
  updateFileButtons();
}

// ファイル送信ボタンの有効/無効を更新
function updateFileButtons() {
  const fileButtons = document.querySelectorAll('.file-btn');
  fileButtons.forEach(btn => {
    btn.disabled = !state.connected;
  });
}

// シリアルポート一覧を更新
async function refreshPorts() {
  addLog('ポートを検索中...', 'info');
  try {
    const ports = await window.electronAPI.listPorts();
    elements.portSelect.innerHTML = '<option value="">ポートを選択...</option>';

    if (ports.length === 0) {
      addLog('利用可能なポートが見つかりませんでした', 'warning');
    } else {
      ports.forEach(port => {
        const option = document.createElement('option');
        option.value = port.path;
        option.textContent = `${port.path} (${port.serialNumber})`;
        elements.portSelect.appendChild(option);
      });
      addLog(`${ports.length}個のポートが見つかりました`, 'success');
    }
  } catch (err) {
    addLog(`ポート検索エラー: ${err.message}`, 'error');
  }
}

// シリアルポートに接続
async function connect() {
  const selectedPort = elements.portSelect.value;
  if (!selectedPort) {
    addLog('ポートを選択してください', 'warning');
    return;
  }

  addLog(`接続中: ${selectedPort}`, 'info');
  try {
    await window.electronAPI.connectPort(selectedPort);
    state.currentPort = selectedPort;
    updateConnectionStatus(true);
    addLog(`接続成功: ${selectedPort}`, 'success');
  } catch (err) {
    addLog(`接続エラー: ${err.message}`, 'error');
  }
}

// シリアルポートから切断
async function disconnect() {
  addLog('切断中...', 'info');
  try {
    await window.electronAPI.disconnectPort();
    updateConnectionStatus(false);
    addLog('切断しました', 'success');
  } catch (err) {
    addLog(`切断エラー: ${err.message}`, 'error');
  }
}

// モード一覧を読み込み
async function loadModes() {
  try {
    const modes = await window.electronAPI.listModes();
    elements.modeSelect.innerHTML = '<option value="">モードを選択...</option>';

    modes.forEach(mode => {
      const option = document.createElement('option');
      option.value = mode;
      option.textContent = mode;
      elements.modeSelect.appendChild(option);
    });

    // デフォルトで hf を選択
    if (modes.includes('hf')) {
      elements.modeSelect.value = 'hf';
      await loadProfiles('hf');
    }
  } catch (err) {
    addLog(`モード読み込みエラー: ${err.message}`, 'error');
  }
}

// プロファイル一覧を読み込み
async function loadProfiles(mode) {
  if (!mode) return;

  state.selectedMode = mode;
  addLog(`プロファイルを読み込み中: ${mode}`, 'info');

  try {
    const { system, profiles } = await window.electronAPI.listProfiles(mode);

    // システムファイルを表示
    elements.systemFiles.innerHTML = '';
    system.forEach(file => {
      const fileItem = createFileItem(file, true);
      elements.systemFiles.appendChild(fileItem);
    });

    // プロファイルファイルを表示
    elements.profileFiles.innerHTML = '';
    profiles.forEach(file => {
      const fileItem = createFileItem(file, false);
      elements.profileFiles.appendChild(fileItem);
    });

    updateFileButtons();
    addLog(`プロファイル読み込み完了: システム ${system.length}個, プロファイル ${profiles.length}個`, 'success');
  } catch (err) {
    addLog(`プロファイル読み込みエラー: ${err.message}`, 'error');
  }
}

// ファイルアイテムのHTML要素を作成
function createFileItem(filename, isSystem) {
  const item = document.createElement('div');
  item.className = 'file-item';

  const nameSpan = document.createElement('span');
  nameSpan.className = 'file-name';
  nameSpan.textContent = filename;

  const sendBtn = document.createElement('button');
  sendBtn.className = 'file-btn';
  sendBtn.textContent = '送信';
  sendBtn.disabled = !state.connected;
  sendBtn.onclick = () => sendParameter(filename, isSystem);

  item.appendChild(nameSpan);
  item.appendChild(sendBtn);

  return item;
}

// パラメータ送信
async function sendParameter(filename, isSystem) {
  if (!state.connected) {
    addLog('ポートに接続してください', 'warning');
    return;
  }

  addLog(`送信中: ${filename}`, 'info');
  try {
    const result = await window.electronAPI.sendParameter({
      mode: state.selectedMode,
      filename: filename,
      isSystem: isSystem,
    });

    if (result.success) {
      addLog(`送信成功: ${filename} → ${result.filename}`, 'success');
    }
  } catch (err) {
    addLog(`送信エラー (${filename}): ${err.message}`, 'error');
  }
}

// 全パラメータ送信
async function sendAllParameters() {
  if (!state.connected) {
    addLog('ポートに接続してください', 'warning');
    return;
  }

  if (!state.selectedMode) {
    addLog('モードを選択してください', 'warning');
    return;
  }

  addLog('全パラメータ送信を開始します...', 'info');
  elements.sendAllBtn.disabled = true;

  try {
    const { system, profiles } = await window.electronAPI.listProfiles(state.selectedMode);

    // プロファイル固有のファイルを送信
    for (const file of profiles) {
      if (file.match(/\.yaml$/)) {
        await sendParameter(file, false);
        await sleep(800);
      }
    }

    // システムファイルを送信
    for (const file of system) {
      await sendParameter(file, true);
      await sleep(800);
    }

    addLog('全パラメータの送信が完了しました', 'success');
  } catch (err) {
    addLog(`全送信エラー: ${err.message}`, 'error');
  } finally {
    elements.sendAllBtn.disabled = false;
  }
}

// ログクリア
function clearLog() {
  elements.logOutput.innerHTML = '';
  addLog('ログをクリアしました', 'info');
}

// ユーティリティ関数
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

// イベントリスナー設定
elements.refreshPortsBtn.addEventListener('click', refreshPorts);
elements.connectBtn.addEventListener('click', connect);
elements.disconnectBtn.addEventListener('click', disconnect);
elements.modeSelect.addEventListener('change', (e) => {
  loadProfiles(e.target.value);
});
elements.sendAllBtn.addEventListener('click', sendAllParameters);
elements.clearLogBtn.addEventListener('click', clearLog);

// シリアルデータ受信
window.electronAPI.onSerialData((data) => {
  addLog(data, 'info');
});

// 初期化
(async function init() {
  addLog('アプリケーションを起動しました', 'success');
  await refreshPorts();
  await loadModes();
})();
