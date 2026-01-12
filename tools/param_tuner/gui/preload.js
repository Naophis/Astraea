const { contextBridge, ipcRenderer } = require('electron');

// レンダラープロセスに安全にAPIを公開
contextBridge.exposeInMainWorld('electronAPI', {
  // シリアルポート関連
  listPorts: () => ipcRenderer.invoke('list-ports'),
  connectPort: (portPath) => ipcRenderer.invoke('connect-port', portPath),
  disconnectPort: () => ipcRenderer.invoke('disconnect-port'),
  onSerialData: (callback) => {
    ipcRenderer.on('serial-data', (event, data) => callback(data));
  },

  // プロファイル関連
  listModes: () => ipcRenderer.invoke('list-modes'),
  listProfiles: (mode) => ipcRenderer.invoke('list-profiles', mode),
  sendParameter: (params) => ipcRenderer.invoke('send-parameter', params),
  sendAllParameters: (mode) => ipcRenderer.invoke('send-all-parameters', mode),

  // ログ記録関連
  startLogging: () => ipcRenderer.invoke('start-logging'),
  stopLogging: () => ipcRenderer.invoke('stop-logging'),
  getLoggingStatus: () => ipcRenderer.invoke('get-logging-status'),
});
