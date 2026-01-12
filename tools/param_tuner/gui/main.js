const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const yaml = require('js-yaml');

const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");
const { ByteLengthParser } = require("@serialport/parser-byte-length");
let mainWindow;
let port = null;
let parser = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js')
    }
  });

  mainWindow.loadFile(path.join(__dirname, 'index.html'));

  // 開発ツールを開く
  mainWindow.webContents.openDevTools();

  mainWindow.on('closed', () => {
    if (port && port.isOpen) {
      port.close();
    }
    mainWindow = null;
  });
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});

// シリアルポート一覧取得
ipcMain.handle('list-ports', async () => {
  try {
    const ports = await SerialPort.list();
    const filteredPorts = ports.filter(p => {
      return (
        (p.path.match(/usbserial/) ||
          p.path.match(/COM/) ||
          p.path.match(/ttyUSB/) ||
          p.path.match(/ttyACM/)) &&
        p.serialNumber
      );
    });
    return filteredPorts.map(p => ({
      path: p.path,
      serialNumber: p.serialNumber
    }));
  } catch (err) {
    console.error('Error listing ports:', err);
    return [];
  }
});

// シリアルポート接続
ipcMain.handle('connect-port', async (event, portPath) => {
  return new Promise((resolve, reject) => {
    if (port && port.isOpen) {
      port.close();
    }

    port = new SerialPort({
      baudRate: 3000000,
      path: portPath
    }, (err) => {
      if (err) {
        console.error('Error opening port:', err);
        reject(err);
      } else {
        parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

        parser.on('data', (data) => {
          if (mainWindow) {
            mainWindow.webContents.send('serial-data', data);
          }
        });

        resolve({ success: true });
      }
    });
  });
});

// シリアルポート切断
ipcMain.handle('disconnect-port', async () => {
  return new Promise((resolve) => {
    if (port && port.isOpen) {
      port.close(() => {
        port = null;
        parser = null;
        resolve({ success: true });
      });
    } else {
      resolve({ success: true });
    }
  });
});

// プロファイル一覧取得
ipcMain.handle('list-profiles', async (event, mode) => {
  try {
    const profileDir = path.join(__dirname, '..', 'profile', mode);
    const systemFiles = ['system.yaml', 'hardware.yaml'];
    const files = fs.readdirSync(profileDir);

    const yamlFiles = files.filter(file =>
      file.match(/\.yaml$/) || file.match(/\.maze$/)
    );

    return {
      system: systemFiles,
      profiles: yamlFiles
    };
  } catch (err) {
    console.error('Error listing profiles:', err);
    return { system: [], profiles: [] };
  }
});

// パラメータ送信
ipcMain.handle('send-parameter', async (event, { mode, filename, isSystem }) => {
  return new Promise(async (resolve, reject) => {
    if (!port || !port.isOpen) {
      reject(new Error('Port not connected'));
      return;
    }

    try {
      const profileDir = isSystem
        ? path.join(__dirname, '..', 'profile')
        : path.join(__dirname, '..', 'profile', mode);

      const filePath = path.join(profileDir, filename);
      let txt = fs.readFileSync(filePath, { encoding: 'utf-8' });

      let sendData;
      let sendFilename;

      // .mazeファイルの処理
      if (filename.match(/\.maze$/)) {
        sendFilename = 'maze.txt';
        let mazeList = txt.split(',')
          .map(e => e.trim())
          .map(e => parseInt(e) | 0xf0);

        let size = 16;
        if (mazeList.length > 300) {
          size = 32;
        }

        // 行列の転置
        for (let y = 0; y < size; y++) {
          for (let x = 0; x < size; x++) {
            if (x >= y) continue;
            let idx = y * size + x;
            let idx2 = x * size + y;
            let tmp = mazeList[idx];
            mazeList[idx] = mazeList[idx2];
            mazeList[idx2] = tmp;
          }
        }

        sendData = mazeList.join(',');
      }
      // YAMLファイルの処理
      else if (filename.match(/\.yaml$/)) {
        const saveData = yaml.load(txt);

        if (isSystem) {
          sendFilename = filename.replace('yaml', 'txt');
        } else {
          sendFilename = filename.replace('yaml', mode);
        }

        sendData = JSON.stringify(saveData);
      }

      const command = `${sendFilename}@${sendData}`;

      port.write(command, (err) => {
        if (err) {
          reject(err);
        } else {
          setTimeout(() => {
            resolve({
              success: true,
              filename: sendFilename
            });
          }, 600);
        }
      });
    } catch (err) {
      reject(err);
    }
  });
});

// 全パラメータ送信
ipcMain.handle('send-all-parameters', async (event, mode) => {
  return new Promise(async (resolve, reject) => {
    if (!port || !port.isOpen) {
      reject(new Error('Port not connected'));
      return;
    }

    try {
      const results = [];

      // プロファイル固有のファイルを送信
      const profileDir = path.join(__dirname, '..', 'profile', mode);
      const files = fs.readdirSync(profileDir);
      const yamlFiles = files.filter(file => file.match(/\.yaml$/));

      for (const file of yamlFiles) {
        try {
          const result = await ipcMain.emit('send-parameter', event, {
            mode,
            filename: file,
            isSystem: false
          });
          results.push({ file, success: true });
          await new Promise(resolve => setTimeout(resolve, 800));
        } catch (err) {
          results.push({ file, success: false, error: err.message });
        }
      }

      // システムファイルを送信
      for (const file of ['system.yaml', 'hardware.yaml']) {
        try {
          const result = await ipcMain.emit('send-parameter', event, {
            mode,
            filename: file,
            isSystem: true
          });
          results.push({ file, success: true });
          await new Promise(resolve => setTimeout(resolve, 800));
        } catch (err) {
          results.push({ file, success: false, error: err.message });
        }
      }

      resolve({ success: true, results });
    } catch (err) {
      reject(err);
    }
  });
});

// モード一覧取得
ipcMain.handle('list-modes', async () => {
  try {
    const profileDir = path.join(__dirname, '..', 'profile');
    const items = fs.readdirSync(profileDir, { withFileTypes: true });
    const modes = items
      .filter(item => item.isDirectory())
      .map(item => item.name);
    return modes;
  } catch (err) {
    console.error('Error listing modes:', err);
    return [];
  }
});
