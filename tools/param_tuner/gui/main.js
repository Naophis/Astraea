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

// ログ記録用の変数
let isLogging = false;
let logFilePath = null;
let logStream = null;

// CSV記録用の変数
let csvObj = {
  dump_to_csv_ready: false,
  data_struct: [],
  file_name: '',
  record: '',
  byte_size: 0
};
let binaryMode = false;
let LOG_STRUCT_SIZE = 12;

// タイムスタンプ付きファイル名生成
function getNowYMDHMS() {
  const dt = new Date();
  const y = dt.getFullYear();
  const m = ('00' + (dt.getMonth() + 1)).slice(-2);
  const d = ('00' + dt.getDate()).slice(-2);
  const h = ('00' + dt.getHours()).slice(-2);
  const M = ('00' + dt.getMinutes()).slice(-2);
  const s = ('00' + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}${s}`;
}

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

          // ログ記録中の場合、ファイルに書き込む
          if (isLogging && logStream) {
            const timestamp = new Date().toISOString();
            logStream.write(`[${timestamp}] ${data}\n`);
          }

          // CSV記録用の特殊コマンド処理
          handleLineData(data);
        });

        resolve({ success: true });
      }
    });
  });
});

// ラインモードのデータ処理
function handleLineData(data) {
  // ready___コマンド検出
  if (data.match(/^ready___/)) {
    csvObj.dump_to_csv_ready = true;
    csvObj.file_name = `${getNowYMDHMS()}.csv`;
    csvObj.record = '';
    csvObj.data_struct = [];
    csvObj.byte_size = parseInt(data.split(':')[1]);
    console.log('CSV Ready mode activated:', csvObj);
    return;
  }

  // データ構造定義の収集
  if (csvObj.dump_to_csv_ready) {
    const d = data.split(':');
    if (d.length === 3) {
      const name = d[0];
      const type = d[1];
      const size = parseInt(d[2]);
      csvObj.data_struct.push({ name, type, size });
      console.log('Data struct added:', { name, type, size });
    }
  }

  // startコマンド検出 - バイナリモードへ切り替え
  if (data.match(/^start/)) {
    csvObj.file_name = `${getNowYMDHMS()}.csv`;
    csvObj.record = '';
    console.log('Switching to binary mode:', csvObj);
    switchToBinaryMode();
  }
}

// バイナリモードへ切り替え
function switchToBinaryMode() {
  const dataSize = 48; // 固定サイズ
  const dataSize2 = csvObj.data_struct.reduce((prev, cur) => prev + cur.size, 0);
  console.log('Binary mode data size:', dataSize, 'calculated:', dataSize2);

  LOG_STRUCT_SIZE = dataSize2 / (4 * 12);
  binaryMode = true;

  // パイプラインをクリア
  port.unpipe(parser);

  // ByteLengthパーサに切り替え
  parser = port.pipe(new ByteLengthParser({ length: dataSize }));

  // ヘッダー行を追加
  const header = csvObj.data_struct.map(d => d.name).join(',');
  csvObj.record += `${header}\n`;
  console.log('CSV Header:', header);

  let cnt = 0;
  let record = [];
  let lastIndex = 0;
  let lastReceived = Date.now();
  let forceSave = false;
  let lastIdx = 0;

  // タイムアウトチェック
  const interval = setInterval(() => {
    const now = Date.now();
    if (now - lastReceived > 1000 || forceSave) {
      console.log('Saving CSV file due to timeout or force save');
      saveCsvFile();
      clearInterval(interval);
      switchToLineMode();
    }
  }, 1000);

  parser.on('data', (binaryData) => {
    let offset = 0;
    cnt++;

    const startIdx = (cnt - 1) * 12;
    const endIdx = startIdx + 12;

    for (let i = startIdx; i < endIdx; i++) {
      const dataStruct = csvObj.data_struct[i];
      if (!dataStruct) continue;

      const tmpData = offset;
      switch (dataStruct.type) {
        case 'float':
          record.push(binaryData.readFloatLE(tmpData));
          offset += dataStruct.size;
          break;
        case 'int':
          record.push(binaryData.readInt32LE(tmpData));
          offset += dataStruct.size;
          break;
        case 'short':
          record.push(binaryData.readInt16LE(tmpData));
          offset += dataStruct.size;
          break;
      }

      if (dataStruct.name === 'index') {
        const idx = binaryData.readInt32LE(tmpData);
        if (idx < lastIdx) {
          forceSave = true;
        }
        lastIdx = idx;
      }
    }

    lastReceived = Date.now();

    if (cnt === LOG_STRUCT_SIZE) {
      cnt = 0;
      if (record.length > 0 && record[0] > 0) {
        lastIndex = record[0];
        const str = record.join(',');
        csvObj.record += `${str}\n`;
      }
      record = [];
    }
  });
}

// ラインモードへ切り替え
function switchToLineMode() {
  binaryMode = false;
  port.unpipe(parser);
  parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

  parser.on('data', (data) => {
    if (mainWindow) {
      mainWindow.webContents.send('serial-data', data);
    }

    if (isLogging && logStream) {
      const timestamp = new Date().toISOString();
      logStream.write(`[${timestamp}] ${data}\n`);
    }

    handleLineData(data);
  });

  // CSV状態をリセット
  csvObj = {
    dump_to_csv_ready: false,
    data_struct: [],
    file_name: '',
    record: '',
    byte_size: 0
  };
}

// CSVファイルを保存
function saveCsvFile() {
  try {
    const logsDir = path.join(__dirname, '..', 'logs');
    if (!fs.existsSync(logsDir)) {
      fs.mkdirSync(logsDir, { recursive: true });
    }

    const filePath = path.join(logsDir, csvObj.file_name);
    fs.writeFileSync(filePath, csvObj.record, { flag: 'w+' });

    // latest.csvにもコピー
    const latestPath = path.join(logsDir, 'latest.csv');
    fs.copyFileSync(filePath, latestPath);

    console.log('CSV file saved:', filePath);
    if (mainWindow) {
      mainWindow.webContents.send('serial-data', `CSV saved: ${csvObj.file_name}`);
    }
  } catch (err) {
    console.error('Error saving CSV file:', err);
  }
}

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

// ログ記録開始
ipcMain.handle('start-logging', async () => {
  try {
    if (isLogging) {
      return { success: false, message: 'Already logging' };
    }

    // log2ディレクトリを作成（存在しない場合）
    const logsDir = path.join(__dirname, '..', 'log2');
    if (!fs.existsSync(logsDir)) {
      fs.mkdirSync(logsDir, { recursive: true });
    }

    // ログファイルパスを生成
    const fileName = getNowYMDHMS();
    logFilePath = path.join(logsDir, fileName);

    // 書き込みストリームを作成
    logStream = fs.createWriteStream(logFilePath, { flags: 'a' });

    isLogging = true;

    return {
      success: true,
      filePath: logFilePath,
      fileName: fileName
    };
  } catch (err) {
    console.error('Error starting logging:', err);
    return { success: false, message: err.message };
  }
});

// ログ記録停止
ipcMain.handle('stop-logging', async () => {
  try {
    if (!isLogging) {
      return { success: false, message: 'Not logging' };
    }

    // ストリームをクローズ
    if (logStream) {
      logStream.end();
      logStream = null;
    }

    const savedPath = logFilePath;
    isLogging = false;
    logFilePath = null;

    return {
      success: true,
      filePath: savedPath
    };
  } catch (err) {
    console.error('Error stopping logging:', err);
    return { success: false, message: err.message };
  }
});

// ログ記録状態取得
ipcMain.handle('get-logging-status', async () => {
  return {
    isLogging: isLogging,
    filePath: logFilePath
  };
});
