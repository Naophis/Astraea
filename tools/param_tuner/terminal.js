var fs = require("fs");
const yaml = require("js-yaml");
const { decode } = require("punycode");
const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");
const { ByteLengthParser } = require("@serialport/parser-byte-length");
const path = require("path");
const { argv } = require("process");

let comport;
let port;
let parser;
let binaryMode = false;

// ===== ユーティリティ関数 =====

const getNowYMD = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.csv`;
};

const getNowYMD_maze = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.maze`;
};

function resolveAfter2Seconds(str) {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(str);
    }, 500);
  });
}

async function sleep2(delay, result) {
  return new Promise((resolve) => {
    setTimeout(() => resolve(result), delay);
  });
}

// ===== 送信関連の関数 =====

async function write(str, result) {
  return new Promise((resolve) => {
    port.write(`${str}`, function () {
      resolve(result);
    });
  });
}

const convert = (filename) => {
  return filename.split(".")[0];
};

const callerFun = async (mode) => {
  while (true) {
    const files = fs.readdirSync(__dirname + `/profile/${mode}/`);
    var list = ["system.yaml", "hardware.yaml"].concat(files.filter((file) => {
      return file.match(/.yaml$/) || file.match(/.maze$/);
    }));
    let col = 5;
    for (var i = 0; list.length;) {
      let str = "";
      if (list[i] === undefined) {
        break;
      }
      for (var j = 0; j < col; j++) {
        if (list[i + j] === undefined) {
          break;
        }
        str += `[${i + j}]: ${convert(list[i + j])}\t`;
      }
      i += col;
      console.log(str);
    }
    console.log(">");
    var str = fs.readFileSync("/dev/stdin").toString().trim();

    var idx = parseInt(str);
    if (str === "all") {
      console.log("all");
      for (const file of files) {
        if (file.match(/.yaml$/)) {
          let txt = fs.readFileSync(`${__dirname}/profile/${mode}/${file}`, {
            encoding: "utf-8",
          });
          let file_name = "";
          if (file.match(/.maze$/)) {
            file_name = "maze.txt";
            console.log(txt);
          } else if (file.match(/.yaml$/)) {
            file_name = file.replace("yaml", mode);
          }
          if (file === "maze.yaml") {
            // Skip maze.yaml
          } else {
            var saveData = yaml.load(txt);
            var str = `${file_name}@${JSON.stringify(saveData)}`;
            write(str);
            await sleep2(800);
            console.log(`${file}, ${file_name}: finish!!`);
          }
        }
      }
      for (const file of ["system.yaml", "hardware.yaml"]) {
        let txt = fs.readFileSync(`${__dirname}/profile/${file}`, {
          encoding: "utf-8",
        });

        let file_name = "";
        if (file.match(/.maze$/)) {
          file_name = "maze.txt";
          console.log(txt);
        } else if (file.match(/.yaml$/)) {
          file_name = file.replace("yaml", "txt");
        }

        var saveData = yaml.load(txt);
        var str = `${file_name}@${JSON.stringify(saveData)}`;
        write(str);
        await sleep2(800);
        console.log(`${file}, ${file_name}: finish!!`);
      }
    } else {
      if (idx > list.length) {
        console.log("out of index");
        continue;
      }
      if (idx === 0 || idx === 1) {
        let file = "system.yaml";
        if (idx === 1) {
          file = "hardware.yaml";
        }
        let txt = fs.readFileSync(`${__dirname}/profile/${list[idx]}`, {
          encoding: "utf-8",
        });
        var file_name = file.replace("yaml", "txt");
        var saveData = yaml.load(txt);
        var str = `${file_name}@${JSON.stringify(saveData)}`;
        write(str);
        await sleep2(800);
        console.log(`${file}, ${file_name}: finish!!`);
      } else {
        let txt = fs.readFileSync(`${__dirname}/profile/${mode}/${list[idx]}`, {
          encoding: "utf-8",
        });

        let file_name = "";
        if (list[idx].match(/.maze$/)) {
          file_name = `maze.${mode}`;
        } else if (list[idx].match(/.yaml$/)) {
          file_name = list[idx].replace("yaml", mode);
        }

        console.log(file_name);

        if (file_name === "maze.hf") {
          file_name = "maze.txt";
          let maze_list = txt.split(",").map((e) => { return e.trim(); }).map((e) => { return (parseInt(e) | 0xf0); });
          let size = 16;
          if (maze_list.length > 300) {
            size = 32;
          }
          for (let y = 0; y < size; y++) {
            for (let x = 0; x < size; x++) {
              if (x >= y) {
                continue;
              }
              let idx = y * size + x;
              let idx2 = x * size + y;
              let tmp = maze_list[idx];
              maze_list[idx] = maze_list[idx2];
              maze_list[idx2] = tmp;
            }
          }
          txt = maze_list.join(",");
          var str = `${file_name}@${txt}`;
          write(str);
          await sleep2(800);
          console.log(`${file_name}: finish!!`);
        } else {
          var saveData = yaml.load(txt);
          var str = `${file_name}@${JSON.stringify(saveData)}`;
          write(str);
          await sleep2(600);
          console.log(`${list[idx]}, ${file_name}: finish!!`);
        }
      }
    }
  }
};

// ===== 受信関連の関数 =====

const switchLineMode = (obj) => {
  port.unpipe(parser);
  parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));
  parser.on("data", function (data) {
    console.log(data);
    if (obj.dump_to_csv_ready) {
      const d = data.split(":");
      console.log(d);
      if (d.length == 3) {
        const name = d[0];
        const type = d[1];
        const size = parseInt(d[2]);
        obj.data_struct.push({
          name: name,
          type: type,
          size: size
        });
      }
    } else if (obj.dump_to_map) {
      if (data.match(/^end___/)) {
        obj.dump_to_map = false;
        console.log(`${__dirname}/maze_logs/${obj.file_name}`);

        let maze_list = obj.record.split(",").map((e) => { return e.trim(); }).map((e) => { return (parseInt(e)); });
        let size = 16;
        if (maze_list.length > 300) {
          size = 32;
        }
        for (let y = 0; y < size; y++) {
          for (let x = 0; x < size; x++) {
            if (x >= y) {
              continue;
            }
            let idx = y * size + x;
            let idx2 = x * size + y;
            let tmp = maze_list[idx];
            maze_list[idx] = maze_list[idx2];
            maze_list[idx2] = tmp;
          }
        }

        fs.writeFileSync(`${__dirname}/maze_logs/${obj.file_name}`, `${maze_list.join(",")}`, {
          flag: "w+",
        });
      }
      if (obj.dump_to_map) {
        obj.record += `${data}\n`;
      }
    }

    if (data.match(/^ready___/)) {
      obj.dump_to_csv_ready = true;
      obj.file_name = getNowYMD();
      obj.record = "";
      obj.data_struct = [];
      obj.byte_size = parseInt(data.split(":")[1]);

      console.log(obj);
    }

    if (data.match(/^start/)) {
      obj.file_name = getNowYMD();
      obj.record = "";
      switchToBinaryMode(obj);
    }

    if (data.match(/^map___/)) {
      obj.dump_to_map = true;
      obj.file_name = getNowYMD_maze();
      obj.record = "";
      console.log(obj);
    }
  });
};

let LOG_STRUCT_SIZE = 12;

const switchToBinaryMode = (obj) => {
  const dataSize = 48;
  const dataSize2 = obj.data_struct.reduce((prev, cur) => {
    return prev + cur.size;
  }, 0);
  console.log('size1: ', dataSize, 'bytes');
  console.log('size2: ', dataSize2, 'bytes');

  LOG_STRUCT_SIZE = dataSize2 / (4 * 12);

  binaryMode = true;

  port.unpipe(parser);
  parser = port.pipe(new ByteLengthParser({ length: dataSize }));

  const header = obj.data_struct.map((data) => {
    return data.name;
  }).join(',');
  obj.record += `${header}\n`;
  console.log('header:', header);
  let cnt = 0;
  let record = [];
  let finish = false;
  let index = 0;
  let last_index = 0;
  let last_recived = new Date().getTime();
  let now = new Date().getTime();
  let force_save = false;

  let last_idx = 0;

  let interval = setInterval(() => {
    console.log("force save");
    if ((now - last_recived > 1000) || force_save) {
      console.log('timeout');
      fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
        flag: "w+",
      });
      fs.copyFileSync(
        `${__dirname}/logs/${obj.file_name}`,
        `${__dirname}/logs/latest.csv`
      );
      finish = true;
      clearInterval(interval);
      switchLineMode({
        dump_to_csv_ready: false,
        dump_to_map: false,
        data_struct: [],
        file_name: getNowYMD(),
        record: "",
      });
    }
    now = new Date().getTime();
  }, 1000);

  parser.on('data', (binaryData) => {
    let offset = 0;
    cnt++;
    index++;

    let bind = false;
    const start_idx = (cnt - 1) * 12;
    const end_idx = start_idx + 12;

    for (let i = start_idx; i < end_idx; i++) {
      const data = obj.data_struct[i];
      if (data === undefined) {
        continue;
      }
      const tmp_data = offset;
      switch (data.type) {
        case 'float':
          record.push(binaryData.readFloatLE(tmp_data));
          offset += data.size;
          bind = true;
          break;
        case 'int':
          record.push(binaryData.readInt32LE(tmp_data));
          offset += data.size;
          bind = true;
          break;
        case 'short':
          record.push(binaryData.readInt16LE(tmp_data));
          offset += data.size;
          bind = true;
          break;
        default:
          throw new Error(`Unsupported data type: ${data.type}`);
      }
      if (data.name === 'index') {
        const idx = parseInt(binaryData.readInt32LE(tmp_data));
        console.log(last_idx, idx);
        if (idx < last_idx) {
          force_save = true;
        }
        last_idx = idx;
      }
    }
    if (bind) {
      last_recived = new Date().getTime();
    }

    if (cnt === LOG_STRUCT_SIZE) {
      cnt = 0;
      if (index > 10 && record[0] <= 0 && !finish) {
        clearInterval(interval);
        fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
          flag: "w+",
        });
        fs.copyFileSync(
          `${__dirname}/logs/${obj.file_name}`,
          `${__dirname}/logs/latest.csv`
        );
        finish = true;
        console.log("end");
        clearInterval(interval);
        switchLineMode({
          dump_to_csv_ready: false,
          dump_to_map: false,
          data_struct: [],
          file_name: getNowYMD(),
          record: "",
        });
      } else if (!finish) {
        let valid = obj.data_struct.every((data, i) => {
          let res = true;
          if (data.name === "index") {
            if (record[i] < 0 || record[i] > 100000)
              res = false;
            if (Math.abs(record[0] - last_index) > 100)
              res = false;
          }
          if (data.name.match(/_pid_/) !== null)
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name.match(/ff_/) !== null)
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name === "x")
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name === "y")
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name === "battery")
            if (record[i] > 15 || record[i] < 0)
              res = false;
          if (data.name === "alpha")
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name === "timestamp")
            if (record[i] > 10000 || record[i] < 0)
              res = false;
          if (data.name === "m_pid_i_v")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          if (data.name === "dist")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "dideal_ang")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "v_c")
            if (record[i] > 10000 || record[i] < -10000)
              res = false;
          if (data.name === "right45_2_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "right45_3_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_2_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_3_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "ff_duty_front")
            if (record[i] > 100 || record[i] < -100)
              res = false;
          if (data.name === "odm_y")
            if (record[i] > 1000 || record[i] < -1000)
              res = false;
          if (data.name === "kim_theta")
            if (record[i] > 400 || record[i] < -400)
              res = false;
          if (data.name === "odm_theta")
            if (record[i] > 400 || record[i] < -400)
              res = false;
          if (data.name === "knym_v")
            if (record[i] > 20000 || record[i] < -20000)
              res = false;
          if (data.name === "knym_w")
            if (record[i] > 2000 || record[i] < -2000)
              res = false;
          if (data.name === "ang_i_bias")
            if (record[i] > 3 || record[i] < -3)
              res = false;
          if (data.name === "ang_i_bias_val")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "left45_2_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "left45_3_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_2_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_3_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "duty_roll")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          if (data.name === "duty_roll_before")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          return res;
        });
        if (valid) {
          last_index = record[0];
          const str = record.join(',');
          console.log(str);
          obj.record += `${str}\n`;
        }
        record = [];
      }
    }
  });
};

// ===== メイン初期化関数 =====

let ready = function (mode, enableTx) {
  console.log(comport);
  port = new SerialPort(
    {
      baudRate: 3000000,
      path: comport,
    },
    (e) => {
      if (e) {
        console.log("comport access denied");
      } else {
        console.log("connect");
        if (enableTx) {
          console.log("TX mode enabled - starting parameter sender");
          callerFun(mode);
        } else {
          console.log("RX mode enabled - starting data receiver");
        }
      }
    }
  );

  let obj = {
    dump_to_csv_ready: false,
    dump_to_map: false,
    data_struct: [],
    file_name: getNowYMD(),
    record: "",
  };

  switchLineMode(obj, parser);
};

// ===== エントリーポイント =====

const main = (argv) => {
  // Usage: node terminal.js [mode] [--tx]
  // mode: _hf (default) or other profile names
  // --tx: enable transmit mode
  let mode = "_hf";
  let enableTx = false;

  for (let i = 2; i < argv.length; i++) {
    if (argv[i] === "--tx") {
      enableTx = true;
    } else if (!argv[i].startsWith("--")) {
      mode = argv[i];
    }
  }

  console.log(`Mode: ${mode}`);
  console.log(`TX Mode: ${enableTx ? "enabled" : "disabled"}`);

  SerialPort.list().then(
    (ports) => {
      for (let i in ports) {
        const p = ports[i];
        console.log(p.path, p.serialNumber);
        if (
          p.path.match(/usbserial/) ||
          p.path.match(/COM/) ||
          p.path.match(/ttyUSB/) ||
          p.path.match(/ttyACM/)
        ) {
          if (p.serialNumber) {
            comport = p.path;
            console.log(`select: ${comport}`);
            ready(mode, enableTx);
            break;
          }
        }
      }
    },
    (err) => console.error(err)
  );
};

main(argv);
