var fs = require("fs");
const yaml = require("js-yaml");
const { decode } = require("punycode");
let SerialPort = require("serialport");
const Readline = SerialPort.parsers.Readline;
const ByteLength = SerialPort.parsers.ByteLength;

let comport;
let port;

let parser;
let binaryMode = false;


let ready = function () {
  port = new SerialPort(
    comport,
    {
      baudRate: 3000000,
      // baudRate: 115200,
    },
    (e) => {
      if (e) {
        console.log("comport access dinied");
      } else {
        console.log("connect");
      }
    }
  );
  // switchToBinaryMode(336);

  let obj = {
    dump_to_csv_ready: false,
    dump_to_map: false,
    data_struct: [],
    file_name: getNowYMD(),
    record: "",
  };

  switchLineMode(obj, parser);
};

const getNowYMD = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.csv`;
}
const getNowYMD_maze = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.maze`;
}

const switchLineMode = (obj) => {
  port.unpipe(parser);
  parser = port.pipe(new Readline({ delimiter: "\r\n" }));
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
            // skip excahnged point
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

      // console.log(obj);
      switchToBinaryMode(obj);
    }

    if (data.match(/^map___/)) {
      obj.dump_to_map = true;
      obj.file_name = getNowYMD_maze();
      obj.record = "";
      console.log(obj);
    }
  });
}

let LOG_STRUCT_SIZE = 12; // 12 bytes per record

const switchToBinaryMode = (obj) => {
  const dataSize = 48;//obj.byte_size
  const dataSize2 = obj.data_struct.reduce((prev, cur) => {
    return prev + cur.size;
  }, 0);
  console.log('size1: ', dataSize, 'bytes');
  console.log('size2: ', dataSize2, 'bytes');

  LOG_STRUCT_SIZE = dataSize2 / (4 * 12); // 4 * 12 bytes per struct

  binaryMode = true;

  // パイプラインをクリア
  port.unpipe(parser);

  // ByteLength パーサに切り替え
  parser = port.pipe(new ByteLength({ length: dataSize }));

  const header = obj.data_struct.map((data) => {
    return data.name
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

  let interval = setInterval(() => {
    console.log("force save");
    if (now - last_recived > 1000) {
      console.log('timeout');
      fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
        flag: "w+",
      });
      fs.copyFileSync(
        `${__dirname}/logs/${obj.file_name}`,
        `${__dirname}/logs/latest.csv`
      );
      finish = true;
      // port.unpipe(parser);
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
    last_recived = new Date().getTime();
    const start_idx = (cnt - 1) * 12;
    const end_idx = start_idx + 12;

    for (let i = start_idx; i < end_idx; i++) {
      const data = obj.data_struct[i];
      if (data === undefined) {
        continue;
      }
      switch (data.type) {
        case 'float':
          record.push(binaryData.readFloatLE(offset));
          offset += data.size;
          break;
        case 'int':
          record.push(binaryData.readInt32LE(offset));
          offset += data.size;
          break;
        default:
          throw new Error(`Unsupported data type: ${data.type}`);
      }
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
          if (data.name === "dist")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "dideal_ang")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "v_c")
            if (record[i] > 10000 || record[i] < -10000)
              res = false;
          if (data.name === "righ45_2_d")
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          return res;
        });
        if (valid) {
          last_index = record[0];
          const str = record.join(',');
          console.log(str)
          obj.record += `${str}\n`;
        }
        record = [];
      }
    }
  });
}

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
          ready();
          break;
        }
      }
    }
  },
  (err) => console.error(err)
);
