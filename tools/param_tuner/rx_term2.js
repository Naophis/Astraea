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
      // baudRate: 3000000,
      baudRate: 115200,
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
  parser = port.pipe(new Readline({ delimiter: "\r\n" }));
  function getNowYMD() {
    var dt = new Date();
    var y = dt.getFullYear();
    var m = ("00" + (dt.getMonth() + 1)).slice(-2);
    var d = ("00" + dt.getDate()).slice(-2);
    var h = ("00" + dt.getHours()).slice(-2);
    var M = ("00" + dt.getMinutes()).slice(-2);
    var s = ("00" + dt.getSeconds()).slice(-2);
    return `${y}${m}${d}_${h}${M}_${s}.csv`;
  }
  function getNowYMD_maze() {
    var dt = new Date();
    var y = dt.getFullYear();
    var m = ("00" + (dt.getMonth() + 1)).slice(-2);
    var d = ("00" + dt.getDate()).slice(-2);
    var h = ("00" + dt.getHours()).slice(-2);
    var M = ("00" + dt.getMinutes()).slice(-2);
    var s = ("00" + dt.getSeconds()).slice(-2);
    return `${y}${m}${d}_${h}${M}_${s}.maze`;
  }
  let obj = {
    dump_to_csv: false,
    dump_to_csv_ready: false,
    dump_to_map: false,
    data_struct: [],
    file_name: getNowYMD(),
    record: "",
  };

  parser.on("data", function (data) {
    console.log(data, obj.dump_to_csv_ready);
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
    } else if (obj.dump_to_csv) {
      if (data.match(/^end___/)) {
        obj.dump_to_csv = false;
        console.log(`${__dirname}/logs/${obj.file_name}`);
        fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
          flag: "w+",
        });
        fs.copyFileSync(
          `${__dirname}/logs/${obj.file_name}`,
          `${__dirname}/logs/latest.csv`
        );
        console.log('Switching back to Readline mode');
        binaryMode = false;
        // パイプラインをクリア
        port.unpipe(parser);

        // Readline パーサに戻す
        parser = port.pipe(new Readline({ delimiter: "\r\n" }));
      }
      if (obj.dump_to_csv) {
        obj.record += `${data}\n`;
      }
    }
    if (obj.dump_to_map) {
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
      obj.dump_to_csv = true;
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
};

const switchToBinaryMode = (obj) => {
  const dataSize = obj.byte_size
  const dataSize2 = obj.data_struct.reduce((prev, cur) => {
    return prev + cur.size;
  }, 0);
  console.log('size1: ', dataSize, 'bytes');
  console.log('size2: ', dataSize2, 'bytes');
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

  parser.on('data', (binaryData) => {
    let offset = 0;
    // console.log(binaryData)
    const result = binaryData.toString('hex').match(/.{1,4}/g).map((hex) => {
      const upper = hex.slice(0, 2);
      const lower = hex.slice(2, 4);
      return parseInt(lower + upper, 16);
    });
    // for (let i = 0; i < result.length; i++) {
    //   console.log(result[i]);
    // }
    const decodedData = obj.data_struct.map((data) => {
      let decodedValue;
      switch (data.type) {
        case 'float':
          decodedValue = binaryData.readFloatLE(offset);
          offset += data.size;
          break;
        case 'int':
          decodedValue = binaryData.readInt32LE(offset);
          offset += data.size;
          break;
        default:
          throw new Error(`Unsupported data type: ${data.type}`);
      }
      return decodedValue;
    });
    const str = decodedData.join(',');
    if (decodedData[0] === -1) {
      fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
        flag: "w+",
      });
      fs.copyFileSync(
        `${__dirname}/logs/${obj.file_name}`,
        `${__dirname}/logs/latest.csv`
      );
      console.log("end");
    } else {
      console.log(str)
      obj.record += `${str}\n`;
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
