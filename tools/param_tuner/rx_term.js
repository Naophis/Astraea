var fs = require("fs");
const yaml = require("js-yaml");
let SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");

let comport;
let port;

let parser;

function resolveAfter2Seconds(str) {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(str);
      console.log(str); // 10
    }, 500);
  });
}

async function f1(str) {
  var x = await resolveAfter2Seconds(str);
}
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
  parser = port.pipe(
    new Readline({
      delimiter: "\r\n",
    })
  );
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
    dump_to_map: false,
    file_name: getNowYMD(),
    record: "",
  };

  parser.on("data", function (data) {
    console.log(data);
    if (obj.dump_to_csv) {
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

    if (data.match(/^start___/)) {
      obj.dump_to_csv = true;
      obj.file_name = getNowYMD();
      obj.record = "";
      console.log(obj);
    }
    if (data.match(/^map___/)) {
      obj.dump_to_map = true;
      obj.file_name = getNowYMD_maze();
      obj.record = "";
      console.log(obj);
    }
  });
};

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
