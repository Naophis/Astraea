var fs = require("fs");
const yaml = require("js-yaml");
let SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");
const { argv } = require("process");
let comport;
let port;

const main = (argv) => {

  let txt = fs.readFileSync(`${__dirname}/profile/hf}/profiles.yaml`, {
    encoding: "utf-8",
  });
};
main(argv);