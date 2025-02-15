import { WebSocketServer } from "ws";
import pino from "pino";

const wss = new WebSocketServer({ port: 8080 });
const logger = pino({
  transport: {
    target: "pino-pretty",
  },
});

const THROTTLE_MIN_MV = 1200;
const THROTTLE_MAX_MV = 2800;
let throttleDirection = 5;

const MIN_REMOTE_VOLTAGE = 3.0;
const MAX_REMOTE_VOLTAGE = 4.2;

let cell_n = 12;

const MIN_BOARD_VOLTAGE = 3.0 * cell_n;
const MAX_BOARD_VOLTAGE = 4.2 * cell_n;

let throttleRaw = 1500;
let channel = 15;
let txIdentity = 224;

let remoteVoltage = 3.8;
let boardVoltage = 3.8 * 12;

let calBrake = 0;
let calAcc = 0;
let centerAcc = 0;
let centerBrake = 0;
let inverted = 0;
let isDual = 0;

function constrain(value: number, min: number, max: number) {
  return Math.min(Math.max(value, min), max);
}

function addNoise(value: number, probability: number, delta: number) {
  const variance = Math.random() > 0.5 ? delta : -delta;
  return Math.random() > probability ? value + variance : value;
}

function limitDecimals(value: number, decimals = 2) {
  return parseFloat(value.toFixed(decimals));
}

function computeState() {
  throttleRaw += throttleDirection;
  throttleDirection =
    throttleRaw >= THROTTLE_MAX_MV || throttleRaw <= THROTTLE_MIN_MV
      ? -throttleDirection
      : throttleDirection;

  remoteVoltage = limitDecimals(
    constrain(
      addNoise(remoteVoltage, 0.8, 0.1),
      MIN_REMOTE_VOLTAGE,
      MAX_REMOTE_VOLTAGE
    )
  );
  boardVoltage = limitDecimals(
    constrain(
      addNoise(boardVoltage, 0.8, 0.1 * cell_n),
      MIN_BOARD_VOLTAGE,
      MAX_BOARD_VOLTAGE
    )
  );

  return [
    channel,
    txIdentity,
    remoteVoltage,
    cell_n,
    boardVoltage,
    throttleRaw,
    calBrake,
    calAcc,
    centerAcc,
    centerBrake,
    inverted,
    isDual,
  ];
}

wss.on("connection", (ws) => {
  ws.on("error", logger.error);

  ws.on("message", function message(data) {
    console.log("received: %s", data);
  });
});

setInterval(() => {
  const newState = computeState().join(",");
  logger.info(`New state: ${newState}`);
  wss.clients.forEach((client) => {
    client.send(newState);
  });
}, 100);

logger.info("Simulator ready");
