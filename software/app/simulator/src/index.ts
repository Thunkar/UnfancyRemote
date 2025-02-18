import { WebSocketServer } from "ws";
import { pino } from "pino";
import { createServer } from "http";
import express, {
  type RequestHandler,
  type Request,
  type Response,
} from "express";
import { pinoHttp } from "pino-http";

const THROTTLE_MIN_MV = 1500;
const THROTTLE_MAX_MV = 2100;
let throttleDirection = 5;

const MIN_REMOTE_VOLTAGE = 3.0;
const MAX_REMOTE_VOLTAGE = 4.2;

let cellN = 12;

const MIN_BOARD_VOLTAGE = 3.0 * cellN;
const MAX_BOARD_VOLTAGE = 4.2 * cellN;

let throttle1Raw = 1500;
let throttle2Raw = 1500;
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
  throttle1Raw += throttleDirection;
  throttle2Raw -= throttleDirection;
  throttleDirection =
    throttle1Raw >= THROTTLE_MAX_MV || throttle1Raw <= THROTTLE_MIN_MV
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
      addNoise(boardVoltage, 0.8, 0.1 * cellN),
      MIN_BOARD_VOLTAGE,
      MAX_BOARD_VOLTAGE
    )
  );

  return [
    channel,
    txIdentity,
    remoteVoltage,
    cellN,
    boardVoltage,
    throttle1Raw,
    throttle2Raw,
    calBrake,
    calAcc,
    centerAcc,
    centerBrake,
    inverted,
    isDual,
  ];
}

async function main() {
  const logger = pino({
    transport: {
      target: "pino-pretty",
    },
  });

  const app = express();
  app.use(pinoHttp({ logger }));
  app.get(
    "/settings",
    (
      req: Request<
        any,
        any,
        {
          param: "channel" | "txIdentity" | "cellN" | "isDual";
          value: number;
        }
      >,
      res: Response
    ) => {
      const { param, value } = req.query;
      logger.info(`${param}:${value}`);
      res.status(200).send("Ok");
    }
  );

  const server = createServer(app);
  const wss = new WebSocketServer({ server });

  wss.on("connection", (ws) => {
    ws.on("error", logger.error);

    ws.on("message", function message(data) {
      console.log("received: %s", data);
    });
  });

  await server.listen(8080);

  logger.info("Server listening on port 8080");

  setInterval(() => {
    const newState = computeState().join(",");
    wss.clients.forEach((client) => {
      client.send(newState);
    });
  }, 100);
}

main();
