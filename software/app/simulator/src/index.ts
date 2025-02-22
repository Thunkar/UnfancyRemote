import { WebSocketServer } from "ws";
import { pino } from "pino";
import { createServer } from "http";
import express, { type Request, type Response, json } from "express";
import { pinoHttp } from "pino-http";
import cors from "cors";

const THROTTLE_MIN_MV = 1500;
const THROTTLE_MAX_MV = 2100;
let throttle1Direction = 5;
let throttle2Direction = 5;

const MIN_REMOTE_VOLTAGE = 3.0;
const MAX_REMOTE_VOLTAGE = 4.2;

let nCells = 12;

const MIN_BOARD_VOLTAGE = 3.0 * nCells;
const MAX_BOARD_VOLTAGE = 4.2 * nCells;

let throttle1Raw = THROTTLE_MIN_MV;
let throttle2Raw = THROTTLE_MAX_MV - 1;

let channel = 15;
let txIdentity = 224;
let isDual = 0;

let remoteVoltage = 3.8;
let boardVoltage = 3.8 * 12;

let calBrake = 0;
let calAcc = 0;
let centerAcc = 0;
let centerBrake = 0;
let inverted = 0;

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
  throttle1Raw += throttle1Direction;
  throttle2Raw += throttle2Direction;
  throttle1Direction =
    throttle1Raw >= THROTTLE_MAX_MV || throttle1Raw <= THROTTLE_MIN_MV
      ? -throttle1Direction
      : throttle1Direction;
  throttle2Direction =
    throttle2Raw >= THROTTLE_MAX_MV || throttle2Raw <= THROTTLE_MIN_MV
      ? -throttle2Direction
      : throttle2Direction;

  remoteVoltage = limitDecimals(
    constrain(
      addNoise(remoteVoltage, 0.8, 0.1),
      MIN_REMOTE_VOLTAGE,
      MAX_REMOTE_VOLTAGE
    )
  );
  boardVoltage = limitDecimals(
    constrain(
      addNoise(boardVoltage, 0.8, 0.1 * nCells),
      MIN_BOARD_VOLTAGE,
      MAX_BOARD_VOLTAGE
    )
  );

  return [remoteVoltage, boardVoltage, throttle1Raw, throttle2Raw];
}

async function main() {
  const logger = pino({
    transport: {
      target: "pino-pretty",
    },
  });

  const app = express();
  app.use(cors());
  app.use(json());
  app.use(
    pinoHttp({
      logger,
      serializers: {
        req(req) {
          return { body: req.raw.body };
        },
      },
    })
  );

  app.post(
    "/settings",
    (
      req: Request<
        any,
        any,
        {
          nCells: number;
          txIdentity: number;
          channel: number;
          isDual: number;
        }
      >,
      res: Response
    ) => {
      const {
        nCells: newNCells,
        txIdentity: newTxIdentity,
        channel: newChannel,
        isDual: newIsDual,
      } = req.body;
      nCells = newNCells;
      txIdentity = newTxIdentity;
      channel = newChannel;
      isDual = newIsDual;
      res.status(200).send("Ok");
    }
  );

  app.get("/settings", (req: Request, res: Response) => {
    res.status(200).json({
      nCells,
      txIdentity,
      channel,
      isDual,
    });
  });

  app.post(
    "/calibration",
    (
      req: Request<
        any,
        any,
        {
          calBrake: number;
          calAcc: number;
          centerBrake: number;
          centerAcc: number;
          inverted: number;
        }
      >,
      res: Response
    ) => {
      const {
        calBrake: newCalBrake,
        calAcc: newCalAcc,
        centerBrake: newCenterBrake,
        centerAcc: newCenterAcc,
        inverted: newInverted,
      } = req.body;
      calBrake = newCalBrake;
      calAcc = newCalAcc;
      centerBrake = newCenterBrake;
      centerAcc = newCenterAcc;
      inverted = newInverted;
      res.status(200).send("Ok");
    }
  );

  app.get("/calibration", (req: Request, res: Response) => {
    res.status(200).json({
      calBrake,
      calAcc,
      centerBrake,
      centerAcc,
      inverted,
    });
  });

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
