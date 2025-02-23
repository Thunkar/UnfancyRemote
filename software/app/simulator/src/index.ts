import { WebSocketServer } from "ws";
import { pino } from "pino";
import { createServer } from "http";
import express, { type Request, type Response, json } from "express";
import { pinoHttp } from "pino-http";
import cors from "cors";

const ENCODED_MAX = 65535;
const ENCODED_HALF = 32768;
const BRAKE_SENSITIVITY = 5;

const THROTTLE_MIN_MV = 1500;
const THROTTLE_MAX_MV = 2100;
let throttle1Direction = 5;
let throttle2Direction = 5;

const MIN_REMOTE_VOLTAGE = 3.0;
const MAX_REMOTE_VOLTAGE = 4.2;

let cellN = 12;

const MIN_BOARD_VOLTAGE = 3.0 * cellN;
const MAX_BOARD_VOLTAGE = 4.2 * cellN;

let throttle1Raw = THROTTLE_MIN_MV;
let throttle2Raw = THROTTLE_MAX_MV - 1;

let encodedThrottle = 0;

let channel = 15;
let txIdentity = 224;
let isDual = 1;

let remoteVoltage = 3.8;
let boardVoltage = 3.8 * 12;

let calAcc = THROTTLE_MAX_MV - 2;
let calBrake = THROTTLE_MAX_MV - 2;
let centerAcc = (THROTTLE_MAX_MV + THROTTLE_MIN_MV) / 2;
let centerBrake = (THROTTLE_MAX_MV + THROTTLE_MIN_MV) / 2;
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

function mapRange(
  value: number,
  fromLow: number,
  fromHigh: number,
  toLow: number,
  toHigh: number
) {
  return ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow) + toLow;
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
      addNoise(boardVoltage, 0.8, 0.1 * cellN),
      MIN_BOARD_VOLTAGE,
      MAX_BOARD_VOLTAGE
    )
  );

  if (isDual) {
    const throttle1 = constrain(
      throttle1Raw,
      Math.min(centerAcc, calAcc),
      Math.max(centerAcc, calAcc)
    );
    const throttle2 = constrain(
      throttle2Raw,
      Math.min(centerBrake, calBrake),
      Math.max(centerBrake, calBrake)
    );

    const isBraking = Math.abs(throttle2 - centerBrake) > BRAKE_SENSITIVITY;

    if (isBraking) {
      encodedThrottle =
        throttle2 > centerBrake
          ? ENCODED_HALF -
            mapRange(throttle2, centerBrake, calBrake, 0, ENCODED_HALF)
          : mapRange(throttle2, calBrake, centerBrake, 0, ENCODED_HALF);
    } else {
      encodedThrottle =
        throttle1 > centerAcc
          ? mapRange(throttle1, centerAcc, calAcc, ENCODED_HALF, ENCODED_MAX)
          : ENCODED_HALF -
            mapRange(
              throttle1,
              calAcc,
              centerAcc,
              ENCODED_HALF + 1,
              ENCODED_MAX
            );
    }
  } else {
    const throttle1 = constrain(
      throttle1Raw,
      Math.min(calBrake, calAcc),
      Math.max(calBrake, calAcc)
    );
    const scaledValue =
      throttle1 > centerAcc
        ? mapRange(
            throttle1,
            centerAcc,
            Math.max(calBrake, calAcc),
            ENCODED_HALF,
            ENCODED_MAX
          )
        : mapRange(
            throttle1,
            Math.min(calBrake, calAcc),
            centerAcc,
            0,
            ENCODED_HALF
          );
    encodedThrottle = inverted ? ENCODED_MAX - scaledValue : scaledValue;
  }

  return [
    remoteVoltage,
    boardVoltage,
    throttle1Raw,
    throttle2Raw,
    encodedThrottle,
  ];
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
          return {
            body: req.raw.body,
            url: req.raw.url,
            method: req.raw.method,
          };
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
          cellN: number;
          txIdentity: number;
          channel: number;
          isDual: number;
        }
      >,
      res: Response
    ) => {
      const {
        cellN: newCellN,
        txIdentity: newTxIdentity,
        channel: newChannel,
        isDual: newIsDual,
      } = req.body;
      cellN = newCellN;
      txIdentity = newTxIdentity;
      channel = newChannel;
      isDual = newIsDual;
      res.status(200).send("Ok");
    }
  );

  app.get("/settings", (req: Request, res: Response) => {
    res.status(200).json({
      cellN,
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
