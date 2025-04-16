import { WebSocketServer } from "ws";
import { pino } from "pino";
import { createServer } from "http";
import express, { type Request, type Response, json } from "express";
import { pinoHttp } from "pino-http";
import cors from "cors";
import yargs from "yargs";
import { hideBin } from "yargs/helpers";

const argv = await yargs(hideBin(process.argv))
  .options({
    mode: { type: "string", default: "tx" },
  })
  .parse();

const simMode: "rx" | "tx" = argv.mode === "rx" ? "rx" : "tx";

const ENCODED_MAX = 4095;
const ENCODED_HALF = 2048;
const BRAKE_SENSITIVITY = 10;

const MIN_SNR = -25;
const MAX_SNR = 15;
const MIN_RSSI = -100;
const MAX_RSSI = 100;

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

type Config = {
  cellN: number;
  channel: number;
  identity: number;
  isDual?: number;
};

const config: Config = {
  cellN: 12,
  channel: 15,
  identity: 224,
  isDual: simMode === "tx" ? 1 : undefined,
};

let remoteVoltage = 3.8;
let boardVoltage = 3.8 * 12;

let snr = 0;
let rssi = 0;

type Calibration = {
  calBrake: number;
  calAcc: number;
  centerBrake: number;
  centerAcc: number;
  inverted: number;
}

let calAcc = THROTTLE_MAX_MV - 2;
let calBrake = config.isDual ? THROTTLE_MAX_MV - 2 : THROTTLE_MIN_MV + 2;
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

  if (config.isDual) {
    const throttle1 = constrain(
      throttle1Raw,
      Math.min(centerAcc, calAcc) + 1,
      Math.max(centerAcc, calAcc) - 1
    );
    const throttle2 = constrain(
      throttle2Raw,
      Math.min(centerBrake, calBrake) + 1,
      Math.max(centerBrake, calBrake) - 1
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
      Math.min(calBrake, calAcc) + 1,
      Math.max(calBrake, calAcc) - 1
    );
    const scaledValue =
      throttle1 > centerAcc
        ? mapRange(
            throttle1,
            centerAcc,
            Math.max(calBrake, calAcc),
            ENCODED_HALF + 1,
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

  snr = limitDecimals(constrain(addNoise(snr, 0.8, 1), MIN_SNR, MAX_SNR));

  rssi = limitDecimals(constrain(addNoise(rssi, 0.8, 1), MIN_RSSI, MAX_RSSI));

  return simMode === "tx"
    ? [remoteVoltage, boardVoltage, throttle1Raw, throttle2Raw, encodedThrottle]
    : [boardVoltage, encodedThrottle, rssi, snr, 9.37, 10.5];
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
    (req: Request<any, any, Config>, res: Response) => {
      const {
        cellN: newCellN,
        identity: newIdentity,
        channel: newChannel,
        isDual: newIsDual,
      } = req.body;
      cellN = newCellN;
      config.identity = newIdentity;
      config.channel = newChannel;
      config.isDual = newIsDual;
      res.status(200).send("Ok");
    }
  );

  app.get("/settings", (req: Request, res: Response<Config>) => {
    if (simMode === "rx") {
      delete config.isDual;
    }
    res.status(200).json(config);
  });

  app.post(
    "/calibration",
    (
      req: Request<
        any,
        any,
        Calibration
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

  app.get("/calibration", (req: Request, res: Response<Calibration>) => {
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

  logger.info("Server listening on port 8080. Mode is %s", simMode);

  setInterval(() => {
    const newState = computeState().join(",");
    wss.clients.forEach((client) => {
      client.send(newState);
    });
  }, 50);
}

main();
