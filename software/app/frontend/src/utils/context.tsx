import { createContext } from "react";

export enum BoardType {
  TX = "TX",
  RX = "RX",
}

export type Settings = {
  cellN: number;
  identity: number;
  channel: number;
  isDual: boolean;
};

export type Calibration = {
  calBrake: number;
  calAcc: number;
  centerAcc: number;
  centerBrake: number;
  inverted: boolean;
};

export const DataContext = createContext<{
  boardType: BoardType;
  remoteVoltage: number;
  cellN: number;
  boardVoltage: number;
  throttle1: number;
  throttle2: number;
  encodedThrottle: number;
  throttle1Buffer: number[];
  throttle2Buffer: number[];
  packetsPerSecond: number;
  TMPacketsPerSecond: number;
  minPacketTimeUs: number;
  meanPacketTimeUs: number;
  maxPacketTimeUs: number;
  channel: number;
  identity: number;
  RSSI: number;
  SNR: number;
  isDual: boolean;
  calBrake: number;
  calAcc: number;
  centerAcc: number;
  centerBrake: number;
  inverted: boolean;
  websocketStatus: string;
  tab: string;
  setTab: (tab: string) => void;
  storeSettings: (settings: Settings) => Promise<void>;
  storeCalibration: (Calibration: Calibration) => Promise<void>;
}>({
  boardType: BoardType.TX,
  remoteVoltage: -1,
  cellN: -1,
  boardVoltage: -1,
  throttle1: 0,
  throttle2: 0,
  encodedThrottle: 0,
  throttle1Buffer: [],
  throttle2Buffer: [],
  channel: -1,
  identity: -1,
  RSSI: -100,
  SNR: -15,
  packetsPerSecond: 0,
  TMPacketsPerSecond: 0,
  minPacketTimeUs: 0,
  meanPacketTimeUs: 0,
  maxPacketTimeUs: 0,
  isDual: false,
  websocketStatus: "Uninstantiated",
  calBrake: 0,
  calAcc: 0,
  centerAcc: 0,
  centerBrake: 0,
  inverted: false,
  tab: "0",
  setTab: () => {},
  storeSettings: () => Promise.resolve(),
  storeCalibration: () => Promise.resolve(),
});
