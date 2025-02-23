import { createContext, useEffect, useState, type ReactNode } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";
import {
  loadCalibration,
  loadSettings,
  saveCalibration,
  saveSettings,
} from "./requests";

const MAX_BUFFER_SIZE = 100;

export type Settings = {
  cellN: number;
  txIdentity: number;
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
  remoteVoltage: number;
  cellN: number;
  boardVoltage: number;
  throttle1: number;
  throttle2: number;
  encodedThrottle: number;
  throttle1Buffer: number[];
  throttle2Buffer: number[];
  channel: number;
  txIdentity: number;
  isDual: boolean;
  calBrake: number;
  calAcc: number;
  centerAcc: number;
  centerBrake: number;
  inverted: boolean;
  websocketStatus: string;
  storeSettings: (settings: Settings) => Promise<void>;
  storeCalibration: (Calibration: Calibration) => Promise<void>;
}>({
  remoteVoltage: -1,
  cellN: -1,
  boardVoltage: -1,
  throttle1: 0,
  throttle2: 0,
  encodedThrottle: 0,
  throttle1Buffer: [],
  throttle2Buffer: [],
  channel: -1,
  txIdentity: -1,
  isDual: false,
  websocketStatus: "Uninstantiated",
  calBrake: 0,
  calAcc: 0,
  centerAcc: 0,
  centerBrake: 0,
  inverted: false,
  storeSettings: () => Promise.resolve(),
  storeCalibration: () => Promise.resolve(),
});

export const DataContextContainer = function ({
  children,
}: {
  children: ReactNode;
}) {
  const [remoteVoltage, setRemoteVoltage] = useState<number>(-1);
  const [cellN, setCellN] = useState<number>(-1);
  const [boardVoltage, setBoardVoltage] = useState<number>(-1);
  const [throttle1Buffer, setThrottle1Buffer] = useState<number[]>([]);
  const [throttle2Buffer, setThrottle2Buffer] = useState<number[]>([]);
  const [encodedThrottle, setEncodedThrottle] = useState<number>(0);
  const [throttle1, setThrottle1] = useState<number>(0);
  const [throttle2, setThrottle2] = useState<number>(0);
  const [channel, setChannel] = useState<number>(-1);
  const [txIdentity, setTxIdentity] = useState<number>(-1);
  const [isDual, setIsDual] = useState<boolean>(false);
  const [calBrake, setCalBrake] = useState<number>(0);
  const [calAcc, setCalAcc] = useState<number>(0);
  const [centerAcc, setCenterAcc] = useState<number>(0);
  const [centerBrake, setCenterBrake] = useState<number>(0);
  const [inverted, setInverted] = useState<boolean>(false);
  const [websocketStatus, setWebsocketStatus] =
    useState<string>("Uninstantiated");

  const [connectWebSocket, setConnectWebSocket] = useState<boolean>(false);

  const { lastMessage, readyState } = useWebSocket(
    import.meta.env.VITE_WS_URL ?? `ws://${window.location.hostname}`,
    {
      reconnectAttempts: 10,
      reconnectInterval: 3000,
    },
    connectWebSocket
  );

  useEffect(() => {
    const connectionStatus = {
      [ReadyState.CONNECTING]: "Connecting",
      [ReadyState.OPEN]: "Open",
      [ReadyState.CLOSING]: "Closing",
      [ReadyState.CLOSED]: "Closed",
      [ReadyState.UNINSTANTIATED]: "Uninstantiated",
    }[readyState];

    setWebsocketStatus(connectionStatus);
  }, [readyState]);

  useEffect(() => {
    const data = lastMessage?.data.split(",") ?? [];

    const [
      remoteVoltageRaw,
      boardVoltageRaw,
      throttle1Raw,
      throttle2Raw,
      encodedThrottle,
    ] = data;

    if (throttle1Raw !== undefined) {
      let newBuffer = [...throttle1Buffer, throttle1Raw];
      newBuffer =
        newBuffer.length >= MAX_BUFFER_SIZE
          ? newBuffer.slice(1, MAX_BUFFER_SIZE)
          : newBuffer;
      setThrottle1Buffer(newBuffer);
      setThrottle1(throttle1Raw);
    }
    if (throttle2Raw !== undefined) {
      let newBuffer = [...throttle2Buffer, isDual ? throttle2Raw : 0];
      newBuffer =
        newBuffer.length >= MAX_BUFFER_SIZE
          ? newBuffer.slice(1, MAX_BUFFER_SIZE)
          : newBuffer;
      setThrottle2Buffer(newBuffer);
      setThrottle2(throttle2Raw);
    }

    if (encodedThrottle !== undefined) {
      setEncodedThrottle(encodedThrottle);
    }

    setBoardVoltage(
      boardVoltageRaw !== undefined ? parseFloat(boardVoltageRaw) : -1
    );
    setRemoteVoltage(
      remoteVoltageRaw !== undefined ? parseFloat(remoteVoltageRaw) : -1
    );
  }, [lastMessage, isDual]);

  const reloadSettings = async () => {
    const { cellN, txIdentity, channel, isDual } = await loadSettings();
    setCellN(cellN);
    setChannel(channel);
    setTxIdentity(txIdentity);
    setIsDual(isDual);
    setConnectWebSocket(true);
  };

  const reloadCalibration = async () => {
    const { calBrake, calAcc, centerAcc, centerBrake, inverted } =
      await loadCalibration();
    setCalBrake(calBrake);
    setCalAcc(calAcc);
    setCenterAcc(centerAcc);
    setCenterBrake(centerBrake);
    setInverted(inverted);
  };

  useEffect(() => {
    reloadSettings();
    reloadCalibration();
  }, []);

  const storeSettings = async (settings: Settings) => {
    await saveSettings(settings);
    await reloadSettings();
  };

  const storeCalibration = async (calibration: Calibration) => {
    await saveCalibration(calibration);
    await reloadCalibration();
  };

  const initialData = {
    encodedThrottle,
    remoteVoltage,
    cellN,
    boardVoltage,
    throttle1,
    throttle2,
    throttle1Buffer,
    throttle2Buffer,
    channel,
    txIdentity,
    isDual,
    calBrake,
    calAcc,
    centerAcc,
    centerBrake,
    inverted,
    websocketStatus,
    storeSettings,
    storeCalibration,
  };

  return (
    <DataContext.Provider value={initialData}>{children}</DataContext.Provider>
  );
};
