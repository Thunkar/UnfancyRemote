import {
  BoardType,
  Calibration,
  DataContext,
  Settings,
} from "../utils/context";
import { useEffect, useState, type ReactNode } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";
import {
  loadCalibration,
  loadSettings,
  saveCalibration,
  saveSettings,
} from "../utils/requests";

const MAX_BUFFER_SIZE = 100;

export const DataContextContainer = function ({
  children,
}: {
  children: ReactNode;
}) {
  const [boardType] = useState<BoardType>(
    import.meta.env.VITE_BOARD_TYPE === "TX" ? BoardType.TX : BoardType.RX
  );
  const [remoteVoltage, setRemoteVoltage] = useState<number>(-1);
  const [cellN, setCellN] = useState<number>(-1);
  const [boardVoltage, setBoardVoltage] = useState<number>(-1);
  const [throttle1Buffer, setThrottle1Buffer] = useState<number[]>([]);
  const [throttle2Buffer, setThrottle2Buffer] = useState<number[]>([]);
  const [encodedThrottle, setEncodedThrottle] = useState<number>(0);
  const [throttle1, setThrottle1] = useState<number>(0);
  const [throttle2, setThrottle2] = useState<number>(0);
  const [channel, setChannel] = useState<number>(-1);
  const [RSSI, setRSSI] = useState<number>(0);
  const [SNR, setSNR] = useState<number>(0);
  const [packetsPerSecond, setPacketsPerSecond] = useState<number>(0);
  const [TMPacketsPerSecond, setTMPacketsPerSecond] = useState<number>(0);
  const [minPacketTimeUs, setMinPacketTimeUs] = useState<number>(0);
  const [meanPacketTimeUs, setMeanPacketTimeUs] = useState<number>(0);
  const [maxPacketTimeUs, setMaxPacketTimeUs] = useState<number>(0);
  const [identity, setidentity] = useState<number>(-1);
  const [isDual, setIsDual] = useState<boolean>(false);
  const [calBrake, setCalBrake] = useState<number>(0);
  const [calAcc, setCalAcc] = useState<number>(0);
  const [centerAcc, setCenterAcc] = useState<number>(0);
  const [centerBrake, setCenterBrake] = useState<number>(0);
  const [inverted, setInverted] = useState<boolean>(false);
  const [websocketStatus, setWebsocketStatus] =
    useState<string>("Uninstantiated");
  const [tab, setTab] = useState<string>("0");

  const [connectWebSocket, setConnectWebSocket] = useState<boolean>(false);

  const { lastMessage, readyState } = useWebSocket(
    import.meta.env.VITE_WS_URL ?? `ws://${window.location.hostname}`,
    {
      share: true,
      retryOnError: true,
      reconnectAttempts: 10e5,
      reconnectInterval: 1000,
      shouldReconnect: () => true,
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

    let remoteVoltageRaw;
    let boardVoltageRaw;
    let throttle1Raw;
    let throttle2Raw;
    let encodedThrottle;
    let SNR;
    let RSSI;
    let packetsPerSecond;
    let TMPacketsPerSecond;
    let minPacketTimeUs;
    let meanPacketTimeUs;
    let maxPacketTimeUs;

    if (boardType === BoardType.TX) {
      remoteVoltageRaw = data[0];
      boardVoltageRaw = data[1];
      throttle1Raw = data[2];
      throttle2Raw = data[3];
      encodedThrottle = data[4];
    } else {
      boardVoltageRaw = data[0];
      encodedThrottle = data[1];
      RSSI = data[2];
      SNR = data[3];
      packetsPerSecond = data[4];
      TMPacketsPerSecond = data[5];
      minPacketTimeUs = data[6];
      meanPacketTimeUs = data[7];
      maxPacketTimeUs = data[8];
    }

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

    setEncodedThrottle(
      encodedThrottle !== undefined ? parseInt(encodedThrottle) : 0
    );

    setBoardVoltage(
      boardVoltageRaw !== undefined ? parseFloat(boardVoltageRaw) : -1
    );
    setRemoteVoltage(
      remoteVoltageRaw !== undefined ? parseFloat(remoteVoltageRaw) : -1
    );

    setSNR(SNR !== undefined ? parseInt(SNR) : 0);
    setRSSI(RSSI !== undefined ? parseInt(RSSI) : 0);

    setPacketsPerSecond(
      packetsPerSecond !== undefined ? parseInt(packetsPerSecond) : 0
    );
    setTMPacketsPerSecond(
      TMPacketsPerSecond !== undefined ? TMPacketsPerSecond : 0
    );

    setMinPacketTimeUs(
      minPacketTimeUs !== undefined ? parseInt(minPacketTimeUs) : 0
    );
    setMeanPacketTimeUs(
      meanPacketTimeUs !== undefined ? parseInt(meanPacketTimeUs) : 0
    );
    setMaxPacketTimeUs(
      maxPacketTimeUs !== undefined ? parseInt(maxPacketTimeUs) : 0
    );
  }, [lastMessage, isDual]);

  const reloadSettings = async () => {
    const { cellN, identity, channel, isDual } = await loadSettings();
    setCellN(cellN);
    setChannel(channel);
    setidentity(identity);
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
    if (boardType === BoardType.TX) {
      reloadCalibration();
    }
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
    boardType,
    encodedThrottle,
    remoteVoltage,
    cellN,
    boardVoltage,
    throttle1,
    throttle2,
    throttle1Buffer,
    throttle2Buffer,
    channel,
    identity,
    packetsPerSecond,
    TMPacketsPerSecond,
    minPacketTimeUs,
    meanPacketTimeUs,
    maxPacketTimeUs,
    SNR,
    RSSI,
    isDual,
    calBrake,
    calAcc,
    centerAcc,
    centerBrake,
    inverted,
    websocketStatus,
    tab,
    setTab,
    storeSettings,
    storeCalibration,
  };

  return (
    <DataContext.Provider value={initialData}>{children}</DataContext.Provider>
  );
};
