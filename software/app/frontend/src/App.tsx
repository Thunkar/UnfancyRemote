import { css } from "@emotion/react";
import { Box, Tab, Typography } from "@mui/material";
import TabContext from "@mui/lab/TabContext";
import TabList from "@mui/lab/TabList";
import TabPanel from "@mui/lab/TabPanel";
import { slant, useAsciiText } from "react-ascii-text";
import { colors } from "./main";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Battery } from "./components/battery";
import { memo, useEffect, useState } from "react";
import { Throttle } from "./components/throttle";
import { RF } from "./components/rf";
import { Settings } from "./components/settings";

const MAX_BUFFER_SIZE = 100;

const container = css({
  display: "flex",
  flexDirection: "column",
  width: "100%",
  height: "100%",
  alignItems: "center",
});

const MemoizedSettings = memo(Settings);

function App() {
  const asciiTextRef = useAsciiText({
    animationCharacters: "▒░█",
    animationCharacterSpacing: 1,
    animationDelay: 0,
    animationDirection: "down",
    animationInterval: 0,
    animationLoop: false,
    animationSpeed: 40,
    fadeInOnly: true,
    font: slant,
    text: "!Fancy",
  });

  // Using a ref callback to bridge the type mismatch.
  const refCallback = (element: HTMLPreElement | null) => {
    if (asciiTextRef) {
      // Directly manipulate the `.current` property only if it's not `undefined`.
      asciiTextRef.current = element ?? undefined; // Convert `null` to `undefined`.
    }
  };

  const { lastMessage, readyState } = useWebSocket(
    import.meta.env.VITE_WS_URL ?? `ws://${window.location.hostname}`,
    {
      reconnectAttempts: 10,
      reconnectInterval: 3000,
    }
  );

  const connectionStatus = {
    [ReadyState.CONNECTING]: "Connecting",
    [ReadyState.OPEN]: "Open",
    [ReadyState.CLOSING]: "Closing",
    [ReadyState.CLOSED]: "Closed",
    [ReadyState.UNINSTANTIATED]: "Uninstantiated",
  }[readyState];

  const [remoteVoltage, setRemoteVoltage] = useState<number>(-1);
  const [cellN, setCellN] = useState<number>(-1);
  const [boardVoltage, setBoardVoltage] = useState<number>(-1);
  const [throttle1Buffer, setThrottle1Buffer] = useState<number[]>([]);
  const [throttle2Buffer, setThrottle2Buffer] = useState<number[]>([]);
  const [channel, setChannel] = useState<number>(-1);
  const [txIdentity, setTxIdentity] = useState<number>(-1);

  useEffect(() => {
    const data = lastMessage?.data.split(",") ?? [];

    const [
      channelRaw,
      txIdentityRaw,
      remoteVoltageRaw,
      cellNRaw,
      boardVoltageRaw,
      throttle1Raw,
      throttle2Raw,
      _calBrake,
      _calAcc,
      _centerAcc,
      _centerBrake,
      _inverted,
      isDual,
    ] = data;

    if (throttle1Raw !== undefined) {
      let newBuffer = [...throttle1Buffer, throttle1Raw];
      newBuffer =
        newBuffer.length >= MAX_BUFFER_SIZE
          ? newBuffer.slice(1, MAX_BUFFER_SIZE)
          : newBuffer;
      setThrottle1Buffer(newBuffer);
    }
    if (throttle2Raw !== undefined && isDual === "1") {
      let newBuffer = [...throttle2Buffer, throttle2Raw];
      newBuffer =
        newBuffer.length >= MAX_BUFFER_SIZE
          ? newBuffer.slice(1, MAX_BUFFER_SIZE)
          : newBuffer;
      setThrottle2Buffer(newBuffer);
    }
    if (cellNRaw !== undefined && parseInt(cellNRaw) !== cellN) {
      setCellN(parseInt(cellNRaw));
    }
    if (channelRaw !== undefined && parseInt(channelRaw) !== channel) {
      setChannel(parseInt(channelRaw));
    }
    if (txIdentityRaw !== undefined && parseInt(txIdentityRaw) !== txIdentity) {
      setTxIdentity(parseInt(txIdentityRaw));
    }
    setBoardVoltage(
      boardVoltageRaw !== undefined ? parseFloat(boardVoltageRaw) : -1
    );
    setRemoteVoltage(
      remoteVoltageRaw !== undefined ? parseFloat(remoteVoltageRaw) : -1
    );
  }, [lastMessage]);

  const [tab, setTab] = useState("0");

  return (
    <div css={container}>
      <pre css={{ color: colors.primary }} ref={refCallback}></pre>
      <Typography variant="subtitle1">
        Connection status: {connectionStatus}
      </Typography>
      <Box sx={{ width: "100%" }}>
        <TabContext value={tab}>
          <Box sx={{ borderBottom: 1, borderColor: "divider" }}>
            <TabList
              onChange={(_event, tab) => setTab(tab)}
              aria-label="lab API tabs example"
            >
              <Tab label="Telemetry" value="0" />
              <Tab label="Settings" value="1" />
              <Tab label="Calibration" value="2" />
            </TabList>
          </Box>
          <TabPanel sx={{ padding: "0.1rem" }} value="0">
            <Battery title="Remote" cells={1} voltage={remoteVoltage}></Battery>
            <Battery
              title="Board"
              cells={cellN}
              voltage={boardVoltage}
            ></Battery>
            <Throttle
              throttle1Values={throttle1Buffer}
              throttle2Values={throttle2Buffer}
            ></Throttle>
            <RF channel={channel} txIdentity={txIdentity}></RF>
          </TabPanel>
          <TabPanel sx={{ padding: "0.1rem" }} value="1">
            <MemoizedSettings
              cellN={cellN}
              channel={channel}
              txIdentity={txIdentity}
            />
          </TabPanel>
          <TabPanel sx={{ padding: "0.1rem" }} value="2"></TabPanel>
        </TabContext>
      </Box>
    </div>
  );
}

export default App;
