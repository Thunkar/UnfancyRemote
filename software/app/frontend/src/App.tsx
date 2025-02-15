import { css } from "@emotion/react";
import { Box, Tab, Tabs, Typography } from "@mui/material";
import TabContext from "@mui/lab/TabContext";
import TabList from "@mui/lab/TabList";
import TabPanel from "@mui/lab/TabPanel";
import { slant, useAsciiText } from "react-ascii-text";
import { colors } from "./main";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Battery } from "./components/battery";
import { useState } from "react";

const container = css({
  display: "flex",
  flexDirection: "column",
  width: "100%",
  height: "100%",
  alignItems: "center",
});

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
    import.meta.env.VITE_WS_URL ?? `ws://${window.location.hostname}/ws`
  );

  const connectionStatus = {
    [ReadyState.CONNECTING]: "Connecting",
    [ReadyState.OPEN]: "Open",
    [ReadyState.CLOSING]: "Closing",
    [ReadyState.CLOSED]: "Closed",
    [ReadyState.UNINSTANTIATED]: "Uninstantiated",
  }[readyState];

  const data = lastMessage?.data.split(",") ?? [];

  const [
    _channel,
    _txIdentity,
    remoteVoltage,
    cell_n,
    boardVoltage,
    _throttleRaw,
    _calBrake,
    _calAcc,
    _centerAcc,
    _centerBrake,
    _inverted,
    _isDual,
  ] = data;

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
            </TabList>
          </Box>
          <TabPanel sx={{ padding: "0.1rem" }} value="0">
            <Battery
              title="Remote"
              cells={1}
              voltage={remoteVoltage ? parseFloat(remoteVoltage) : -1}
            ></Battery>
            <Battery
              title="Board"
              cells={cell_n}
              voltage={boardVoltage ? parseFloat(boardVoltage) : -1}
            ></Battery>
          </TabPanel>
        </TabContext>
      </Box>
    </div>
  );
}

export default App;
