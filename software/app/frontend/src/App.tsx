import { css } from "@emotion/react";
import { Typography } from "@mui/material";
import { slant, useAsciiText } from "react-ascii-text";
import { colors } from "./main";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Battery } from "./components/battery";

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
    animationDelay: 2000,
    animationDirection: "down",
    animationInterval: 100,
    animationLoop: false,
    animationSpeed: 30,
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
  ] = data;

  return (
    <div css={container}>
      <pre css={{ color: colors.primary }} ref={refCallback}></pre>
      <Typography variant="subtitle1">
        Connection status: {connectionStatus}
      </Typography>
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
    </div>
  );
}

export default App;
