import TabContext from "@mui/lab/TabContext";
import TabList from "@mui/lab/TabList";
import TabPanel from "@mui/lab/TabPanel";
import Tab from "@mui/material/Tab";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import { smallSlant, useAsciiText } from "react-ascii-text";
import { colors } from "./main";
import { Battery } from "./components/battery";
import { ReactNode, useContext } from "react";
import { ThrottleRaw } from "./components/throttleRaw";
import { RFData } from "./components/rfData";
import { Settings } from "./components/settings";
import { BoardType, DataContext } from "./utils/context";
import { Calibration } from "./components/calibration";
import { Throttle } from "./components/throttle";
import { CalibrationData } from "./components/calibrationData";
import { RF } from "./components/rf";
import Divider from "@mui/material/Divider";

function CustomTabPanel({
  children,
  value,
  currentTab,
}: {
  children: ReactNode;
  value: string;
  currentTab: string;
}) {
  return (
    <TabPanel
      sx={{
        flexGrow: currentTab === value ? 1 : 0,
        padding: "0.1rem",
        display: "flex",
        flexDirection: "column",
      }}
      value={value}
    >
      {children}
    </TabPanel>
  );
}

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
    font: smallSlant,
    text: "!Fancy",
  });

  // Using a ref callback to bridge the type mismatch.
  const refCallback = (element: HTMLPreElement | null) => {
    if (asciiTextRef) {
      // Directly manipulate the `.current` property only if it's not `undefined`.
      asciiTextRef.current = element ?? undefined; // Convert `null` to `undefined`.
    }
  };

  const {
    boardType,
    remoteVoltage,
    boardVoltage,
    encodedThrottle,
    throttle1Buffer,
    throttle2Buffer,
    channel,
    identity,
    RSSI,
    SNR,
    websocketStatus,
    cellN,
    isDual,
    calAcc,
    calBrake,
    centerAcc,
    centerBrake,
    inverted,
    tab,
    setTab,
  } = useContext(DataContext);

  return (
    <>
      <pre
        css={{ color: colors.primary, padding: 0, margin: 0 }}
        ref={refCallback}
      ></pre>
      <Typography variant="subtitle1">
        Board is {import.meta.env.VITE_BOARD_TYPE}. Websocket status:{" "}
        {websocketStatus}
      </Typography>
      <Box
        sx={{
          width: "100%",
          flexGrow: 1,
          display: "flex",
          flexDirection: "column",
        }}
      >
        <TabContext value={tab}>
          <Box sx={{ borderBottom: 1, borderColor: "divider" }}>
            <TabList
              variant="fullWidth"
              onChange={(_event, tab) => setTab(tab)}
            >
              <Tab label="Telemetry" value="0" />
              <Tab label="Settings" value="1" />
              {boardType === BoardType.TX && (
                <Tab label="Calibration" value="2" />
              )}
            </TabList>
          </Box>
          <CustomTabPanel value="0" currentTab={tab}>
            {boardType === BoardType.TX && (
              <>
                <Battery
                  title="Remote"
                  cells={1}
                  voltage={remoteVoltage}
                ></Battery>
                <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
              </>
            )}
            <Battery
              title="Board"
              cells={cellN}
              voltage={boardVoltage}
            ></Battery>
            <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
            {boardType === BoardType.TX && (
              <>
                <ThrottleRaw
                  throttle1Values={throttle1Buffer}
                  throttle2Values={throttle2Buffer}
                ></ThrottleRaw>
                <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
              </>
            )}
            <Throttle throttle={encodedThrottle}></Throttle>
            <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
            {boardType === BoardType.TX && (
              <>
                <CalibrationData
                  calAcc={calAcc}
                  calBrake={calBrake}
                  centerAcc={centerAcc}
                  centerBrake={centerBrake}
                  inverted={inverted}
                  isDual={isDual}
                ></CalibrationData>
                <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
              </>
            )}
            <RFData channel={channel} identity={identity}></RFData>
            {boardType === BoardType.RX && (
              <>
                <Divider sx={{ margin: "0.5rem 0 0 0.1rem" }} />
                <RF SNR={SNR} RSSI={RSSI}></RF>
              </>
            )}
          </CustomTabPanel>
          <CustomTabPanel value="1" currentTab={tab}>
            <Settings />
          </CustomTabPanel>
          {boardType === BoardType.TX && (
            <CustomTabPanel value="2" currentTab={tab}>
              <Calibration />
            </CustomTabPanel>
          )}
        </TabContext>
      </Box>
    </>
  );
}

export default App;
