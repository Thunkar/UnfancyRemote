import { css } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";

const MAX_SNR = 15;
const MIN_SNR = -25;

const MAX_RSSI = -13;
const MIN_RSSI = -100;

function getColor(c1: string, c2: string, ratio: number) {
  const interpolate = (start: number[], end: number[], ratio: number) => {
    const r = Math.trunc(ratio * end[0] + (1 - ratio) * start[0]);
    const g = Math.trunc(ratio * end[1] + (1 - ratio) * start[1]);
    const b = Math.trunc(ratio * end[2] + (1 - ratio) * start[2]);
    return [r, g, b];
  };

  const hexToRgb = (hex: string) => [
    parseInt(hex.substring(1, 3), 16),
    parseInt(hex.substring(3, 5), 16),
    parseInt(hex.substring(5, 7), 16),
  ];

  const rgbToHex = (rgb: number[]) =>
    "#" +
    rgb
      .map((x) => {
        const hex = x.toString(16);
        return hex.length === 1 ? "0" + hex : hex;
      })
      .join("");

  const rgbInterpolated = interpolate(hexToRgb(c1), hexToRgb(c2), ratio / 100);
  return rgbToHex(rgbInterpolated);
}

function SignalBar({ percentage }: { percentage: number }) {
  return (
    <div css={container}>
      <div
        css={{
          bottom: 0,
          height: `${percentage}%`,
          backgroundColor: getColor("#cc0000", "#2bff00", percentage),
          borderRadius: "0.5rem",
          position: "absolute",
          width: "100%",
        }}
      ></div>
      <div
        css={{
          position: "absolute",
          top: "calc(50% - 1px)",
          height: "2px",
          width: "100%",
          backgroundColor: "white",
        }}
      ></div>
    </div>
  );
}

const container = css({
  display: "flex",
  margin: "0 0.5rem",
  border: "1px solid",
  borderColor: "white",
  borderRadius: "0.5rem",
  height: "20rem",
  width: "2rem",
  position: "relative",
});

export function RF({ SNR, RSSI }: { SNR: number; RSSI: number }) {
  const SNROffset = SNR - MIN_SNR;
  let SNRPercentage = (SNROffset * 100) / (MAX_SNR - MIN_SNR);
  SNRPercentage = Math.max(Math.min(SNRPercentage, 100), 0);

  const RSSIOffset = RSSI - MIN_RSSI;
  let RSSIPercentage = (RSSIOffset * 100) / (MAX_RSSI - MIN_RSSI);
  RSSIPercentage = Math.max(Math.min(RSSIPercentage, 100), 0);

  return (
    <Box sx={{ padding: "0.1rem 0.5rem" }}>
      <Box
        sx={{
          display: "flex",
          justifyContent: "space-around",
          flexDirection: "row",
          textAlign: "center",
        }}
      >
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            width: "50%",
            alignItems: "center",
          }}
        >
          <Typography
            sx={{ width: "100%", margin: "0.5rem 0" }}
            variant="caption"
          >
            SNR: {SNR}dB
          </Typography>
          <SignalBar percentage={SNRPercentage} />
        </Box>
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            width: "50%",
            alignItems: "center",
          }}
        >
          <Typography
            sx={{ width: "100%", margin: "0.5rem 0" }}
            variant="caption"
          >
            RSSI: {RSSI}dBm
          </Typography>
          <SignalBar percentage={RSSIPercentage} />
        </Box>
      </Box>
    </Box>
  );
}
