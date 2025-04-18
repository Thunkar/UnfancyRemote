import { css, Divider } from "@mui/material";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";

const timings = css({
  margin: "0",
  padding: "0",
});

type TimingDataProps = {
  packetsPerSecond: number;
  TMPacketsPerSecond: number;
  minPacketTimeUs: number;
  meanPacketTimeUs: number;
  maxPacketTimeUs: number;
};

export function TimingData({
  packetsPerSecond,
  TMPacketsPerSecond,
  minPacketTimeUs,
  meanPacketTimeUs,
  maxPacketTimeUs,
}: TimingDataProps) {
  return (
    <Box
      sx={{
        margin: "0.5rem",
        padding: "0.1rem 0.5rem",
        display: "flex",
        flexDirection: "row",
        justifyContent: "space-around",
      }}
    >
      <Box sx={{ display: "flex", flexDirection: "column", width: "50%" }}>
        <Typography variant="subtitle2">Reception timings:</Typography>
        <Typography variant="caption">
          <pre css={timings}>
            min:{"  "}
            {minPacketTimeUs
              ? (minPacketTimeUs / 1000).toFixed(2).padStart(6)
              : "N/A"}
            ms
          </pre>
          <pre css={timings}>
            mean:{" "}
            {meanPacketTimeUs
              ? (meanPacketTimeUs / 1000).toFixed(2).padStart(6)
              : "N/A"}
            ms
          </pre>
          <pre css={timings}>
            max:{"  "}
            {maxPacketTimeUs
              ? (maxPacketTimeUs / 1000).toFixed(2).padStart(6)
              : "N/A"}
            ms
          </pre>
        </Typography>
      </Box>
      <Divider
        orientation="vertical"
        sx={{ height: "5.5rem", margin: "0 1rem" }}
      />
      <Box sx={{ display: "flex", flexDirection: "column", width: "50%" }}>
        <Typography variant="subtitle2">Packets per second</Typography>
        <Typography variant="caption">
          Throttle: {packetsPerSecond}
          <br />
          TM: {TMPacketsPerSecond}
        </Typography>
      </Box>
    </Box>
  );
}
