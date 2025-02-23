import { Box, Divider, Typography, css } from "@mui/material";

const CH_BANDWIDTH_HZ = 2500000;
const BASE_FREQUENCY = 2400000000;

const container = css({
  display: "flex",
  flexDirection: "row",
});

const dataContainer = css({
  display: "flex",
  flexDirection: "column",
  width: "50%",
  justifyContent: "center",
});

export function RF({
  channel,
  txIdentity,
}: {
  channel: number;
  txIdentity: number;
}) {
  const frequency = channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;
  return (
    <Box
      sx={{
        padding: "0.1rem 0.5rem",
      }}
    >
      <Typography sx={{ lineHeight: "1rem" }} variant="subtitle2">
        RF
      </Typography>
      <div css={container}>
        <div css={dataContainer}>
          <Typography variant="caption">Channel: {channel}</Typography>
          <Typography variant="caption">Tx Identity: {txIdentity}</Typography>
        </div>
        <Divider
          sx={{ height: "2.75rem", margin: "0 1rem" }}
          orientation="vertical"
        />
        <div css={dataContainer}>
          <Typography variant="caption">
            Frequency: {frequency / 1000000} MHz
          </Typography>
        </div>
      </div>
    </Box>
  );
}
