import { css } from "@emotion/react";
import { Divider, Typography } from "@mui/material";
import { Box } from "@mui/system";

const ENCODED_MAX = 65535;

const container = css({
  display: "flex",
  margin: "0 0.5rem",
  border: "1px solid",
  borderColor: "white",
  borderRadius: "0.5rem",
  height: "2rem",
});

export function Throttle({
  throttle,
  isDual,
  calAcc,
  calBrake,
  centerAcc,
  centerBrake,
  inverted,
}: {
  throttle: number;
  isDual: boolean;
  calAcc: number;
  calBrake: number;
  centerAcc: number;
  centerBrake: number;
  inverted: boolean;
}) {
  let percentage = (throttle * 100) / ENCODED_MAX;
  percentage = Math.max(Math.min(percentage, 100), 0);

  return (
    <Box sx={{ padding: "0.1rem 0.5rem" }}>
      <Typography sx={{ margin: "0.1rem" }} variant="subtitle2">
        Encoded Throttle
      </Typography>
      <div css={container}>
        <div
          css={{
            left: percentage > 50 ? "50%" : `${percentage}%`,
            width:
              percentage > 50 ? `${percentage - 50}%` : `${50 - percentage}%`,
            backgroundColor: percentage > 50 ? "blue" : "red",
            borderRadius:
              percentage > 50 ? "0 0.5rem 0.5rem 0" : "0.5rem 0 0 0.5rem",
            height: "100%",
            position: "relative",
          }}
        ></div>
        <div
          css={{
            position: "absolute",
            left: "calc(50% - 1px)",
            width: "2px",
            height: "2rem",
            backgroundColor: "white",
          }}
        ></div>
      </div>
      <Box
        sx={{
          display: "flex",
          justifyContent: "space-around",
          flexDirection: "row",
          textAlign: "center",
        }}
      >
        <Typography variant="caption">
          Center throttle:
          <br />
          {centerAcc}
        </Typography>
        <Divider orientation="vertical" sx={{ height: "2.5rem" }} />
        <Typography variant="caption">
          Max throttle:
          <br />
          {calAcc}
        </Typography>
        <Divider orientation="vertical" sx={{ height: "2.5rem" }} />
        {isDual && (
          <Typography variant="caption">
            Center brake:
            <br />
            {centerBrake}
          </Typography>
        )}
        {isDual && <Divider orientation="vertical" sx={{ height: "2.5rem" }} />}
        <Typography variant="caption">
          Max brake:
          <br />
          {calBrake}
        </Typography>
        <Divider orientation="vertical" sx={{ height: "2.5rem" }} />
        {!isDual && (
          <Typography variant="caption">
            Inverted:
            <br />
            {inverted ? "Yes" : "No"}
          </Typography>
        )}
      </Box>
      <Divider sx={{ margin: "0.5rem 0" }} />
    </Box>
  );
}
