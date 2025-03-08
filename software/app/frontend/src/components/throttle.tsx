import { css } from "@mui/material/styles";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";

const ENCODED_MAX = 4095;

const container = css({
  display: "flex",
  margin: "0 0.5rem",
  border: "1px solid",
  borderColor: "white",
  borderRadius: "0.5rem",
  height: "2rem",
});

export function Throttle({ throttle }: { throttle: number }) {
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
    </Box>
  );
}
