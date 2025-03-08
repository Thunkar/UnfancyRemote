import Box from "@mui/material/Box";
import Divider from "@mui/material/Divider";
import Typography from "@mui/material/Typography";

export function CalibrationData({
  isDual,
  calAcc,
  calBrake,
  centerAcc,
  centerBrake,
  inverted,
}: {
  isDual: boolean;
  calAcc: number;
  calBrake: number;
  centerAcc: number;
  centerBrake: number;
  inverted: boolean;
}) {
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
    </Box>
  );
}
