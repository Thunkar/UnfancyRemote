import { Box, Divider, Typography } from "@mui/material";
import { LineChart } from "@mui/x-charts/LineChart";

export function Throttle({
  throttle1Values,
  throttle2Values,
}: {
  throttle1Values: number[];
  throttle2Values: number[];
}) {
  return (
    <Box sx={{ padding: "0.5rem" }}>
      <Typography sx={{ margin: "0.1rem" }} variant="subtitle1">
        Throttle
      </Typography>
      <LineChart
        skipAnimation
        series={[
          {
            data: throttle1Values,
            showMark: false,
            curve: "step",
            label: "Throttle 1",
          },
          {
            data: throttle2Values,
            showMark: false,
            curve: "step",
            label: "Throttle 2",
          },
        ]}
        yAxis={[
          { max: 2200, min: 1500 },
          { max: 2200, min: 1500 },
        ]}
        width={400}
        height={250}
        margin={{ left: 40, right: 40, top: 10, bottom: 40 }}
        grid={{ vertical: true, horizontal: true }}
      />
      <Divider />
    </Box>
  );
}
