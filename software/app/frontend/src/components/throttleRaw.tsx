import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import { LineChart } from "@mui/x-charts/LineChart";

export function ThrottleRaw({
  throttle1Values,
  throttle2Values,
}: {
  throttle1Values: number[];
  throttle2Values: number[];
}) {
  return (
    <Box sx={{ padding: "0.1rem 0.5rem" }}>
      <Typography sx={{ margin: "0.1rem" }} variant="subtitle2">
        Raw Throttle
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
          { max: 2800, min: 1200 },
          { max: 2800, min: 1200 },
        ]}
        width={400}
        height={250}
        margin={{ left: 40, right: 40, top: 10, bottom: 20 }}
        grid={{ vertical: true, horizontal: true }}
      />
    </Box>
  );
}
