import { Divider } from "@mui/material";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";



export function TimingData({ minPacketTimeUs, maxPacketTimeUs }: { minPacketTimeUs: number, maxPacketTimeUs: number }) {
  return (
    <Box sx={{ margin: "0.5rem", padding: "0.1rem 0.5rem", display: "flex", flexDirection: "row", justifyContent: "space-around" }}>
        <Typography variant="caption">
          Min packet time:
          <br />
          {minPacketTimeUs} µs
        </Typography>
        <Divider orientation="vertical" sx={{ height: "2.5rem" }} />
        <Typography variant="caption">
          Max packet time:
          <br />
          {maxPacketTimeUs} µs
        </Typography>
    </Box>

  );

}