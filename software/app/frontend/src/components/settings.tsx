import { Box, Button, Input, TextField } from "@mui/material";

export function Settings({
  cellN,
  txIdentity,
  channel,
}: {
  cellN: number;
  txIdentity: number;
  channel: number;
}) {
  const handleChange = async (param: string, value: number) => {
    const url = new URL(
      import.meta.env.VITE_WS_URL ??
        `http://${window.location.hostname}/settings`
    );

    url.search = new URLSearchParams({
      param,
      value: value.toString(),
    }).toString();

    await fetch(url);
  };

  return (
    <Box sx={{ padding: "0.5rem" }}>
      <TextField value={cellN} fullWidth label="# of cells" />
      <Button variant="outlined">Update</Button>
    </Box>
  );
}
