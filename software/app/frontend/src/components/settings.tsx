import { Box, Button, Input, TextField } from "@mui/material";
import { useState } from "react";

export function Settings({
  cellN,
  txIdentity,
  channel,
}: {
  cellN: number;
  txIdentity: number;
  channel: number;
}) {
  const [currentNCells, setCurrentNCells] = useState<number>(cellN);

  const handleChange = async () => {
    const url = new URL(
      import.meta.env.VITE_HTTP_URL
        ? `${import.meta.env.VITE_HTTP_URL}/settings`
        : `http://${window.location.hostname}/settings`
    );

    await fetch(url, {
      method: "POST",
      body: JSON.stringify({ nCells: currentNCells }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  };

  return (
    <Box sx={{ padding: "0.5rem" }}>
      <TextField
        value={currentNCells}
        onChange={(event) => setCurrentNCells(parseInt(event.target.value))}
        fullWidth
        label="# of cells"
      />
      <Button variant="outlined" onClick={() => handleChange()}>
        Update
      </Button>
    </Box>
  );
}
