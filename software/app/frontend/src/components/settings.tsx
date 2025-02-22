import { Box, Button, TextField } from "@mui/material";
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
  const [currentTxIdentity, setCurrentTxIdentity] =
    useState<number>(txIdentity);
  const [currentChannel, setCurrentChannel] = useState<number>(channel);

  const handleChange = async () => {
    const url = new URL(
      import.meta.env.VITE_HTTP_URL
        ? `${import.meta.env.VITE_HTTP_URL}/settings`
        : `http://${window.location.hostname}/settings`
    );

    await fetch(url, {
      method: "POST",
      body: JSON.stringify({
        nCells: currentNCells,
        txIdentity: currentTxIdentity,
        channel: currentChannel,
      }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  };

  return (
    <Box
      sx={{
        padding: "0.5rem",
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        gap: "1rem",
      }}
    >
      <TextField
        value={currentNCells}
        onChange={(event) => setCurrentNCells(parseInt(event.target.value))}
        fullWidth
        label="# of cells"
      />
      <TextField
        value={currentTxIdentity}
        onChange={(event) => setCurrentTxIdentity(parseInt(event.target.value))}
        fullWidth
        label="TX Identity"
      />
      <TextField
        value={currentChannel}
        onChange={(event) => setCurrentChannel(parseInt(event.target.value))}
        fullWidth
        label="Channel"
      />
      <Button variant="outlined" onClick={() => handleChange()}>
        Update
      </Button>
    </Box>
  );
}
