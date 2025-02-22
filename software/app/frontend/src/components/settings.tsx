import {
  Box,
  Button,
  TextField,
  ToggleButton,
  ToggleButtonGroup,
} from "@mui/material";
import { useContext, useState } from "react";
import { DataContext } from "../utils/context";

export function Settings() {
  const { cellN, txIdentity, channel, isDual, storeSettings } =
    useContext(DataContext);

  const [currentNCells, setCurrentNCells] = useState<number>(cellN);
  const [currentTxIdentity, setCurrentTxIdentity] =
    useState<number>(txIdentity);
  const [currentChannel, setCurrentChannel] = useState<number>(channel);
  const [currentIsDual, setCurrentIsDual] = useState<boolean>(isDual);

  const handleChange = async () => {
    storeSettings({
      nCells: currentNCells,
      txIdentity: currentTxIdentity,
      channel: currentChannel,
      isDual: currentIsDual,
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
        justifySelf: "flex-start",
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
      <ToggleButtonGroup
        value={currentIsDual}
        exclusive
        onChange={() => setCurrentIsDual(!currentIsDual)}
        aria-label="text alignment"
      >
        <ToggleButton value={true}>Dual throttle</ToggleButton>
        <ToggleButton value={false}>Single throttle</ToggleButton>
      </ToggleButtonGroup>
      <Button variant="outlined" onClick={() => handleChange()}>
        Update
      </Button>
    </Box>
  );
}
