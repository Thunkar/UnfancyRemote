import {
  Box,
  Button,
  TextField,
  ToggleButton,
  ToggleButtonGroup,
} from "@mui/material";
import { useContext, useEffect, useState } from "react";
import { BoardType, DataContext } from "../utils/context";

export function Settings() {
  const { boardType, cellN, identity, channel, isDual, storeSettings } =
    useContext(DataContext);

  const [currentCellN, setCurrentCellN] = useState<number>(cellN);
  const [currentidentity, setCurrentidentity] = useState<number>(identity);
  const [currentChannel, setCurrentChannel] = useState<number>(channel);
  const [currentIsDual, setCurrentIsDual] = useState<boolean>(isDual);

  const [dirty, setDirty] = useState<boolean>(false);

  useEffect(() => {
    const isDirty =
      cellN !== currentCellN ||
      identity !== currentidentity ||
      channel !== currentChannel ||
      isDual !== currentIsDual;
    setDirty(isDirty);
  }, [
    cellN,
    identity,
    channel,
    isDual,
    currentChannel,
    currentIsDual,
    currentCellN,
    currentidentity,
  ]);

  const handleChange = async () => {
    storeSettings({
      cellN: currentCellN,
      identity: currentidentity,
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
        height: "100%",
        justifySelf: "flex-start",
      }}
    >
      <TextField
        value={currentCellN}
        onChange={(event) => setCurrentCellN(parseInt(event.target.value))}
        fullWidth
        label="# of cells"
      />
      <TextField
        value={currentidentity}
        onChange={(event) => setCurrentidentity(parseInt(event.target.value))}
        fullWidth
        label="TX Identity"
      />
      <TextField
        value={currentChannel}
        onChange={(event) => setCurrentChannel(parseInt(event.target.value))}
        fullWidth
        label="Channel"
      />
      {boardType === BoardType.TX && (
        <ToggleButtonGroup
          value={currentIsDual}
          exclusive
          onChange={() => setCurrentIsDual(!currentIsDual)}
          fullWidth
        >
          <ToggleButton value={true}>Dual throttle</ToggleButton>
          <ToggleButton value={false}>Single throttle</ToggleButton>
        </ToggleButtonGroup>
      )}
      <Button
        variant="outlined"
        sx={{ mt: "auto" }}
        disabled={!dirty}
        onClick={() => handleChange()}
      >
        Update
      </Button>
    </Box>
  );
}
