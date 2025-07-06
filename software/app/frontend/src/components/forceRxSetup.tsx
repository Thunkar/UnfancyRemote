import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import Button from "@mui/material/Button";
import { ConfirmationDialog } from "./confirmationDialog";
import { useState } from "react";
import { forceRXSetup } from "../utils/requests";

export function ForceRxSetup() {
  const [confirmationDialogOpen, setConfirmationDialogOpen] = useState(false);

  const handleChange = async (result: boolean) => {
    if (result) {
      await forceRXSetup();
    }
    setConfirmationDialogOpen(false);
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
      <Typography variant="subtitle1">
        Force the paired receiver into setup mode, so it can be configured via
        WiFi.
      </Typography>
      <Typography variant="caption" color="warning">
        WARNING: This will reset the receiver and make it NO LONGER OUTPUT A
        THROTTLE SIGNAL. NEVER use this feature while riding!
      </Typography>
      <Button
        variant="outlined"
        sx={{ mt: "auto" }}
        onClick={() => setConfirmationDialogOpen(true)}
      >
        Update
      </Button>
      {confirmationDialogOpen && (
        <ConfirmationDialog
          open={confirmationDialogOpen}
          title="Confirm RX reset"
          body="The receiver will reset and start in setup mode"
          onClose={(result) => handleChange(result)}
        />
      )}
    </Box>
  );
}
