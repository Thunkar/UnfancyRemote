import Dialog from "@mui/material/Dialog";
import DialogActions from "@mui/material/DialogActions";
import DialogTitle from "@mui/material/DialogTitle";
import DialogContent from "@mui/material/DialogContent";
import Typography from "@mui/material/Typography";
import Button from "@mui/material/Button";
import { useEffect, useState } from "react";
import { CircularProgress } from "@mui/material";

interface ConfirmationDialogProps {
  open: boolean;
  title: string;
  body: string;
  onClose: (result: boolean) => void;
}

const DISABLED_TIMEOUT = 5000;

export function ConfirmationDialog({
  open,
  title,
  body,
  onClose,
}: ConfirmationDialogProps) {
  const [buttonDisabled, setButtonDisabled] = useState(true);

  const [progress, setProgress] = useState(0);
  const start = Date.now();

  useEffect(() => {
    const handle = setInterval(() => {
      const ellapsed = Date.now() - start;
      const currentProgress = Math.round((ellapsed / DISABLED_TIMEOUT) * 100);
      setProgress(currentProgress);

      if (currentProgress >= 100) {
        clearInterval(handle);
        setButtonDisabled(false);
      }

      return () => clearInterval(handle);
    }, 100);
  }, []);

  return (
    <Dialog open={open}>
      <DialogTitle>{title}</DialogTitle>
      <DialogContent
        css={{
          display: "flex",
          flexDirection: "column",
          overflowY: "hidden",
          wordBreak: "break-word",
        }}
      >
        <Typography variant="subtitle1">{body}</Typography>
      </DialogContent>
      <DialogActions>
        <Button
          variant="contained"
          color="warning"
          onClick={() => onClose(true)}
          disabled={buttonDisabled}
        >
          {buttonDisabled && (
            <CircularProgress
              color="error"
              sx={{ marginRight: 1.5 }}
              size={20}
              value={progress}
              variant="determinate"
            />
          )}
          Ok
        </Button>
        <Button
          variant="contained"
          color="error"
          onClick={() => onClose(false)}
        >
          Cancel
        </Button>
      </DialogActions>
    </Dialog>
  );
}
