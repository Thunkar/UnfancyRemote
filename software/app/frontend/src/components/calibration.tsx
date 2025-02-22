import KeyboardArrowLeft from "@mui/icons-material/KeyboardArrowLeft";
import KeyboardArrowRight from "@mui/icons-material/KeyboardArrowRight";

import { Box, Button, Divider, MobileStepper, Typography } from "@mui/material";
import { useContext, useEffect, useState } from "react";
import { DataContext } from "../utils/context";

type Step = {
  label: string;
  description: string;
};

const dualSteps: Step[] = [
  {
    label: "Center",
    description:
      "Let the throttle and brake triggers return to the center position",
  },
  {
    label: "Calibrate throttle",
    description: "Press the throttle to the maximum position.",
  },
  {
    label: "Calibrate brake",
    description: "Press the brake to the maximum position.",
  },
];
const singleSteps: Step[] = [];

export function Calibration() {
  const { throttle1, throttle2, isDual } = useContext(DataContext);

  const [throttle1Max, setThrottle1Max] = useState(0);
  const [throttle2Max, setThrottle2Max] = useState(0);
  const [throttle1Min, setThrottle1Min] = useState(10e3);
  const [throttle2Min, setThrottle2Min] = useState(10e3);

  useEffect(() => {
    if (throttle1 > throttle1Max) {
      setThrottle1Max(throttle1);
    }
    if (throttle1 < throttle1Min) {
      setThrottle1Min(throttle1);
    }
    if (throttle2 > throttle2Max) {
      setThrottle2Max(throttle2);
    }
    if (throttle2 < throttle2Min) {
      setThrottle2Min(throttle2);
    }
  }, [throttle1, throttle2]);

  const steps = isDual ? dualSteps : singleSteps;
  const maxSteps = steps.length;

  const [activeStep, setActiveStep] = useState(0);

  const handleNext = () => {
    setActiveStep((prevActiveStep) => prevActiveStep + 1);
  };

  const handleBack = () => {
    setActiveStep((prevActiveStep) => prevActiveStep - 1);
  };

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        width: "100%",
        height: "100%",
        flexGrow: 1,
      }}
    >
      <Box sx={{ width: "100%", p: 2, flexGrow: 1 }}>
        <Typography variant="h4" sx={{ mb: "0.5rem" }}>
          {steps[activeStep].label}
        </Typography>
        <Typography variant="body1">{steps[activeStep].description}</Typography>
        {isDual ? (
          <Box
            sx={{
              display: "flex",
              flexDirection: "row",
              justifyContent: "space-around",
              alignItems: "center",
            }}
          >
            <Typography variant="h3">{throttle1}</Typography>
            <Box
              sx={{
                display: "flex",
                flexDirection: "column",
                justifyContent: "center",
              }}
            >
              <Typography variant="caption">{throttle1Max} (max)</Typography>
              <Typography variant="caption">{throttle1Min} (min)</Typography>
            </Box>
            <Divider orientation="vertical" sx={{ height: "4rem" }} />
            <Typography variant="h3">{throttle2}</Typography>
            <Box
              sx={{
                display: "flex",
                flexDirection: "column",
                justifyContent: "center",
              }}
            >
              <Typography variant="caption">{throttle2Max} (max)</Typography>
              <Typography variant="caption">{throttle2Min} (min)</Typography>
            </Box>
          </Box>
        ) : (
          <Box></Box>
        )}
      </Box>
      <MobileStepper
        variant="text"
        steps={maxSteps}
        position="static"
        activeStep={activeStep}
        nextButton={
          <Button
            size="small"
            onClick={handleNext}
            disabled={activeStep === maxSteps - 1}
          >
            Next
            <KeyboardArrowRight />
          </Button>
        }
        backButton={
          <Button size="small" onClick={handleBack} disabled={activeStep === 0}>
            <KeyboardArrowLeft />
            Back
          </Button>
        }
      />
    </Box>
  );
}
