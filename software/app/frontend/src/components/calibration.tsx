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
      "Let the throttle and brake triggers return to the neutral position",
  },
  {
    label: "Calibrate throttle",
    description: "Press the throttle to the maximum position.",
  },
  {
    label: "Calibrate brake",
    description: "Press the brake to the maximum position.",
  },
  {
    label: "Confirm calibration",
    description: "Review the values and save",
  },
];
const singleSteps: Step[] = [
  {
    label: "Center",
    description: "Let the throttle return to the neutral position",
  },
  {
    label: "Calibrate throttle",
    description: "Push the throttle to the maximum position.",
  },
  {
    label: "Calibrate brake",
    description: "Pull the throttle to the minimum position.",
  },
  {
    label: "Confirm calibration",
    description: "Review the values and save",
  },
];

export function Calibration() {
  const { throttle1, throttle2, isDual, storeCalibration, setTab } =
    useContext(DataContext);

  const [throttle1Max, setThrottle1Max] = useState(0);
  const [throttle2Max, setThrottle2Max] = useState(0);
  const [throttle1Min, setThrottle1Min] = useState(10e3);
  const [throttle2Min, setThrottle2Min] = useState(10e3);

  const [currentCenterAcc, setCurrentCenterAcc] = useState(0);
  const [currentCenterBrake, setCurrentCenterBrake] = useState(0);
  const [currentCalAcc, setCurrentCalAcc] = useState(0);
  const [currentCalBrake, setCurrentCalBrake] = useState(0);
  const [currentInverted, setCurrentInverted] = useState(false);
  const [diff, setDiff] = useState(0);

  const [activeStep, setActiveStep] = useState(0);

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
    switch (activeStep) {
      case 0: {
        setCurrentCenterAcc(throttle1);
        setCurrentCenterBrake(throttle2);
        break;
      }
      case 1: {
        const newDiff = Math.abs(currentCenterAcc - throttle1);
        if (newDiff > diff) {
          setCurrentCalAcc(throttle1);
          setDiff(newDiff);
        }
        break;
      }
      case 2: {
        let newDiff;
        let current;
        if (isDual) {
          current = throttle2;
          newDiff = Math.abs(currentCenterBrake - current);
        } else {
          current = throttle1;
          newDiff = Math.abs(currentCenterAcc - current);
        }
        if (newDiff > diff) {
          setCurrentCalBrake(current);
          setDiff(newDiff);
        }
        break;
      }
      case 3: {
        setCurrentInverted(currentCalAcc < currentCalBrake);
      }
    }
  }, [throttle1, throttle2, activeStep]);

  const steps = isDual ? dualSteps : singleSteps;
  const maxSteps = steps.length;

  const handleNext = () => {
    setDiff(0);
    setActiveStep((prevActiveStep) => prevActiveStep + 1);
  };

  const handleBack = () => {
    setDiff(0);
    setActiveStep((prevActiveStep) => prevActiveStep - 1);
  };

  const handleSave = async () => {
    await storeCalibration({
      calBrake: currentCalBrake,
      calAcc: currentCalAcc,
      centerAcc: currentCenterAcc,
      centerBrake: currentCenterBrake,
      inverted: currentInverted,
    });
    setTab("0");
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
      <Box sx={{ width: "100%", p: "0.5rem", flexGrow: 1 }}>
        <Typography variant="h4" sx={{ mb: "0.5rem" }}>
          {steps[activeStep].label}
        </Typography>
        <Typography sx={{ height: "2rem" }} variant="body1">
          {steps[activeStep].description}
        </Typography>
        <Box
          sx={{
            display: "flex",
            flexDirection: "row",
            justifyContent: "space-around",
            alignItems: "center",
            margin: "2rem 0",
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
          {isDual && <Divider orientation="vertical" sx={{ height: "4rem" }} />}
          {isDual && <Typography variant="h3">{throttle2}</Typography>}
          {isDual && (
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
          )}
        </Box>
        <Box>
          <Divider sx={{ mb: "1rem" }} />
          {activeStep > 0 && (
            <Typography variant="body1">
              Center throttle: {currentCenterAcc}
            </Typography>
          )}
          {isDual && activeStep > 0 && (
            <Typography variant="body1">
              Center brake: {currentCenterBrake}
            </Typography>
          )}
          {activeStep > 1 && (
            <Typography variant="body1">
              Max throttle: {currentCalAcc}
            </Typography>
          )}
          {activeStep > 2 && (
            <Typography variant="body1">
              Max brake: {currentCalBrake}
            </Typography>
          )}
          {!isDual && activeStep === 3 && (
            <Typography variant="body1">
              Inverted: {currentInverted ? "Yes" : "No"}
            </Typography>
          )}
        </Box>
      </Box>
      <MobileStepper
        variant="text"
        steps={maxSteps}
        position="static"
        activeStep={activeStep}
        nextButton={
          activeStep !== 3 ? (
            <Button
              size="small"
              onClick={handleNext}
              disabled={activeStep === maxSteps - 1}
            >
              Next
              <KeyboardArrowRight />
            </Button>
          ) : (
            <Button size="small" onClick={() => handleSave()}>
              Save
            </Button>
          )
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
