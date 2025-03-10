import { Settings, Calibration } from "./context";

function buildUrl(path: string): URL {
  return new URL(
    import.meta.env.VITE_HTTP_URL
      ? `${import.meta.env.VITE_HTTP_URL}/${path}`
      : `http://${window.location.hostname}/${path}`
  );
}

export async function saveSettings(settings: Settings): Promise<void> {
  const url = buildUrl("settings");
  await fetch(url, {
    method: "POST",
    body: JSON.stringify({
      cellN: settings.cellN,
      identity: settings.identity,
      channel: settings.channel,
      isDual: settings.isDual ? 1 : 0,
    }),
    headers: {
      "Content-Type": "application/json",
    },
  });
}

export async function loadSettings(): Promise<Settings> {
  const url = buildUrl("settings");

  const response = await fetch(url);
  const body = await response.json();
  return {
    cellN: body.cellN,
    identity: body.identity,
    channel: body.channel,
    isDual: body.isDual === 1,
  };
}

export async function saveCalibration(calibration: Calibration): Promise<void> {
  const url = buildUrl("calibration");

  await fetch(url, {
    method: "POST",
    body: JSON.stringify({
      calBrake: calibration.calBrake,
      calAcc: calibration.calAcc,
      centerAcc: calibration.centerAcc,
      centerBrake: calibration.centerBrake,
      inverted: calibration.inverted ? 1 : 0,
    }),
    headers: {
      "Content-Type": "application/json",
    },
  });
}

export async function loadCalibration(): Promise<Calibration> {
  const url = buildUrl("calibration");

  const response = await fetch(url);
  const body = await response.json();
  return {
    calBrake: body.calBrake,
    calAcc: body.calAcc,
    centerAcc: body.centerAcc,
    centerBrake: body.centerBrake,
    inverted: body.inverted === 1,
  };
}
