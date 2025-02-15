import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import App from "./App.tsx";
import "./index.css";
import {
  createTheme,
  CssBaseline,
  ThemeOptions,
  ThemeProvider,
} from "@mui/material";

export const colors = {
  primary: "#ffd217",
  secondary: "#2c6bfd",
};

const themeOptions: ThemeOptions = {
  palette: {
    mode: "dark",
    primary: {
      main: colors.primary,
    },
    secondary: {
      main: colors.secondary,
    },
  },
};

const theme = createTheme(themeOptions);

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <App />
    </ThemeProvider>
  </StrictMode>
);
