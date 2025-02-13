import { css } from "@emotion/react";
import { Button } from "@mui/material";
import { ansiRegular, useAsciiText } from "react-ascii-text";
import { colors } from "./main";

const container = css({
  display: "flex",
  flexDirection: "column",
  width: "100%",
  height: "100%",
  alignItems: "center",
});

function App() {
  const asciiTextRef = useAsciiText({
    animationCharacters: "▒░█",
    animationCharacterSpacing: 1,
    animationDelay: 2000,
    animationDirection: "down",
    animationInterval: 100,
    animationLoop: false,
    animationSpeed: 30,
    fadeInOnly: true,
    font: ansiRegular,
    text: "!Fancy",
  });

  // Using a ref callback to bridge the type mismatch.
  const refCallback = (element: HTMLPreElement | null) => {
    if (asciiTextRef) {
      // Directly manipulate the `.current` property only if it's not `undefined`.
      asciiTextRef.current = element ?? undefined; // Convert `null` to `undefined`.
    }
  };

  return (
    <div css={container}>
      <pre css={{ color: colors.primary }} ref={refCallback}></pre>
      <Button variant="outlined">Test</Button>
    </div>
  );
}

export default App;
