import { css } from "@emotion/react";
import { Typography } from "@mui/material";

const container = css({
  display: "flex",
  flexDirection: "column",
  width: "100%",
  padding: "0.1rem 0.5rem",
  boxSizing: "border-box",
});

const text = css({
  display: "flex",
  width: "30%",
  alignContent: "center",
});

const data = css({
  display: "flex",
  flexDirection: "row",
});

const squaresContainer = css({
  display: "flex",
  flexDirection: "row",
  flexGrow: 1,
});

const levelSquare = {
  height: "100%",
  width: "100%",
};

const colors = {
  [4.0]: css({
    ...levelSquare,
    backgroundColor: "#2bff00",
  }),
  [3.9]: css({
    ...levelSquare,
    backgroundColor: "#fff925",
  }),
  [3.8]: css({
    ...levelSquare,
    backgroundColor: "#ffdc25",
  }),
  [3.7]: css({
    ...levelSquare,
    backgroundColor: "#ff9925",
  }),
  [3.6]: css({
    ...levelSquare,
    backgroundColor: "#ff6500",
  }),
  [2.5]: css({
    ...levelSquare,
    backgroundColor: "#cc0000",
  }),
};

export function Battery({
  title,
  voltage,
  cells = 1,
}: {
  title: string;
  voltage: number;
  cells: number;
}) {
  const levels = Object.entries(colors)
    .sort((a, b) => (a[0] < b[0] ? -1 : 1))
    .map(([key, value]) =>
      parseFloat(key) <= voltage / cells
        ? value
        : css({ visibility: "hidden", ...levelSquare })
    );
  return (
    <div css={container}>
      <Typography variant="subtitle1">{title}:</Typography>
      <div css={data}>
        <div css={text}>
          <Typography variant="h4">{voltage.toFixed(2)}V</Typography>
        </div>
        <div css={squaresContainer}>
          {levels.map((level, index) => (
            <div key={index} css={level}></div>
          ))}
        </div>
      </div>
    </div>
  );
}
