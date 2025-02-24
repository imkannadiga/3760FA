import React, { useState, useEffect } from "react";
import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

const COSTMAP_API_URL = "http://localhost:10002/api/costmap/download_map"; // Change this

const CostmapViewer = () => {
  const [costmap, setCostmap] = useState(null);

  useEffect(() => {
    fetch(COSTMAP_API_URL)
      .then((res) => res.json())
      .then((data) => setCostmap(data))
      .catch((err) => console.error("Error fetching costmap:", err));
  }, []);

  const renderCostmap = () => {
    if (!costmap) return null;

    console.log("Updated costmap recieved!")

    const { width, height, data } = costmap;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    canvas.width = width;
    canvas.height = height;

    const imageData = ctx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
      let value = data[i];
      let color = [255, 255, 255, 255]; // Default white (should not appear)
  
      if (value === -1) {
          color = [100, 150, 150, 255]; // Blueish-greenish-grayish
      } else if (value === 0) {
          color = [0, 0, 0, 255]; // Black
      } else if (value >= 1 && value <= 98) {
          // Gradient from blue (0, 0, 255) to red (255, 0, 0)
          const ratio = (value - 1) / 97; // Normalize between 0 and 1
          color = [Math.round(255 * ratio), 0, Math.round(255 * (1 - ratio)), 255];
      } else if (value === 99) {
          color = [0, 255, 255, 255]; // Cyan
      } else if (value === 100) {
          color = [128, 0, 128, 255]; // Purple
      } else if (value >= 101 && value <= 127) {
          // Gradient from light green (144, 238, 144) to dark green (0, 128, 0)
          const ratio = (value - 101) / 26;
          color = [
              Math.round(144 * (1 - ratio)), 
              Math.round(128 + (110 * (1 - ratio))), 
              Math.round(144 * (1 - ratio)), 
              255
          ];
      } else if (value < -1) {
          // Gradient from red (255, 0, 0) to yellow (255, 255, 0)
          const ratio = Math.min(Math.abs(value) / 127, 1);
          color = [255, Math.round(255 * ratio), 0, 255];
      }
  
      imageData.data.set(color, i * 4);
  }
  
    ctx.putImageData(imageData, 0, 0);
    return <img src={canvas.toDataURL()} alt="Costmap" style={{ width: "100%", height: "100%", objectFit: "contain" }} />;
  };

  return (
    <div style={{ width: "100vw", height: "100vh" }}>
      <TransformWrapper style={{width:"auto"}}>
        <TransformComponent>
          <div style={{ width: "80vw", height: "80vh", display: "flex", justifyContent: "center", alignItems: "center" }}>
            {renderCostmap()}
          </div>
        </TransformComponent>
      </TransformWrapper>
    </div>
  );
};

export default CostmapViewer;
