import React, { useEffect, useRef, useState } from "react";
import axios from "axios";
import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

const MapViewer = ({ apiUrl }) => {
  const canvasRef = useRef(null);
  const [mapData, setMapData] = useState(null);
  const [selectedMap, setSelectedMap] = useState("map");

  // Fetch map data from API
  useEffect(() => {
    const fetchMaps = async () => {
      try {
        const response = await axios.get(apiUrl);
        setMapData(response.data);
      } catch (error) {
        console.error("Error fetching map data:", error);
      }
    };

    fetchMaps();
    const interval = setInterval(fetchMaps, 250); // Refresh every 5s
    return () => clearInterval(interval);
  }, [apiUrl]);

  // Draw the map and robot on canvas
  useEffect(() => {
    if (!mapData) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const map = mapData[selectedMap];
    const robot = mapData.transform;

    if (!map) return;

    const width = map.info.width;
    const height = map.info.height;
    const resolution = map.info.resolution;
    const origin = map.info.origin;

    canvas.width = width;
    canvas.height = height;

    const imageData = ctx.createImageData(width, height);
    const data = map.data;

    // Draw Occupancy Grid
    for (let i = 0; i < data.length; i++) {
      const value = data[i];
      const idx = i * 4;

      if (value === -1) {
        imageData.data[idx] = 128; // Unknown (Gray)
        imageData.data[idx + 1] = 128;
        imageData.data[idx + 2] = 128;
      } else {
        const color = 255 - (value / 100) * 255; // 0 = White, 100 = Black
        imageData.data[idx] = color;
        imageData.data[idx + 1] = color;
        imageData.data[idx + 2] = color;
      }
      imageData.data[idx + 3] = 255; // Alpha
    }
    ctx.putImageData(imageData, 0, 0);

    // Draw Robot Position
    // Step 3: Draw Robot
    const robotX = (robot.translation.x - origin.x) / resolution;
    const robotY = height - (robot.translation.y - origin.y) / resolution;

    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(robotX, robotY, 5, 0, 2 * Math.PI);
    ctx.fill();

    // Step 4: Draw Robot Orientation (Optional)
    const angle = Math.atan2(
      2 *
        (robot.rotation.w * robot.rotation.z +
          robot.rotation.x * robot.rotation.y),
      1 - 2 * (robot.rotation.y ** 2 + robot.rotation.z ** 2)
    );

    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(robotX + 20 * Math.cos(angle), robotY - 20 * Math.sin(angle));
    ctx.stroke();

  }, [mapData, selectedMap]);

  return (
    <div className="p-4">
      <h1 className="text-xl font-bold mb-2">Map Viewer</h1>
      <div className="flex gap-2 mb-4">
        <button onClick={() => setSelectedMap("map")} className="px-4 py-2 bg-blue-500 text-white rounded">
          Main Map
        </button>
        <button onClick={() => setSelectedMap("global_costmap")} className="px-4 py-2 bg-blue-500 text-white rounded">
          Global Costmap
        </button>
        <button onClick={() => setSelectedMap("local_costmap")} className="px-4 py-2 bg-blue-500 text-white rounded">
          Local Costmap
        </button>
      </div>
      <TransformWrapper style={{width:"auto"}}>
        <TransformComponent>
          <div style={{ width: "80vw", height: "80vh", display: "flex", justifyContent: "center", alignItems: "center" }}>
          <canvas ref={canvasRef}/>
          </div>
        </TransformComponent>
      </TransformWrapper>
    </div>
  );
};

export default MapViewer;
