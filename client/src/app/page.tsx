"use client";

import { useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";

export default function Home() {
  const [socketUrl] = useState(process.env.NEXT_PUBLIC_WEBSOCKET_URL);
  const [sensorData, setSensorData] = useState<any[]>([]); // Store multiple data points for history

  const { lastMessage, readyState } = useWebSocket(socketUrl!, {
    shouldReconnect: () => true, // Automatically reconnect if the connection closes
  });

  useEffect(() => {
    if (lastMessage !== null) {
      try {
        const parsedMessage = JSON.parse(lastMessage.data);

        // Check if the message type is 'sensor_data'
        if (parsedMessage.type === "sensor_data") {
          setSensorData((prevData) => [
            ...prevData.slice(-9), // Keep the latest 10 records
            {
              timestamp: parsedMessage.timestamp,
              gps: parsedMessage.gps,
              lidar: parsedMessage.lidar,
            },
          ]);
        }
      } catch (error) {
        console.error("Failed to parse message:", error);
      }
    }
  }, [lastMessage]);

  const connectionStatus = {
    [ReadyState.CONNECTING]: "Connecting",
    [ReadyState.OPEN]: "Open",
    [ReadyState.CLOSING]: "Closing",
    [ReadyState.CLOSED]: "Closed",
    [ReadyState.UNINSTANTIATED]: "Uninstantiated",
  }[readyState];

  return (
    <div>
      <main>
        <h1>Spot Sensor Data Client</h1>
        <p>
          Status: <strong>{connectionStatus}</strong>
        </p>

        <div>
          <h2>Real-Time Sensor Data</h2>
          {sensorData.length > 0 ? (
            <table>
              <thead>
                <tr>
                  <th>Timestamp</th>
                  <th>GPS (x, y, z)</th>
                  <th>LiDAR Min Distance (m)</th>
                </tr>
              </thead>
              <tbody>
                {sensorData.map((data, idx) => (
                  <tr key={idx}>
                    <td>{data.timestamp}</td>
                    <td>
                      {data.gps.x.toFixed(2)}, {data.gps.y.toFixed(2)},{" "}
                      {data.gps.z.toFixed(2)}
                    </td>
                    <td>{data.lidar.min_distance.toFixed(2)} m</td>
                  </tr>
                ))}
              </tbody>
            </table>
          ) : (
            <p>No sensor data received yet.</p>
          )}
        </div>
      </main>
    </div>
  );
}
