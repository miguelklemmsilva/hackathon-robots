"use client";

import { useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Line } from "react-chartjs-2";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

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
        if (parsedMessage.payload) {
          const payload = parsedMessage.payload;

          if (payload.type === "sensor_data") {
            setSensorData((prevData) => [
              ...prevData.slice(-19), // Keep the latest 20 records
              payload,
            ]);
          }
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

  // Prepare data for Heading (Inertial Unit)
  const headingChartData = {
    labels: sensorData.map((data) => data.timestamp),
    datasets: [
      {
        label: "Inertial Unit Heading (°)",
        data: sensorData.map((data) => data.inertial_unit.heading),
        borderColor: "rgba(255, 206, 86, 1)",
        backgroundColor: "rgba(255, 206, 86, 0.2)",
        tension: 0.3,
      },
    ],
  };

  return (
    <div>
      <main>
        <h1>Spot Sensor Data Dashboard</h1>
        <p>
          Status: <strong>{connectionStatus}</strong>
        </p>

        {/* Inertial Unit Heading Chart */}
        <div>
          <h2>Inertial Unit Heading Over Time</h2>
          <Line data={headingChartData} />
        </div>

        {/* Real-Time Sensor Data Table */}
        <div>
          <h2>Real-Time Sensor Data</h2>
          {sensorData.length > 0 ? (
            <table>
              <thead>
                <tr>
                  <th>Timestamp</th>
                  <th>GPS (x, y, z)</th>
                  <th>LiDAR Min Distance (m)</th>
                  <th>Inertial Unit Heading (°)</th>
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
                    <td>{data.inertial_unit.heading.toFixed(2)}°</td>
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