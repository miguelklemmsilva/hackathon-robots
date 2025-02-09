"use client";

import { useCallback, useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Line, Scatter } from "react-chartjs-2";
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
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";

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
  const [xCoordinate, setXCoordinate] = useState<string>("");
  const [yCoordinate, setYCoordinate] = useState<string>("");

  const { lastMessage, readyState, sendMessage } = useWebSocket(socketUrl!, {
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

  const handleSendCoordinates = () => {
    if (!xCoordinate || !yCoordinate) {
      alert("Please enter both X and Y coordinates.");
      return;
    }

    const message = {
      action: "broadcast",
      payload: {
        type: "move_command",
        coordinates: {
          x: parseFloat(xCoordinate),
          y: parseFloat(yCoordinate),
        },
      },
    };

    sendMessage(JSON.stringify(message));
    console.log("Sent move command:", message);
  };

  const headingChartData = {
    labels: sensorData.map((data) => {
      const date = new Date(data.timestamp * 1000); // Convert epoch to milliseconds
      return date.toLocaleTimeString("en-GB", {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      });
    }),
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
    <div className="min-h-screen bg-gray-100 p-8">
      <main className="max-w-7xl mx-auto">
        <h1 className="text-3xl font-bold mb-4">Spot Sensor Data Dashboard</h1>
        <p className="text-lg mb-8">
          Status: <strong className="text-blue-500">{connectionStatus}</strong>
        </p>

        {/* Two-Column Layout */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
          {/* Chart Section */}
          <div className="bg-white p-6 rounded-lg shadow-md">
            <h2 className="text-xl font-semibold mb-4">
              Inertial Unit Heading Over Time
            </h2>
            <Line data={headingChartData} />
          </div>

          {/* GPS Position Radar (Scatter Chart) */}
          <div className="bg-white p-6 rounded-lg shadow-md">
            <h2 className="text-xl font-semibold mb-4">Live Radar Map (GPS)</h2>
            <Scatter
              data={{
                datasets: [
                  {
                    label: "Spot GPS Position",
                    data: sensorData.map((data) => ({
                      x: data.gps.x,
                      y: data.gps.z,
                    })),
                    backgroundColor: "rgba(75, 192, 192, 0.6)",
                    borderColor: "rgba(75, 192, 192, 1)",
                    pointRadius: 5,
                  },
                ],
              }}
              options={{
                scales: {
                  x: {
                    type: "linear",
                    position: "bottom",
                    title: { display: true, text: "East-West Position (m)" },
                  },
                  y: {
                    title: { display: true, text: "North-South Position (m)" },
                  },
                },
              }}
            />
          </div>
        </div>

        {/* Command Input Section */}
        <div className="bg-white p-6 mt-8 rounded-lg shadow-md">
          <h2 className="text-xl font-semibold mb-4">Send Move Command</h2>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <Input
              type="number"
              placeholder="Enter X Coordinate"
              value={xCoordinate}
              onChange={(e) => setXCoordinate(e.target.value)}
            />
            <Input
              type="number"
              placeholder="Enter Y Coordinate"
              value={yCoordinate}
              onChange={(e) => setYCoordinate(e.target.value)}
            />
            <Button
              className={`bg-blue-500 hover:bg-blue-600 active:bg-blue-700 text-white font-semibold px-6 py-2 rounded-lg transition duration-300 ease-in-out transform hover:scale-105 active:scale-95 shadow-md ${
                readyState !== ReadyState.OPEN
                  ? "opacity-50 cursor-not-allowed"
                  : ""
              }`}
              onClick={handleSendCoordinates}
              disabled={readyState !== ReadyState.OPEN}
            >
              Send Command
            </Button>
          </div>
        </div>

        {/* Real-Time Sensor Data Table */}
        <div className="bg-white p-6 mt-8 rounded-lg shadow-md overflow-auto max-h-[500px]">
          <h2 className="text-xl font-semibold mb-4">Real-Time Sensor Data</h2>
          {sensorData.length > 0 ? (
            <table className="table-auto w-full border-collapse border border-gray-200">
              <thead>
                <tr className="bg-gray-100">
                  <th className="border border-gray-300 px-4 py-2">
                    Timestamp
                  </th>
                  <th className="border border-gray-300 px-4 py-2">
                    GPS (x, y, z)
                  </th>
                  <th className="border border-gray-300 px-4 py-2">
                    LiDAR Min Distance (m)
                  </th>
                  <th className="border border-gray-300 px-4 py-2">
                    Inertial Unit Heading (°)
                  </th>
                </tr>
              </thead>
              <tbody>
                {sensorData.map((data, idx) => (
                  <tr
                    key={idx}
                    className={`${idx % 2 === 0 ? "bg-white" : "bg-gray-50"}`}
                  >
                    <td className="border border-gray-300 px-4 py-2">
                      {new Date(data.timestamp * 1000).toLocaleTimeString()}
                    </td>
                    <td className="border border-gray-300 px-4 py-2">
                      {data.gps.x.toFixed(2)}, {data.gps.y.toFixed(2)},{" "}
                      {data.gps.z.toFixed(2)}
                    </td>
                    <td className="border border-gray-300 px-4 py-2">
                      {data.lidar?.min_distance != null &&
                      !isNaN(data.lidar.min_distance)
                        ? `${parseFloat(data.lidar.min_distance).toFixed(2)} m`
                        : "N/A"}
                    </td>
                    <td className="border border-gray-300 px-4 py-2">
                      {data.inertial_unit.heading.toFixed(2)}°
                    </td>
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
