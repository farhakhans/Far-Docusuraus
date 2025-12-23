import React, { useState } from 'react';
import Layout from '@theme/Layout';

const RoboticsCalculator = () => {
  const [result, setResult] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  // Function to call the server-side calculation API
  const calculateKinematics = async (jointAngles, robotType) => {
    setLoading(true);
    setError('');

    try {
      const response = await fetch(`/api/robotics/calculate?jointAngles=${encodeURIComponent(JSON.stringify(jointAngles))}&robotType=${robotType}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      const data = await response.json();

      if (data.success) {
        setResult(data.data);
      } else {
        setError(data.error || 'Calculation failed');
      }
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  // Function to call the server-side simulation API
  const runSimulation = async (parameters, simulationType) => {
    setLoading(true);
    setError('');

    try {
      const response = await fetch('/api/robotics/simulate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ parameters, simulationType }),
      });

      const data = await response.json();

      if (data.success) {
        setResult(data.data);
      } else {
        setError(data.error || 'Simulation failed');
      }
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  const handleArmCalculation = () => {
    calculateKinematics([1.57, 0.785], 'arm'); // 90° and 45° in radians
  };

  const handleMobileCalculation = () => {
    calculateKinematics([5, 5], 'mobile'); // Point at (5,5)
  };

  const handlePhysicsSimulation = () => {
    runSimulation({ mass: 2, acceleration: 9.8 }, 'physics');
  };

  const handlePathSimulation = () => {
    runSimulation({ start: [0, 0], end: [10, 10] }, 'path');
  };

  const handleControlSimulation = () => {
    runSimulation({ setpoint: 10, current: 5, kp: 1.5 }, 'control');
  };

  return (
    <Layout title="Robotics Calculator" description="Server-side computation for robotics calculations">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Robotics Server-Side Computation</h1>
            <p>
              This page demonstrates server-side computation capabilities for robotics calculations.
              All calculations are performed on the server for accuracy and security.
            </p>

            <div className="margin-vert--lg">
              <h2>Kinematics Calculations</h2>
              <button
                className="button button--primary margin-right--sm margin-bottom--sm"
                onClick={handleArmCalculation}
                disabled={loading}
              >
                Calculate Arm Kinematics
              </button>
              <button
                className="button button--secondary margin-bottom--sm"
                onClick={handleMobileCalculation}
                disabled={loading}
              >
                Calculate Mobile Robot Path
              </button>
            </div>

            <div className="margin-vert--lg">
              <h2>Physics Simulations</h2>
              <button
                className="button button--info margin-right--sm margin-bottom--sm"
                onClick={handlePhysicsSimulation}
                disabled={loading}
              >
                Physics Simulation (F=ma)
              </button>
              <button
                className="button button--info margin-right--sm margin-bottom--sm"
                onClick={handlePathSimulation}
                disabled={loading}
              >
                Path Planning
              </button>
              <button
                className="button button--info margin-bottom--sm"
                onClick={handleControlSimulation}
                disabled={loading}
              >
                Control System
              </button>
            </div>

            {loading && (
              <div className="alert alert--info" role="alert">
                <strong>Processing...</strong> Performing server-side computation.
              </div>
            )}

            {error && (
              <div className="alert alert--danger" role="alert">
                <strong>Error:</strong> {error}
              </div>
            )}

            {result && !loading && (
              <div className="card margin-vert--lg">
                <div className="card__header">
                  <h3>Computation Result</h3>
                </div>
                <div className="card__body">
                  <pre>{JSON.stringify(result, null, 2)}</pre>
                </div>
              </div>
            )}

            <div className="margin-vert--lg">
              <h2>API Endpoints</h2>
              <ul>
                <li><code>GET /api/robotics/calculate?jointAngles=[...]&robotType=arm|mobile</code> - Forward kinematics</li>
                <li><code>POST /api/robotics/simulate</code> - Physics, path, and control simulations</li>
                <li><code>GET /api/robotics/simulations</code> - Available simulation types</li>
                <li><code>GET /api/health</code> - Health check</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default RoboticsCalculator;