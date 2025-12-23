// Vercel Serverless Function for Server-Side Computation

// Handle different HTTP methods
module.exports = async (req, res) => {
  const { method, url, query, body } = req;

  // Enable CORS for all requests
  res.setHeader('Access-Control-Allow-Credentials', true);
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,OPTIONS,POST,PUT,DELETE');
  res.setHeader(
    'Access-Control-Allow-Headers',
    'X-CSRF-Token, X-Requested-With, Accept, Accept-Version, Content-Length, Content-MD5, Content-Type, Date, X-Api-Version'
  );

  // Handle preflight requests
  if (method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  // Extract API path
  const path = url.replace('/api', '');

  try {
    if (path.startsWith('/robotics/calculate') && method === 'GET') {
      // Example: Calculate robot kinematics
      const { jointAngles, robotType } = query;

      // Simulate server-side computation
      let result = null;

      if (robotType === 'arm') {
        // Simple forward kinematics calculation example
        const angles = JSON.parse(jointAngles);
        const x = Math.cos(angles[0]) * Math.cos(angles[1]);
        const y = Math.sin(angles[0]) * Math.cos(angles[1]);
        const z = Math.sin(angles[1]);

        result = { x, y, z, type: 'forward_kinematics' };
      } else if (robotType === 'mobile') {
        // Simple path planning calculation
        const [startX, startY] = JSON.parse(jointAngles);
        const distance = Math.sqrt(startX * startX + startY * startY);
        result = { distance, path: `(${startX}, ${startY}) to (0, 0)`, type: 'path_calculation' };
      }

      res.json({
        success: true,
        data: result,
        timestamp: new Date().toISOString()
      });
    }
    else if (path.startsWith('/robotics/simulate') && method === 'POST') {
      // Example: Simulate robot behavior
      const { parameters, simulationType } = body;

      // Simulate different types of robotics computations
      let result = {};

      switch (simulationType) {
        case 'physics':
          // Physics simulation
          const mass = parameters.mass || 1;
          const acceleration = parameters.acceleration || 1;
          const force = mass * acceleration;
          result = { force, energy: 0.5 * mass * acceleration * acceleration };
          break;

        case 'path':
          // Path planning simulation
          const start = parameters.start || [0, 0];
          const end = parameters.end || [10, 10];
          const distance = Math.sqrt(
            Math.pow(end[0] - start[0], 2) +
            Math.pow(end[1] - start[1], 2)
          );
          result = { distance, path: [start, end], time: distance / 5 }; // assuming 5 units/sec
          break;

        case 'control':
          // Control system simulation
          const setpoint = parameters.setpoint || 0;
          const current = parameters.current || 0;
          const error = setpoint - current;
          const kp = parameters.kp || 1;
          const output = error * kp;
          result = { error, output, correction: output };
          break;

        default:
          throw new Error('Invalid simulation type');
      }

      res.json({
        success: true,
        data: result,
        simulationType,
        timestamp: new Date().toISOString()
      });
    }
    else if (path.startsWith('/robotics/simulations') && method === 'GET') {
      // Get available simulation types
      res.json({
        success: true,
        availableSimulations: [
          {
            id: 'physics',
            name: 'Physics Simulation',
            description: 'Calculate forces, torques, and motion parameters'
          },
          {
            id: 'path',
            name: 'Path Planning',
            description: 'Calculate optimal paths and distances'
          },
          {
            id: 'control',
            name: 'Control Systems',
            description: 'Simulate PID control and feedback systems'
          }
        ],
        timestamp: new Date().toISOString()
      });
    }
    else if (path.startsWith('/health') && method === 'GET') {
      // Health check endpoint
      res.json({
        status: 'healthy',
        uptime: process.uptime ? process.uptime() : 'N/A',
        timestamp: new Date().toISOString()
      });
    }
    else {
      // Handle 404 for unknown API routes
      res.status(404).json({
        success: false,
        error: 'API endpoint not found',
        path: path
      });
    }
  } catch (error) {
    console.error('API Error:', error);
    res.status(500).json({
      success: false,
      error: error.message || 'Internal server error'
    });
  }
};