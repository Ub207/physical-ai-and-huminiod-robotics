import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const RobotSimulation = ({ simulationType = 'basic', config = {} }) => {
  const [simulationState, setSimulationState] = useState('stopped');
  const [robotData, setRobotData] = useState({
    position: { x: 0, y: 0, z: 0 },
    joints: [],
    sensors: {}
  });

  useEffect(() => {
    // Initialize simulation based on type
    initializeSimulation();
  }, [simulationType]);

  const initializeSimulation = () => {
    // Setup simulation based on type
    console.log(`Initializing ${simulationType} simulation`);
  };

  const startSimulation = () => {
    setSimulationState('running');
    // Start simulation logic here
  };

  const stopSimulation = () => {
    setSimulationState('stopped');
    // Stop simulation logic here
  };

  const resetSimulation = () => {
    setSimulationState('stopped');
    setRobotData({
      position: { x: 0, y: 0, z: 0 },
      joints: [],
      sensors: {}
    });
  };

  return (
    <div className="simulation-container">
      <div className="simulation-header">
        <h3>{simulationType} Simulation</h3>
        <div className="simulation-controls">
          <button
            onClick={startSimulation}
            disabled={simulationState === 'running'}
            className={`button ${simulationState === 'running' ? 'button--secondary' : 'button--primary'}`}
          >
            {simulationState === 'running' ? 'Running' : 'Start'}
          </button>
          <button
            onClick={stopSimulation}
            disabled={simulationState === 'stopped'}
            className="button button--secondary"
          >
            Stop
          </button>
          <button
            onClick={resetSimulation}
            className="button button--outline"
          >
            Reset
          </button>
        </div>
      </div>

      <div className="simulation-viewport">
        <BrowserOnly>
          {() => {
            // This would contain the actual simulation component
            // For now, we'll show a placeholder
            return (
              <div className="simulation-placeholder">
                <div className="simulation-robot-model">
                  {/* 3D robot visualization would go here */}
                  <div className="robot-visualization">
                    <div className="robot-base"></div>
                    <div className="robot-arm"></div>
                    <div className="robot-sensor"></div>
                  </div>
                </div>

                <div className="simulation-info">
                  <h4>Robot State</h4>
                  <div className="state-data">
                    <p><strong>Position:</strong> ({robotData.position.x}, {robotData.position.y}, {robotData.position.z})</p>
                    <p><strong>Status:</strong> {simulationState}</p>
                    <p><strong>Type:</strong> {simulationType}</p>
                  </div>
                </div>
              </div>
            );
          }}
        </BrowserOnly>
      </div>

      <div className="simulation-description">
        <p>This interactive simulation demonstrates {simulationType} concepts in Physical AI and Robotics.</p>
        <ul>
          <li>Click "Start" to begin the simulation</li>
          <li>Observe how the robot responds to different inputs</li>
          <li>Experiment with different parameters</li>
        </ul>
      </div>
    </div>
  );
};

export default RobotSimulation;