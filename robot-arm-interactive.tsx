import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Slider } from '@/components/ui/slider';

const RobotArmVisualization = () => {
  const [elevation, setElevation] = useState(0);
  const [angles, setAngles] = useState({
    shoulder: -49,
    elbow: 100,
    wrist: 50
  });

  // Constants
  const SEGMENT_LENGTH = 100;
  const CLAW_LENGTH = 170;
  const MIN_ANGLE = -125;
  const MAX_ANGLE = 125;

  // Clamp angle to constraints
  const clampAngle = (angle) => {
    return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
  };

  // Calculate angles for target elevation
  useEffect(() => {
    const adjustAngles = () => {
      // For the claw to point at target elevation, the sum of all angles should be (90 - elevation)
      const targetTotalAngle = 90 - elevation;
      
      // Start with shoulder at half the target total
      let newShoulder = clampAngle(-targetTotalAngle * 0.4);
      
      // Use elbow to bring us most of the way there
      let newElbow = clampAngle(targetTotalAngle * 0.8);
      
      // Wrist needs to make the final angle correct
      // Total angle needs to be targetTotalAngle
      // So wrist = targetTotalAngle - shoulder - elbow
      let newWrist = clampAngle(targetTotalAngle - newShoulder - newElbow);
      
      setAngles({
        shoulder: Number(newShoulder.toFixed(1)),
        elbow: Number(newElbow.toFixed(1)),
        wrist: Number(newWrist.toFixed(1))
      });
    };

    adjustAngles();
  }, [elevation]);

  // Calculate endpoint positions
  const calculatePoints = () => {
    const shoulderRad = (angles.shoulder) * Math.PI / 180;
    const elbowRad = (angles.elbow) * Math.PI / 180;
    const wristRad = (angles.wrist) * Math.PI / 180;
    
    const elbowX = SEGMENT_LENGTH * Math.sin(shoulderRad);
    const elbowY = -SEGMENT_LENGTH * Math.cos(shoulderRad);
    
    const total_shoulder_elbow = shoulderRad + elbowRad;
    const wristX = elbowX + SEGMENT_LENGTH * Math.sin(total_shoulder_elbow);
    const wristY = elbowY - SEGMENT_LENGTH * Math.cos(total_shoulder_elbow);
    
    const total_angle = shoulderRad + elbowRad + wristRad;
    const clawX = wristX + CLAW_LENGTH * Math.sin(total_angle);
    const clawY = wristY - CLAW_LENGTH * Math.cos(total_angle);
    
    return {
      elbow: { x: elbowX, y: elbowY },
      wrist: { x: wristX, y: wristY },
      claw: { x: clawX, y: clawY }
    };
  };

  const points = calculatePoints();

  const calculateClawElevation = () => {
    const totalAngle = angles.shoulder + angles.elbow + angles.wrist;
    return 90 - totalAngle;
  };

  return (
    <Card className="w-full max-w-2xl">
      <CardHeader>
        <CardTitle>Robot Arm Visualization (0° = vertical)</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">
          <div className="flex items-center space-x-4">
            <span className="min-w-24">Target Elevation: {elevation}°</span>
            <Slider 
              min={-90}
              max={90}
              step={1}
              value={[elevation]}
              onValueChange={(values) => setElevation(values[0])}
              className="w-full"
            />
          </div>
          
          <svg viewBox="-200 -400 800 400" className="w-full h-96 border rounded">
            {/* Grid */}
            <g stroke="#eee" strokeWidth="1">
              <line x1="-200" y1="0" x2="600" y2="0" />
              <line x1="0" y1="-400" x2="0" y2="0" />
            </g>
            
            {/* Target elevation line */}
            <line 
              x1="0" 
              y1="0" 
              x2={Math.cos((90 - elevation) * Math.PI / 180) * 200}
              y2={-Math.sin((90 - elevation) * Math.PI / 180) * 200}
              stroke="#aaa" 
              strokeWidth="1"
              strokeDasharray="5,5"
            />
            
            {/* Base */}
            <rect x="-20" y="-20" width="40" height="40" fill="#666" />
            
            {/* Arm segments */}
            <line x1="0" y1="0" x2={points.elbow.x} y2={points.elbow.y} stroke="#222" strokeWidth="8" />
            <line x1={points.elbow.x} y1={points.elbow.y} x2={points.wrist.x} y2={points.wrist.y} stroke="#222" strokeWidth="8" />
            <line x1={points.wrist.x} y1={points.wrist.y} x2={points.claw.x} y2={points.claw.y} stroke="#f44" strokeWidth="4" />
            
            {/* Joints */}
            <circle cx="0" cy="0" r="10" fill="#44f" />
            <circle cx={points.elbow.x} cy={points.elbow.y} r="10" fill="#44f" />
            <circle cx={points.wrist.x} cy={points.wrist.y} r="10" fill="#44f" />
            
            {/* Angle labels */}
            <text x="30" y="-20" fill="#000">{angles.shoulder}°</text>
            <text x={points.elbow.x + 20} y={points.elbow.y} fill="#000">{angles.elbow}°</text>
            <text x={points.wrist.x + 20} y={points.wrist.y} fill="#000">{angles.wrist}°</text>
          </svg>
          
          <div className="text-sm text-gray-500">
            <div>Current angles (from vertical):</div>
            <div>Shoulder: {angles.shoulder}° CCW</div>
            <div>Elbow: {angles.elbow}° CW</div>
            <div>Wrist: {angles.wrist}° CW</div>
            <div>Current claw elevation: {calculateClawElevation().toFixed(1)}°</div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
};

export default RobotArmVisualization;
