import React from 'react';
import AnimationInitializer from '../components/AnimationInitializer';

// Custom root component to wrap the entire app
// This can be used to provide custom context or state to the entire app
export default function Root({children}) {
  return (
    <div className="physical-ai-book">
      <AnimationInitializer />
      {children}
    </div>
  );
}