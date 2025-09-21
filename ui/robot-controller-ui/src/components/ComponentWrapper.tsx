import React from 'react';
import ErrorBoundary from './ErrorBoundary';

interface ComponentWrapperProps {
  children: React.ReactNode;
  title?: string;
  className?: string;
}

const ComponentWrapper: React.FC<ComponentWrapperProps> = ({ 
  children, 
  title,
  className = "p-4 bg-gray-800 rounded-lg"
}) => {
  return (
    <ErrorBoundary>
      <div className={className}>
        {title && (
          <h3 className="text-white font-bold mb-4">{title}</h3>
        )}
        {children}
      </div>
    </ErrorBoundary>
  );
};

export default ComponentWrapper;
