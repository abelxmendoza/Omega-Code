// File: src/components/CommandLogContext.tsx
import React, { createContext, useState, useContext, ReactNode } from 'react';

// Define the type for the command log context
interface CommandLogContextType {
  commands: string[];
  addCommand: (command: string) => void;
}

// Create the context with an undefined initial value
const CommandLogContext = createContext<CommandLogContextType | undefined>(undefined);

// Custom hook to use the CommandLogContext
export const useCommandLog = (): CommandLogContextType => {
  const context = useContext(CommandLogContext);
  if (!context) {
    throw new Error('useCommandLog must be used within a CommandLogProvider');
  }
  return context;
};

// Provider component that wraps the application and provides the command log context
export const CommandLogProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [commands, setCommands] = useState<string[]>([]);

  // Function to add a command to the log
  const addCommand = (command: string) => {
    console.log(`Adding command to log: ${command}`);
    setCommands((prevCommands) => [command, ...prevCommands]);
  };

  return (
    <CommandLogContext.Provider value={{ commands, addCommand }}>
      {children}
    </CommandLogContext.Provider>
  );
};
