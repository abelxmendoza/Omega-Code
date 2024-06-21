import React, { createContext, useState, useContext, ReactNode } from 'react';

interface CommandLogContextType {
  commands: string[];
  addCommand: (command: string) => void;
}

const CommandLogContext = createContext<CommandLogContextType | undefined>(undefined);

export const useCommandLog = (): CommandLogContextType => {
  const context = useContext(CommandLogContext);
  if (!context) {
    throw new Error('useCommandLog must be used within a CommandLogProvider');
  }
  return context;
};

export const CommandLogProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [commands, setCommands] = useState<string[]>([]);

  const addCommand = (command: string) => {
    setCommands((prevCommands) => [command, ...prevCommands]);
  };

  return (
    <CommandLogContext.Provider value={{ commands, addCommand }}>
      {children}
    </CommandLogContext.Provider>
  );
};
