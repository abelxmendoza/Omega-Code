/*
# File: /src/redux/reducers/commandLogSlice.ts
# Summary:
Manages the state for the command log. This slice provides actions to add commands to the log 
and clear all commands, allowing for tracking and resetting the command history.
*/

import { createSlice, PayloadAction } from '@reduxjs/toolkit';

interface CommandLogState {
  commands: string[]; // Array to store the command log
}

// Initial state of the command log
const initialState: CommandLogState = {
  commands: [],
};

const commandLogSlice = createSlice({
  name: 'commandLog', // Slice name
  initialState,
  reducers: {
    // Add a command to the log
    addCommand(state, action: PayloadAction<string>) {
      state.commands = [...state.commands, action.payload];
    },
    // Clear all commands from the log
    clearCommands(state) {
      state.commands = [];
    },
  },
});

// Export the actions to be used in components or middleware
export const { addCommand, clearCommands } = commandLogSlice.actions;

// Export the reducer to be included in the store
export default commandLogSlice.reducer;
