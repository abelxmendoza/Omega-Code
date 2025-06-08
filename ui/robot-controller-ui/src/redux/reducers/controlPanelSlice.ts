/*
# File: /src/redux/reducers/controlPanelSlice.ts
# Summary:
This slice manages the state for the control panel, tracking the last executed command 
and providing actions to update it. It integrates with the Redux store for centralized state management.
*/

import { createSlice, PayloadAction } from '@reduxjs/toolkit';

// Define the interface for the control panel state
interface ControlPanelState {
  lastCommand: string | null; // Stores the last executed command
}

// Define the initial state of the control panel
const initialState: ControlPanelState = {
  lastCommand: null, // Default to null when no commands have been executed
};

// Create the control panel slice
const controlPanelSlice = createSlice({
  name: 'controlPanel', // Unique name for the slice
  initialState, // Initial state of the slice
  reducers: {
    /**
     * Action: executeCommand
     * Updates the state with the latest executed command.
     * @param state - The current state of the slice.
     * @param action - The dispatched action containing the new command.
     */
    executeCommand: (state, action: PayloadAction<string>) => {
      state.lastCommand = action.payload; // Update the lastCommand with the new value
    },
  },
});

// Export the actions to be used in components or middleware
export const { executeCommand } = controlPanelSlice.actions;

// Export the reducer to be included in the Redux store
export default controlPanelSlice.reducer;
