import { createSlice, PayloadAction } from '@reduxjs/toolkit';

interface ControlPanelState {
  lastCommand: string | null;
}

const initialState: ControlPanelState = {
  lastCommand: null,
};

const controlPanelSlice = createSlice({
  name: 'controlPanel',
  initialState,
  reducers: {
    executeCommand: (state, action: PayloadAction<string>) => {
      state.lastCommand = action.payload;
    },
  },
});

export const { executeCommand } = controlPanelSlice.actions;
export default controlPanelSlice.reducer;

