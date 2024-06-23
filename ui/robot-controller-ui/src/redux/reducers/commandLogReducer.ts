// src/redux/reducers/commandLogReducer.ts
import { createSlice, PayloadAction } from '@reduxjs/toolkit';

interface CommandLogState {
  commands: string[];
}

const initialState: CommandLogState = {
  commands: [],
};

const commandLogSlice = createSlice({
  name: 'commandLog',
  initialState,
  reducers: {
    addCommand(state, action: PayloadAction<string>) {
      state.commands.push(action.payload);
    },
  },
});

export const { addCommand } = commandLogSlice.actions;
export default commandLogSlice.reducer;
