// src/store.ts
import { configureStore, createSlice, PayloadAction } from '@reduxjs/toolkit';

interface CommandState {
  commands: string[];
}

const initialState: CommandState = {
  commands: [],
};

const commandSlice = createSlice({
  name: 'commands',
  initialState,
  reducers: {
    addCommand(state, action: PayloadAction<string>) {
      state.commands.push(action.payload);
    },
  },
});

export const { addCommand } = commandSlice.actions;

const store = configureStore({
  reducer: {
    commands: commandSlice.reducer,
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;
export default store;
