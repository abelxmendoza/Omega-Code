/*
# File: /src/redux/store.ts
# Summary:
Configures and initializes the Redux store using the root reducer. 
This store acts as the central state management system for the application.
*/

import { configureStore } from '@reduxjs/toolkit';
import rootReducer from './reducers'; // Import the combined root reducer

// Configure the Redux store
const store = configureStore({
  reducer: rootReducer, // Use the root reducer
});

// Export types for state and dispatch to ensure type safety
export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;

// Export the configured store
export default store;
