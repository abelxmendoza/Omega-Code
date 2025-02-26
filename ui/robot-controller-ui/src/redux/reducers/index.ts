/*
# File: /src/redux/reducers/index.ts
# Summary:
Combines all reducers into a single root reducer, which serves as the central point 
for managing application state in the Redux store.
*/

import { combineReducers } from '@reduxjs/toolkit'; // Redux utility to combine multiple reducers
import commandLogReducer from './commandLogSlice'; // Reducer for managing command log state
import controlPanelReducer from './controlPanelSlice'; // Reducer for managing control panel state

// Combine all reducers into a root reducer
const rootReducer = combineReducers({
  commandLog: commandLogReducer, // Handles the command log state
  controlPanel: controlPanelReducer, // Handles the control panel state
});

// Export the root reducer to be used in the Redux store configuration
export default rootReducer;
