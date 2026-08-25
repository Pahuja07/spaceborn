import express from 'express';
import cors from 'cors';

import dotenv from 'dotenv';

// Import Configurations & Database
import connectDB from './config/db.js';
import scenarioRoutes from "./routes/scenarioRoutes.js"
// Import Middleware
import stackRoutes from './routes/stackRoutes.js'
// Import Routes
import runRoutes from './routes/runRoutes.js';
import authRoutes from './routes/authRoutes.js';
import ScoreRouter from './routes/scoreRoutes.js'
import exportRoutes from './routes/exportRoutes.js';
import { errorHandler } from './middlewares/errorHandler.js';

const PORT = process.env.PORT || 8000;

dotenv.config();

const app = express();

// Connect to Database
connectDB();

// Global Middlewares
app.use(cors());
app.use(express.json({ limit: '10mb' })); // Allows parsing JSON payloads from simulation logs
app.use(express.urlencoded({ extended: true }));


// Health Check Route
app.get('/health', (req, res) => {
  res.status(200).json({ status: 'ok', service: 'SpaceBorn Backend API' });
});

// Mount API Routes
app.use('/api/v1/auth', authRoutes);
app.use('/api/v1/runs', runRoutes);
app.use('/api/v1/scenarios', scenarioRoutes);
app.use('/api/v1/stacks', stackRoutes);
app.use('/api/v1/scores', ScoreRouter);
app.use('/api/v1/export', exportRoutes);
// Catch 404 Route Not Found
app.use((req, res) => res.status(404).json({ success: false, message: 'Route not found' }));
app.use(errorHandler);

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

// Centralized Error Handling Middleware

export default app;
