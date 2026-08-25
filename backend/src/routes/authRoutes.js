import express from 'express';
import { register, login, getMe, generateApiKey } from '../controllers/authController.js';
import { protect } from '../middlewares/authMiddleware.js';

const router = express.Router();

// Public Routes
router.post('/register', register);
router.post('/login', login);

// Protected Routes (Dashboard UI)
router.get('/me', protect, getMe);
router.post('/generate-api-key', protect, generateApiKey);

export default router;