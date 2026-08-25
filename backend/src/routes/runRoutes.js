import express from 'express';
import { body } from 'express-validator';
import { validateRequest } from '../middlewares/ValidateRequest.js';
import { getRuns, createRun, getRunById, startRun, updateRunTelemetry } from '../controllers/runController.js';
import { protect, protectSimulator } from '../middlewares/authMiddleware.js';

const router = express.Router();

// Rule set for creating a new simulation run
const validateRunInput = [
  body('stackName').trim().notEmpty().withMessage('Stack name is required'),
  body('scenario').trim().notEmpty().withMessage('Scenario is required'),
  validateRequest // Checks the rules above
];

router.route('/')
  .get(protect, getRuns)
  .post(protect, validateRunInput, createRun);

router.get('/:id', protect, getRunById);
router.post('/:id/start', protect, startRun);
router.patch('/:id/telemetry', protectSimulator, updateRunTelemetry);

export default router;
