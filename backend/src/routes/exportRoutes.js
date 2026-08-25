import express from 'express';
import { exportRunAsJson, exportScoresAsCsv } from '../controllers/exportController.js';

const router = express.Router();

router.get('/run/:runId/json', exportRunAsJson);
router.get('/scores/csv', exportScoresAsCsv);

export default router;