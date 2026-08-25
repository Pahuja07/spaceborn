import express from 'express';
import { 
  getAllScores, 
  recordScore, 
  getLeaderboardStats 
} from '../controllers/scoreController.js';

const router = express.Router();

router.route('/')
  .get(getAllScores)
  .post(recordScore);

router.get('/leaderboard/peak', getLeaderboardStats);

export default router;