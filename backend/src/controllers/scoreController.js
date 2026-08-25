import Score from '../models/ScoreReport.js';

// Get leaderboard & evaluated run scores
export const getAllScores = async (req, res) => {
  try {
    const scores = await Score.find()
      .populate('stackId', 'name version rosVersion')
      .populate('scenarioId', 'title difficulty')
      .sort({ overallScore: -1 });

    res.status(200).json({
      success: true,
      count: scores.length,
      data: scores
    });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};

// Record a new run score
export const recordScore = async (req, res) => {
  try {
    const newScore = await Score.create(req.body);
    res.status(201).json({ success: true, data: newScore });
  } catch (error) {
    res.status(400).json({ success: false, message: error.message });
  }
};

// Get peak score overview stats
export const getLeaderboardStats = async (req, res) => {
  try {
    const topScore = await Score.findOne()
      .populate('stackId', 'name version')
      .sort({ overallScore: -1 });

    res.status(200).json({
      success: true,
      peakScore: topScore ? topScore.overallScore : 0,
      topStack: topScore ? `${topScore.stackId.name} ${topScore.stackId.version}` : 'N/A'
    });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};
