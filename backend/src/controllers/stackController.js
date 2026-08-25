import Stack from '../models/Stack.js'

// Get all navigation stacks & library stats
export const getAllStacks = async (req, res) => {
  try {
    const stacks = await Stack.find();
    
    // Calculate dashboard overview stats dynamically
    const totalActive = stacks.filter(s => s.isActive).length;
    const peakScoreObj = stacks.reduce((max, s) => (s.historicalBestScore > max.score ? { score: s.historicalBestScore, name: `${s.name} ${s.version}` } : max), { score: 0, name: '' });
    const totalRuns = stacks.reduce((sum, s) => sum + s.totalEvaluatedRuns, 0);

    res.status(200).json({
      success: true,
      stats: {
        totalActiveStacks: totalActive,
        peakScore: peakScoreObj.score,
        peakScoreStack: peakScoreObj.name,
        systemTotalRunCount: totalRuns
      },
      count: stacks.length,
      data: stacks
    });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};

// Get single stack by ID
export const getStackById = async (req, res) => {
  try {
    const stack = await Stack.findById(req.params.id);
    if (!stack) return res.status(404).json({ success: false, message: 'Stack not found' });
    res.status(200).json({ success: true, data: stack });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};

// Create / Upload new stack
export const createStack = async (req, res) => {
  try {
    const newStack = await Stack.create(req.body);
    res.status(201).json({ success: true, data: newStack });
  } catch (error) {
    res.status(400).json({ success: false, message: error.message });
  }
};