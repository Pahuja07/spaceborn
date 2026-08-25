
import Scenario from "../models/Scenario.js"
// Get all simulation scenarios
export const getAllScenarios = async (req, res) => {
  try {
    const scenarios = await Scenario.find();
    res.status(200).json({ success: true, count: scenarios.length, data: scenarios });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};

// Get single scenario by ID
export const getScenarioById = async (req, res) => {
  try {
    const scenario = await Scenario.findById(req.params.id);
    if (!scenario) return res.status(404).json({ success: false, message: 'Scenario not found' });
    res.status(200).json({ success: true, data: scenario });
  } catch (error) {
    res.status(500).json({ success: false, message: error.message });
  }
};

// Create or Upload YAML custom scenario
export const createScenario = async (req, res) => {
  try {
    const scenario = await Scenario.create(req.body);
    res.status(201).json({ success: true, data: scenario });
  } catch (error) {
    res.status(400).json({ success: false, message: error.message });
  }
};