import mongoose from 'mongoose';

const scenarioSchema = new mongoose.Schema({
  title: { type: String, required: true },
  version: { type: String, required: true },
  description: { type: String, required: true },
  duration: { type: String, required: true },
  difficulty: { 
    type: String, 
    enum: ['EASY', 'MEDIUM', 'HARD', 'EXPERT', 'DEV'], 
    required: true 
  },
  sensorSuite: { type: String, required: true },
  goalSpec: { type: String, required: true },
  signalLossCurve: [{ type: Number }], // Array representing preview bar heights
  isCustomYaml: { type: Boolean, default: false }
}, { timestamps: true });

export default mongoose.model('Scenario', scenarioSchema);
