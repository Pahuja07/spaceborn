import mongoose from 'mongoose';

const simulationRunSchema = new mongoose.Schema({
  runId: { type: String, required: true, unique: true },
  stackName: { type: String, required: true },
  stackVersion: { type: String, default: 'v1.0.0' },
  scenario: { type: String, required: true },
  status: { 
    type: String, 
    enum: ['Queued', 'Running', 'Passed', 'Failed', 'Error'], 
    default: 'Queued' 
  },
  score: { type: Number, default: null },
  duration: { type: String, default: '--' },
  startedBy: { type: String, required: true },
  logs: [{ type: String }],
  user: { type: mongoose.Schema.Types.ObjectId, ref: 'User' }
}, { timestamps: true });

const SimulationRun = mongoose.model('SimulationRun', simulationRunSchema);
export default SimulationRun;