import mongoose from 'mongoose';

const scoreSchema = new mongoose.Schema({
  stackId: { 
    type: mongoose.Schema.Types.ObjectId, 
    ref: 'Stack', 
    required: true 
  },
  scenarioId: { 
    type: mongoose.Schema.Types.ObjectId, 
    ref: 'Scenario', 
    required: true 
  },
  runId: { type: String, required: true, unique: true },
  overallScore: { type: Number, required: true }, // e.g., 94.2
  status: { 
    type: String, 
    enum: ['Passed', 'Failed', 'Running'], 
    default: 'Passed' 
  },
  metrics: {
    positionError: { type: Number }, // in meters
    headingError: { type: Number },  // in degrees
    pathEfficiency: { type: Number },// percentage
    completionTime: { type: Number } // in seconds
  },
  executedAt: { type: Date, default: Date.now }
}, { timestamps: true });

export default mongoose.model('Score', scoreSchema);