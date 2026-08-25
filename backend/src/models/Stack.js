import mongoose from 'mongoose';

const stackSchema = new mongoose.Schema({
  name: { type: String, required: true },
  version: { type: String, required: true },
  rosVersion: { type: String, required: true }, // e.g., "ROS 2 HUMBLE"
  tags: [{ type: String }],                    // e.g., ["SLAM", "LIDAR-3D"]
  totalEvaluatedRuns: { type: Number, default: 0 },
  historicalBestScore: { type: Number, default: 0.0 },
  lastRunStatus: { 
    type: String, 
    enum: ['Passed', 'Failed', 'Running', 'Pending'], 
    default: 'Passed' 
  },
  isActive: { type: Boolean, default: true }
}, { timestamps: true });

export default mongoose.model('Stack', stackSchema);
