import SimulationRun from '../models/SimulationRun.js';
import { startSimulation } from '../services/simulationRunner.js';

// Get all simulation runs with search, status filtering, and pagination
export const getRuns = async (req, res, next) => {
  try {
    const { status, search, page = 1, limit = 10 } = req.query;
    
    // Build query filters matching the UI
    let query = {};
    if (status && status !== 'All') {
      query.status = status;
    }
    if (search) {
      query.$or = [
        { runId: { $regex: search, $options: 'i' } },
        { stackName: { $regex: search, $options: 'i' } },
        { scenario: { $regex: search, $options: 'i' } }
      ];
    }

    const totalRuns = await SimulationRun.countDocuments(query);
    const runs = await SimulationRun.find(query)
      .sort({ createdAt: -1 })
      .skip((page - 1) * limit)
      .limit(Number(limit));

    res.status(200).json({
      success: true,
      count: runs.length,
      totalRuns,
      totalPages: Math.ceil(totalRuns / limit),
      currentPage: Number(page),
      data: runs
    });
  } catch (error) {
    next(error);
  }
};

// Create / Queue a new Simulation Run
export const createRun = async (req, res, next) => {
  try {
    const { stackName, stackVersion, scenario } = req.body;

    // Generate custom Run ID (e.g., RUN-8043)
    const count = await SimulationRun.countDocuments();
    const runId = `RUN-${8030 + count + 1}`;

    const newRun = await SimulationRun.create({
      runId,
      stackName,
      stackVersion,
      scenario,
      status: 'Queued',
      startedBy: req.user.name || 'Automated API',
      user: req.user.id
    });

    res.status(201).json({ success: true, data: newRun });
  } catch (error) {
    next(error);
  }
};

// Get single run details & raw terminal logs
export const getRunById = async (req, res, next) => {
  try {
    const run = await SimulationRun.findOne({ runId: req.params.id });
    if (!run) return res.status(404).json({ message: 'Simulation run not found' });

    res.status(200).json({ success: true, data: run });
  } catch (error) {
    next(error);
  }
};

// SIMULATOR ENDPOINT: Receive real-time telemetry/status updates from ROS/AirSim
export const updateRunTelemetry = async (req, res, next) => {
  try {
    const { status, score, duration, logs } = req.body;
    const { id } = req.params;

    const updateData = {};
    if (status) updateData.status = status;
    if (score !== undefined) updateData.score = score;
    if (duration) updateData.duration = duration;

    // Append logs if sent from simulator
    const run = await SimulationRun.findOneAndUpdate(
      { runId: id },
      { 
        $set: updateData,
        $push: logs ? { logs: { $each: Array.isArray(logs) ? logs : [logs] } } : {}
      },
      { new: true }
    );

    if (!run) return res.status(404).json({ message: 'Simulation run not found' });

    res.status(200).json({ success: true, data: run });
  } catch (error) {
    next(error);
  }
};

// Launch the S04 ROS 2/Gazebo stack for a queued run. This requires Linux/WSL
// with ROS 2 Humble and a built workspace at SIMULATOR_WORKSPACE.
export const startRun = async (req, res, next) => {
  try {
    const run = await SimulationRun.findOne({ runId: req.params.id });
    if (!run) return res.status(404).json({ success: false, message: 'Simulation run not found' });
    if (run.status === 'Running') {
      return res.status(409).json({ success: false, message: 'Simulation is already running' });
    }

    run.status = 'Running';
    run.logs.push(`Launching S04 simulator at ${new Date().toISOString()}`);
    await run.save();

    try {
      const pid = startSimulation({
        runId: run.runId,
        onLog: (log) => {
          if (log) SimulationRun.updateOne({ runId: run.runId }, { $push: { logs: log } }).catch(console.error);
        },
        onExit: (code, signal) => {
          const status = code === 0 ? 'Passed' : 'Error';
          const message = `Simulator exited (${signal || `code ${code}`})`;
          SimulationRun.updateOne({ runId: run.runId }, { $set: { status }, $push: { logs: message } }).catch(console.error);
        }
      });
      return res.status(202).json({ success: true, pid, data: run });
    } catch (error) {
      run.status = 'Error';
      run.logs.push(error.message);
      await run.save();
      return res.status(503).json({ success: false, message: error.message });
    }
  } catch (error) {
    next(error);
  }
};
