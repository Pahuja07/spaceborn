import ScoreReport from '../models/ScoreReport.js';
import SimulationRun from '../models/SimulationRun.js';

// Export run details or score reports as JSON file download
export const exportRunAsJson = async (req, res, next) => {
  try {
    const { runId } = req.params;
    const runData = await SimulationRun.findOne({ runId });

    if (!runData) {
      return res.status(404).json({ success: false, message: 'Run data not found for export' });
    }

    const reportData = await ScoreReport.findOne({ runId });

    const exportPayload = {
      exportedAt: new Date().toISOString(),
      runDetails: runData,
      scoreMetrics: reportData || null
    };

    res.setHeader('Content-Type', 'application/json');
    res.setHeader('Content-Disposition', `attachment; filename=run-${runId}-export.json`);
    
    return res.status(200).send(JSON.stringify(exportPayload, null, 2));
  } catch (error) {
    next(error);
  }
};

// Export all scores as CSV
export const exportScoresAsCsv = async (req, res, next) => {
  try {
    const scores = await ScoreReport.find()
      .populate('stackId', 'name version')
      .populate('scenarioId', 'title');

    const csvHeader = 'Run ID,Stack Name,Stack Version,Scenario Title,Overall Score,Status,Executed At\n';
    const csvRows = scores.map(score => {
      const stackName = score.stackId?.name || 'N/A';
      const stackVersion = score.stackId?.version || 'N/A';
      const scenarioTitle = score.scenarioId?.title || 'N/A';
      return `"${score.runId}","${stackName}","${stackVersion}","${scenarioTitle}",${score.overallScore},"${score.status}","${score.executedAt}"`;
    }).join('\n');

    res.setHeader('Content-Type', 'text/csv');
    res.setHeader('Content-Disposition', 'attachment; filename=leaderboard-export.csv');

    return res.status(200).send(csvHeader + csvRows);
  } catch (error) {
    next(error);
  }
};
