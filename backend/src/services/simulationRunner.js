import { spawn } from 'node:child_process';

const runners = new Map();

const commandFor = (workspace) => [
  `source /opt/ros/humble/setup.bash && source "${workspace}/install/setup.bash" || exit $?`,
  'ros2 launch clearpath_gz simulation.launch.py & simulator_pid=$!',
  'ros2 launch clearpath_config rtabmap_rgbd_sync_launch.py nav2:=true explore:=true; launch_status=$?',
  'kill $simulator_pid 2>/dev/null || true',
  'wait $simulator_pid 2>/dev/null || true',
  'exit $launch_status'
].join('; ');

export const startSimulation = ({ runId, onLog, onExit }) => {
  if (runners.has(runId)) throw new Error('Simulation is already running');

  const workspace = process.env.SIMULATOR_WORKSPACE;
  if (!workspace) {
    throw new Error('SIMULATOR_WORKSPACE must point to the built S04 ROS 2 workspace');
  }

  const child = spawn('bash', ['-lc', commandFor(workspace)], {
    env: process.env,
    stdio: ['ignore', 'pipe', 'pipe']
  });

  const forward = (chunk) => onLog(chunk.toString().trim());
  child.stdout.on('data', forward);
  child.stderr.on('data', forward);
  child.once('exit', (code, signal) => {
    runners.delete(runId);
    onExit(code, signal);
  });
  child.once('error', (error) => {
    runners.delete(runId);
    onLog(error.message);
    onExit(1, null);
  });

  runners.set(runId, child);
  return child.pid;
};

export const stopSimulation = (runId) => {
  const child = runners.get(runId);
  if (!child) return false;
  child.kill('SIGTERM');
  return true;
};
