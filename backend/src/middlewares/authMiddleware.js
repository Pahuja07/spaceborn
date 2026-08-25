import jwt from 'jsonwebtoken';
import User from '../models/User.js';

// Verify User JWT Token
export const protect = async (req, res, next) => {
  let token = req.headers.authorization?.startsWith('Bearer') ? req.headers.authorization.split(' ')[1] : null;

  if (!token) return res.status(401).json({ message: 'Not authorized, token missing' });

  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET);
    req.user = decoded;
    next();
  } catch (error) {
    return res.status(401).json({ message: 'Token verification failed' });
  }
};

// Verify Simulator API Key (Used when simulator sends logs/telemetry)
export const protectSimulator = async (req, res, next) => {
  const apiKey = req.headers['x-api-key'];

  if (!apiKey) return res.status(401).json({ message: 'Access denied. Missing Simulator API Key.' });

  try {
    const user = await User.findOne({ apiKey });
    if (!user) return res.status(403).json({ message: 'Invalid Simulator API Key.' });

    req.user = user;
    next();
  } catch (error) {
    next(error);
  }
};